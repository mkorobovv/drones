package annealing

import (
	"math"
	"math/rand"

	"github.com/mkorobovv/drones/core/domain"
	"github.com/mkorobovv/drones/pkg/mathlib"
)

type Config struct {
	N, M                   int
	T, Dt                  float64
	Alpha1, Alpha2, Alpha3 float64
	G                      float64
	StepSize               float64
	Cylinders              []domain.Cylinder
	Window                 domain.Window
	InitialTemperature     float64
	InitialState           domain.State
	InitialControls        []domain.Control
}

type Annealing struct {
	config Config
}

// Optimize запуск алгоритма оптимизации
func (a *Annealing) Optimize(problem domain.Problem) domain.Output {
	bestControls := a.config.InitialControls
	bestScore := problem.Cost(a.config.InitialState, bestControls)

	currentControls := bestControls
	currentScore := bestScore

	temperature := func(iter int) float64 {
		return a.config.InitialTemperature / (1 + 0.01*float64(iter))
	}

	for i := 0; i < a.config.N; i++ {
		t := temperature(i)

		candidateControls := GetNeighbor(currentControls, a.config.StepSize)
		candidateScore := problem.Cost(a.config.InitialState, currentControls)

		accept := candidateScore < currentScore ||
			rand.Float64() < math.Exp((currentScore-candidateScore)/t)

		if accept {
			currentControls = candidateControls
			currentScore = candidateScore

			if candidateScore < bestScore {
				bestControls = candidateControls
				bestScore = candidateScore
			}
		}
	}

	return domain.Output{
		BestControls: bestControls,
		BestScore:    bestScore,
	}
}

// GetNeighbor генерация соседнего решения
func GetNeighbor(controls []domain.Control, stepSize float64) []domain.Control {
	neighbor := make([]domain.Control, len(controls))

	for i := range controls {
		for j := 0; j < 4; j++ {
			neighbor[i][j] = controls[i][j] + rand.NormFloat64()*stepSize
		}

		// Ограничения на управление
		neighbor[i][0] = mathlib.Clamp(neighbor[i][0], -math.Pi/12, math.Pi/12)
		neighbor[i][1] = mathlib.Clamp(neighbor[i][1], -math.Pi, math.Pi)
		neighbor[i][2] = mathlib.Clamp(neighbor[i][2], -math.Pi/12, math.Pi/12)
		neighbor[i][3] = mathlib.Clamp(neighbor[i][3], 0, 12)
	}

	return neighbor
}
