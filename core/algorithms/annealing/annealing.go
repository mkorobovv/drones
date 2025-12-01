package annealing

import (
	"math"
	"math/rand"

	"github.com/mkorobovv/drones/core/domain"
	"github.com/mkorobovv/drones/pkg/mathlib"
)

type Config struct {
	NumIterations      int
	StepSize           float64
	InitialTemperature float64
	InitialState       domain.State
	InitialControls    []domain.Control
}

type Annealing struct {
	config  Config
	problem problem
}

type problem interface {
	Cost(state domain.State, controls []domain.Control) float64
}

func New(config Config, problem problem) *Annealing {
	return &Annealing{
		config:  config,
		problem: problem,
	}
}

// Optimize запуск алгоритма оптимизации
func (a *Annealing) Optimize() domain.Output {
	bestControls := a.config.InitialControls
	bestScore := a.problem.Cost(a.config.InitialState, bestControls)

	currentControls := bestControls
	currentScore := bestScore

	temperature := func(iter int) float64 {
		return a.config.InitialTemperature / (1 + 0.01*float64(iter))
	}

	for i := 0; i < a.config.NumIterations; i++ {
		t := temperature(i)

		candidateControls := GetNeighbor(currentControls, a.config.StepSize)
		candidateScore := a.problem.Cost(a.config.InitialState, currentControls)

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
