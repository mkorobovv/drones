package annealing

import (
	"math"

	"github.com/mkorobovv/drones/core/domain"
	"github.com/mkorobovv/drones/core/vars"
	"github.com/mkorobovv/drones/pkg/mathlib"
)

// Optimize запуск алгоритма оптимизации
func (a *Annealing) Optimize() domain.Output {
	bestControls := a.config.InitialControls
	bestScore := a.costService.Cost(a.config.InitialState, bestControls)

	currentControls := bestControls
	currentScore := bestScore

	temperature := func(i int) float64 {
		return a.config.InitialTemperature / (1 + 0.01*float64(i))
	}

	for i := range a.config.NumIterations {
		t := temperature(i)

		candidateControls := GetNeighbor(currentControls, a.config.StepSize)
		candidateScore := a.costService.Cost(a.config.InitialState, candidateControls)

		accept := candidateScore < currentScore ||
			vars.Seed.Float64() < math.Exp((currentScore-candidateScore)/t)

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
			neighbor[i][j] = controls[i][j] + vars.Seed.NormFloat64()*stepSize
		}

		// Ограничения на управление
		neighbor[i][0] = mathlib.Clamp(neighbor[i][0], -math.Pi/12, math.Pi/12)
		neighbor[i][1] = mathlib.Clamp(neighbor[i][1], -math.Pi, math.Pi)
		neighbor[i][2] = mathlib.Clamp(neighbor[i][2], -math.Pi/12, math.Pi/12)
		neighbor[i][3] = mathlib.Clamp(neighbor[i][3], 0, 12)
	}

	return neighbor
}
