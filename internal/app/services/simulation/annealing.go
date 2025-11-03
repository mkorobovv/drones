package simulation

import (
	"math"

	"github.com/mkorobovv/drones/internal/app/domain"
	"github.com/mkorobovv/drones/internal/pkg/mathlib"
)

func (s *Simulation) Annealing(state domain.State, controls []domain.Control) ([]domain.Control, float64) {
	if s == nil || s.cost == nil {
		return nil, math.Inf(1)
	}

	if len(controls) != s.config.Intervals {
		return nil, math.Inf(1)
	}

	s.ensureRandom()

	current := cloneControls(controls)
	best := cloneControls(controls)

	currentCost := s.cost.Objective(state, current)
	bestCost := currentCost

	cfg := s.config.Annealing

	temperature := cfg.InitialTemperature
	minTemperature := cfg.MinTemperature
	coolingRate := cfg.CoolingRate
	iterations := cfg.MaxIterations

	if temperature <= 0 {
		temperature = 1
	}

	if minTemperature <= 0 {
		minTemperature = 1e-3
	}

	if coolingRate <= 0 || coolingRate >= 1 {
		coolingRate = 0.95
	}

	if iterations <= 0 {
		iterations = s.config.Intervals * 10
		if iterations == 0 {
			iterations = 100
		}
	}

	for i := 0; i < iterations && temperature > minTemperature; i++ {
		candidate := s.Neighbor(current)
		candidateCost := s.cost.Objective(state, candidate)
		delta := candidateCost - currentCost

		if delta <= 0 || s.accept(delta, temperature) {
			current = candidate
			currentCost = candidateCost

			if currentCost < bestCost {
				best = cloneControls(current)
				bestCost = currentCost
			}
		}

		temperature *= coolingRate
	}

	return best, bestCost
}

func (s *Simulation) Neighbor(controls []domain.Control) []domain.Control {
	s.ensureRandom()

	neighbor := make([]domain.Control, len(controls))
	stepSize := s.config.Annealing.StepSize

	if stepSize <= 0 {
		stepSize = 0.1
	}

	for i, control := range controls {
		for j := range control {
			neighbor[i][j] = controls[i][j] + s.random.NormFloat64()*stepSize
		}

		neighbor[i][0] = mathlib.Clamp(neighbor[i][0], s.config.U1Bound.Min, s.config.U1Bound.Max)
		neighbor[i][1] = mathlib.Clamp(neighbor[i][1], s.config.U2Bound.Min, s.config.U2Bound.Max)
		neighbor[i][2] = mathlib.Clamp(neighbor[i][2], s.config.U3Bound.Min, s.config.U3Bound.Max)
		neighbor[i][3] = mathlib.Clamp(neighbor[i][3], s.config.U4Bound.Min, s.config.U4Bound.Max)
	}

	return neighbor
}

func (s *Simulation) accept(delta, temperature float64) bool {
	if temperature <= 0 {
		return false
	}

	probability := math.Exp(-delta / temperature)

	if math.IsNaN(probability) || probability <= 0 {
		return false
	}

	if probability >= 1 {
		return true
	}

	return s.random.Float64() < probability
}

func cloneControls(src []domain.Control) []domain.Control {
	dst := make([]domain.Control, len(src))
	copy(dst, src)
	return dst
}
