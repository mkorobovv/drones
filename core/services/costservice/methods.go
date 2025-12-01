package costservice

import (
	"math"

	"github.com/mkorobovv/drones/core/domain"
	"github.com/mkorobovv/drones/pkg/mathlib"
)

func (c *CostService) Cost(state domain.State, controls []domain.Control) float64 {
	var (
		cylinderPenalty float64
		windowPenalty   float64
		terminalPenalty float64
	)

	states := c.Trajectory(state, controls)

	for _, trajectoryState := range states {
		for _, cylinder := range c.config.Cylinders {
			cylinderDistance := cylinder.Distance(trajectoryState[0], trajectoryState[2])

			cylinderPenalty += c.config.CylinderPenalty * cylinder.Penalty(cylinderDistance)
		}
	}

	for _, window := range c.config.Windows {
		minDistance := math.Inf(1)

		for _, trajectoryState := range states {
			windowDistance := window.Distance(trajectoryState[0], trajectoryState[2])

			if windowDistance < minDistance {
				minDistance = windowDistance
			}
		}

		windowPenalty += c.config.WindowPenalty * window.Penalty(minDistance)
	}

	terminalPenalty = c.config.TerminalPenalty * mathlib.EuclideanDistance(states[c.config.NumIntervals], c.config.TerminalState)

	return c.config.RK45Step*float64(c.config.NumIntervals) + terminalPenalty + cylinderPenalty + windowPenalty
}

// Trajectory integrates the system dynamics for the provided controls.
func (c *CostService) Trajectory(state domain.State, controls []domain.Control) []domain.State {
	states := make([]domain.State, c.config.NumIntervals+1)
	states[0] = state

	for k := 0; k < c.config.NumIntervals; k++ {
		states[k+1] = c.RK45Step(states[k], controls[k])
	}

	return states
}

func (c *CostService) RK45Step(state domain.State, control domain.Control) domain.State {
	var newState domain.State

	k1 := c.dynamicsFunc(state, control)

	for i := 0; i < len(state); i++ {
		newState[i] = state[i] + 0.5*c.config.RK45Step*k1[i]
	}

	k2 := c.dynamicsFunc(newState, control)

	for i := 0; i < len(state); i++ {
		newState[i] = state[i] + 0.5*c.config.RK45Step*k2[i]
	}

	k3 := c.dynamicsFunc(newState, control)

	for i := 0; i < len(state); i++ {
		newState[i] = state[i] + c.config.RK45Step*k3[i]
	}

	k4 := c.dynamicsFunc(newState, control)

	var finalState domain.State

	for i := 0; i < len(state); i++ {
		finalState[i] = state[i] + (c.config.RK45Step/6)*(k1[i]+2*k2[i]+2*k3[i]+k4[i])
	}

	return finalState
}
