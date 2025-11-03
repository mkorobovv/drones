package cost

import (
	"github.com/mkorobovv/drones/internal/app/domain"
	"github.com/mkorobovv/drones/internal/pkg/mathlib"
)

func (c *Cost) Objective(state domain.State, controls []domain.Control) float64 {
	intervals := c.config.Intervals

	states := make([]domain.State, intervals+1)
	states[0] = state

	for k := 0; k < intervals; k++ {
		states[k+1] = c.dynamics.RK4Step(states[k], controls[k])
	}

	var (
		cylinderPenalty float64
		windowPenalty   float64
		terminalPenalty float64
	)

	for k := 0; k < intervals; k++ {
		for _, cylinder := range c.config.Cylinders {
			cylinderDistance := cylinder.Distance(states[k][0], states[k][2])

			cylinderPenalty += cylinder.Penalty(cylinderDistance)
		}

		for _, window := range c.config.Windows {
			windowDistance := window.Distance(states[k][0], states[k][2])

			windowPenalty += window.Penalty(windowDistance)
		}
	}

	terminalPenalty = c.config.TargetPenaltyCoefficient * mathlib.EuclideanDistance(states[intervals], c.config.Target)

	return terminalPenalty + cylinderPenalty + windowPenalty
}
