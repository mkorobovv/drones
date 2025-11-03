package window

import (
	"math"

	"github.com/mkorobovv/drones/internal/pkg/mathlib"
)

// Window is constraint for drone. Drone must fly through the window
type Window struct {
	X1, X3             float64
	Radius             float64
	PenaltyCoefficient float64
}

func (w Window) Distance(x1, x3 float64) float64 {
	return math.Hypot(x1-w.X1, x3-w.X3) - w.Radius
}

func (w Window) Penalty(value float64) float64 {
	return w.PenaltyCoefficient * mathlib.PositivePart(value)
}
