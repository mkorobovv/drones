package cylinder

import (
	"math"

	"github.com/mkorobovv/drones/internal/pkg/mathlib"
)

// Cylinder is constraint for drone. Drone must fly without touching the cylinders
type Cylinder struct {
	X1, X3             float64
	Radius             float64
	PenaltyCoefficient float64
}

func (c Cylinder) Distance(x1, x3 float64) float64 {
	return c.Radius - math.Hypot(x1-c.X1, x3-c.X3)
}

func (c Cylinder) Penalty(value float64) float64 {
	return c.PenaltyCoefficient * mathlib.PositivePart(value)
}
