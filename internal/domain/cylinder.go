package domain

import (
	"math"

	"github.com/ControlField/controlfield/pkg/mathlib"
)

type Cylinder struct {
	Coordinates [3]float64
	Radius      float64
}

func (c Cylinder) Distance(x1, x3 float64) float64 {
	return c.Radius - math.Hypot(x1-c.Coordinates[0], x3-c.Coordinates[2])
}

func (c Cylinder) Penalty(value float64) float64 {
	return mathlib.Heaviside(value)
}
