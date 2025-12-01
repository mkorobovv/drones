package domain

import (
	"math"

	"github.com/mkorobovv/drones/pkg/mathlib"
)

type State [6]float64

type Control [4]float64

type Window struct {
	Coordinates [3]float64
	Radius      float64
}

func (w Window) Distance(x1, x3 float64) float64 {
	return math.Hypot(x1-w.Coordinates[0], x3-w.Coordinates[2]) - w.Radius
}

func (w Window) Penalty(value float64) float64 {
	return mathlib.PositivePart(value)
}

type Cylinder struct {
	Coordinates [3]float64
	Radius      float64
}

func (c Cylinder) Distance(x1, x3 float64) float64 {
	return c.Radius - math.Hypot(x1-c.Coordinates[0], x3-c.Coordinates[2])
}

func (c Cylinder) Penalty(value float64) float64 {
	return mathlib.PositivePart(value)
}
