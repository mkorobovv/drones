package domain

import (
	"math"
)

type Window struct {
	Coordinates [3]float64
	Radius      float64
}

func (w Window) Distance(x1, x3 float64) float64 {
	return math.Hypot(x1-w.Coordinates[0], x3-w.Coordinates[2]) - w.Radius
}

func (w Window) Penalty(value float64) float64 {
	// Если value <= 0, значит мы внутри окна (Hypot - Radius <= 0)
	if value <= 0 {
		return 0
	}
	// Если снаружи, возвращаем квадрат расстояния до границы
	// Это дает алгоритму понять: "чем ближе, тем меньше штраф"
	return value * value
}
