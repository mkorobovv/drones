package mathlib

import (
	"math"
)

func PositivePart(value float64) float64 {
	if value > 0 {
		return value
	}
	return 0
}

func Heaviside(value float64) float64 {
	switch {
	case value > 0:
		return 1
	case value == 0:
		return 0.5
	default:
		return 0
	}
}

func Clamp(value, min, max float64) float64 {
	switch {
	case value < min:
		return min
	case value > max:
		return max
	default:
		return value
	}
}

func EuclideanDistance(current, target [6]float64) float64 {
	var distance float64

	for i := 0; i < len(current); i++ {
		distance += math.Pow(current[i]-target[i], 2)
	}

	return math.Sqrt(distance)
}
