package domain

import (
	"github.com/mkorobovv/drones/internal/app/domain/cylinder"
	"github.com/mkorobovv/drones/internal/app/domain/window"
)

type Input struct {
	Windows   []window.Window
	Cylinders []cylinder.Cylinder

	// Start is initial point where drone simulation starts
	Start State

	U1Bound Bound
	U2Bound Bound
	U3Bound Bound
	U4Bound Bound

	// Target is terminal state where drone simulation ends
	Target                   State
	TargetPenaltyCoefficient float64

	Dynamics Dynamics

	Intervals int

	Annealing Annealing
}

type Bound struct {
	Min, Max float64
}

type Dynamics struct {
	RK4Step     float64
	Iterations  int
	Temperature float64
}

type Annealing struct {
	StepSize           float64
	InitialTemperature float64
	CoolingRate        float64
	MinTemperature     float64
	MaxIterations      int
}
