package costservice

import "github.com/mkorobovv/drones/core/domain"

type Config struct {
	NumIntervals int
	RK45Step     float64

	TerminalState   domain.State
	TerminalPenalty float64

	Cylinders       []domain.Cylinder
	CylinderPenalty float64

	Windows       []domain.Window
	WindowPenalty float64
}

type CostService struct {
	config       Config
	dynamicsFunc dynamicsFunc
}

type dynamicsFunc func(state domain.State, control domain.Control) domain.State

func New(config Config, dynamicsFunc dynamicsFunc) *CostService {
	return &CostService{
		config:       config,
		dynamicsFunc: dynamicsFunc,
	}
}
