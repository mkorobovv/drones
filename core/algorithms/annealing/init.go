package annealing

import "github.com/mkorobovv/drones/core/domain"

type Config struct {
	NumIterations      int
	StepSize           float64
	InitialTemperature float64
	InitialState       domain.State
	InitialControls    []domain.Control
}

type Annealing struct {
	config      Config
	costService costService
}

type costService interface {
	Cost(state domain.State, controls []domain.Control) float64
}

func New(config Config, costService costService) *Annealing {
	return &Annealing{
		config:      config,
		costService: costService,
	}
}
