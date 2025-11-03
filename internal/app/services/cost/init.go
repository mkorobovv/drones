package cost

import (
	"github.com/mkorobovv/drones/internal/app/domain"
)

type Cost struct {
	config   domain.Input
	dynamics dynamics
}

type dynamics interface {
	Drone(state domain.State, control domain.Control) domain.State
	RK4Step(state domain.State, control domain.Control) domain.State
}

func New(config domain.Input, dyn dynamics) *Cost {
	return &Cost{
		config:   config,
		dynamics: dyn,
	}
}
