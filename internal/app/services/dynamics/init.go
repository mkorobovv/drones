package dynamics

import (
	"github.com/mkorobovv/drones/internal/app/domain"
)

type Dynamics struct {
	config domain.Dynamics
}

func New(cfg domain.Dynamics) *Dynamics {
	return &Dynamics{
		config: cfg,
	}
}
