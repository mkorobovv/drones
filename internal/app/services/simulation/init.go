package simulation

import (
	"math/rand"
	"time"

	"github.com/mkorobovv/drones/internal/app/domain"
)

type Simulation struct {
	config domain.Input
	cost   cost
	random *rand.Rand
}

type cost interface {
	Objective(state domain.State, controls []domain.Control) float64
}

func New(config domain.Input, c cost, rnd *rand.Rand) *Simulation {
	if c == nil {
		return nil
	}

	if rnd == nil {
		rnd = rand.New(rand.NewSource(time.Now().UnixNano()))
	}

	cfg := config

	if cfg.Annealing.StepSize <= 0 {
		cfg.Annealing.StepSize = 0.1
	}

	if cfg.Annealing.InitialTemperature <= 0 {
		cfg.Annealing.InitialTemperature = 1
	}

	if cfg.Annealing.CoolingRate <= 0 || cfg.Annealing.CoolingRate >= 1 {
		cfg.Annealing.CoolingRate = 0.95
	}

	if cfg.Annealing.MinTemperature <= 0 {
		cfg.Annealing.MinTemperature = 1e-3
	}

	if cfg.Annealing.MaxIterations <= 0 {
		cfg.Annealing.MaxIterations = cfg.Intervals * 10
		if cfg.Annealing.MaxIterations == 0 {
			cfg.Annealing.MaxIterations = 100
		}
	}

	return &Simulation{
		config: cfg,
		cost:   c,
		random: rnd,
	}
}

func (s *Simulation) ensureRandom() {
	if s.random == nil {
		s.random = rand.New(rand.NewSource(time.Now().UnixNano()))
	}
}
