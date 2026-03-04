package generator

import (
	"fmt"

	"github.com/ControlField/controlfield/internal/domain"
	"github.com/ControlField/controlfield/internal/infrastructure/postgres"
)

type TaskConfig struct {
	BaseState domain.State
	Radius    float64
	Steps     int
}

type Config struct {
	Database postgres.Config
	Input    domain.Input
	Tasks    TaskConfig
}

func (c Config) Validate() error {
	switch {
	case c.Input.NumIntervals <= 0:
		return fmt.Errorf("num intervals must be positive")
	case c.Input.NumIterations <= 0:
		return fmt.Errorf("num iterations must be positive")
	case c.Input.Time <= 0:
		return fmt.Errorf("time must be positive")
	case c.Input.StepSize <= 0:
		return fmt.Errorf("step size must be positive")
	case c.Tasks.Steps <= 0:
		return fmt.Errorf("task steps must be positive")
	case c.Tasks.Radius < 0:
		return fmt.Errorf("task radius must be non-negative")
	}

	return nil
}

func DefaultConfig() Config {
	return Config{
		Database: postgres.Config{
			Host:     "localhost",
			Port:     5432,
			User:     "postgres",
			Password: "postgres",
			Database: "trajectory",
		},
		Input: domain.Input{
			NumIntervals:       15,
			NumIterations:      200_000,
			InitialTemperature: 200,
			Time:               5.6,
			Cylinders: []domain.Cylinder{
				{Coordinates: [3]float64{1.5, 0.0, 2.5}, Radius: 2.5},
				{Coordinates: [3]float64{6.5, 0.0, 7.5}, Radius: 2.5},
			},
			CylinderPenalty: 0.9,
			Windows: []domain.Window{
				{Coordinates: [3]float64{4.0, 0.0, 5.0}, Radius: 0.5},
			},
			WindowPenalty:   1.6,
			TerminalState:   domain.State{5, 5, 10, 0, 0, 0},
			TerminalPenalty: 0.9,
			StepSize:        0.01,
		},
		Tasks: TaskConfig{
			BaseState: domain.State{0, 0, 0, 0, 0, 0},
			Radius:    0.45,
			Steps:     5,
		},
	}
}
