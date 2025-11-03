package dynamics

import "github.com/mkorobovv/drones/internal/app/domain"

func (d *Dynamics) RK4Step(state domain.State, control domain.Control) domain.State {
	var newState domain.State

	k1 := d.Drone(state, control)

	for i := 0; i < len(state); i++ {
		newState[i] = state[i] + 0.5*d.config.RK4Step*k1[i]
	}

	k2 := d.Drone(newState, control)

	for i := 0; i < len(state); i++ {
		newState[i] = state[i] + 0.5*d.config.RK4Step*k2[i]
	}

	k3 := d.Drone(newState, control)

	for i := 0; i < len(state); i++ {
		newState[i] = state[i] + d.config.RK4Step*k3[i]
	}

	k4 := d.Drone(newState, control)

	var finalState domain.State

	for i := 0; i < len(state); i++ {
		finalState[i] = state[i] + (d.config.RK4Step/6)*(k1[i]+2*k2[i]+2*k3[i]+k4[i])
	}

	return finalState
}
