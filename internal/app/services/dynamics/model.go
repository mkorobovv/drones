package dynamics

import (
	"math"

	"github.com/mkorobovv/drones/internal/app/domain"
)

const gravity = 9.81

func (d *Dynamics) Drone(state domain.State, control domain.Control) domain.State {
	return domain.State{
		state[3],
		state[4],
		state[5],
		(math.Cos(control[2])*math.Sin(control[1])*math.Cos(control[0]) + math.Sin(control[2])*math.Sin(control[0])) * control[3],
		(math.Sin(control[2])*math.Sin(control[1])*math.Cos(control[0]) + math.Cos(control[2])*math.Sin(control[0])) * control[3],
		control[3]*math.Cos(control[1])*math.Cos(control[0]) - gravity,
	}
}
