package costservice

import (
	"testing"

	"github.com/ControlField/controlfield/internal/domain"
)

func TestTrajectoryHasExpectedLengthAndInitialState(t *testing.T) {
	svc := New(
		Config{
			NumIntervals: 4,
			RK45Step:     1,
		},
		func(state domain.State, control domain.Control) domain.State {
			return domain.State{control[0], 0, 0, 0, 0, 0}
		},
	)

	initial := domain.State{0, 0, 0, 0, 0, 0}
	controls := []domain.Control{
		{1, 0, 0, 0},
		{1, 0, 0, 0},
		{1, 0, 0, 0},
		{1, 0, 0, 0},
	}

	traj := svc.Trajectory(initial, controls)

	if len(traj) != 5 {
		t.Fatalf("expected trajectory length 5, got %d", len(traj))
	}

	if traj[0] != initial {
		t.Fatalf("expected first trajectory point to equal initial state")
	}
}
