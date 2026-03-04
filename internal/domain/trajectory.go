package domain

import "time"

type Trajectory struct {
	TrajectoryID int64
	PositionID   int64
	State        []float64
	Control      []float64
	CreatedAt    time.Time
}
