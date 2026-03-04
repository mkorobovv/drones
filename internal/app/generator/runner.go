package generator

import (
	"context"
	"fmt"
	"log/slog"
	"math/rand"
	"runtime"
	"time"

	"github.com/ControlField/controlfield/internal/algorithms/annealing"
	"github.com/ControlField/controlfield/internal/domain"
	"github.com/ControlField/controlfield/internal/models/quadrotor"
	"github.com/ControlField/controlfield/internal/services/costservice"
	"golang.org/x/sync/errgroup"
)

type trajectoryRepository interface {
	CreateTrajectory(ctx context.Context) (id int64, err error)
	SaveStates(ctx context.Context, trajectories []domain.Trajectory) error
}

type scoreRepository interface {
	SaveScore(ctx context.Context, score domain.Score) error
}

type Runner struct {
	logger               *slog.Logger
	trajectoryRepository trajectoryRepository
	scoreRepository      scoreRepository
	numWorkers           int
}

func NewRunner(logger *slog.Logger, trajectoryRepository trajectoryRepository, scoreRepository scoreRepository) *Runner {
	workers := runtime.NumCPU()
	if workers < 1 {
		workers = 1
	}

	return &Runner{
		logger:               logger,
		trajectoryRepository: trajectoryRepository,
		scoreRepository:      scoreRepository,
		numWorkers:           workers,
	}
}

func (r *Runner) Run(ctx context.Context, cfg Config) error {
	if err := cfg.Validate(); err != nil {
		return fmt.Errorf("invalid config: %w", err)
	}

	r.logger.Info("Starting multiple optimizations", slog.Int("workers", r.numWorkers))
	tasks := neighborhoodGenerator(ctx, cfg.Tasks)

	group, groupCtx := errgroup.WithContext(ctx)

	for workerID := 0; workerID < r.numWorkers; workerID++ {
		seed := time.Now().UnixNano() + int64(workerID)*1_000_000
		group.Go(func() error {
			rnd := rand.New(rand.NewSource(seed))

			for task := range tasks {
				if err := r.processTask(groupCtx, cfg.Input, task, rnd); err != nil {
					return fmt.Errorf("process task %d: %w", task.ID, err)
				}
			}

			return nil
		})
	}

	if err := group.Wait(); err != nil {
		return fmt.Errorf("optimization failed: %w", err)
	}

	r.logger.Info("Optimization done successfully")

	return nil
}

func (r *Runner) processTask(ctx context.Context, input domain.Input, task Task, rnd *rand.Rand) error {
	controls := generateControls(input.NumIntervals, rnd)

	inp := input
	inp.InitialState = task.State

	trajectoryID, err := r.trajectoryRepository.CreateTrajectory(ctx)
	if err != nil {
		return err
	}

	costSvc := costservice.New(
		costservice.Config{
			NumIntervals:    inp.NumIntervals,
			RK45Step:        inp.Time / float64(inp.NumIntervals),
			TerminalState:   inp.TerminalState,
			TerminalPenalty: inp.TerminalPenalty,
			Cylinders:       inp.Cylinders,
			CylinderPenalty: inp.CylinderPenalty,
			Windows:         inp.Windows,
			WindowPenalty:   inp.WindowPenalty,
		},
		quadrotor.Model,
	)

	annealingAlg := annealing.New(
		annealing.Config{
			NumIterations:      inp.NumIterations,
			StepSize:           inp.StepSize,
			InitialTemperature: inp.InitialTemperature,
			InitialState:       inp.InitialState,
			InitialControls:    controls,
			Rnd:                rnd,
		},
		costSvc,
	)

	optimized := annealingAlg.Optimize()
	trajectory := costSvc.Trajectory(inp.InitialState, optimized.BestControls)

	states := make([]domain.Trajectory, 0, len(trajectory))
	for idx := range inp.NumIntervals {
		stateRow := domain.Trajectory{
			TrajectoryID: trajectoryID,
			PositionID:   int64(idx),
			State:        trajectory[idx].ToSlice(),
			Control:      optimized.BestControls[idx].ToSlice(),
		}

		states = append(states, stateRow)
	}

	if err := r.trajectoryRepository.SaveStates(ctx, states); err != nil {
		r.logger.Error(err.Error(), slog.String("source", "save trajectory"), slog.Int64("task_id", task.ID))
		return err
	}

	if err := r.scoreRepository.SaveScore(ctx, domain.Score{TrajectoryID: trajectoryID, Score: optimized.BestScore}); err != nil {
		r.logger.Error(err.Error(), slog.String("source", "save score"), slog.Int64("task_id", task.ID))
		return err
	}

	return nil
}
