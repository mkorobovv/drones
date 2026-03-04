package generator

import (
	"context"
	"math"
	"math/rand"

	"github.com/ControlField/controlfield/internal/domain"
)

type Task struct {
	ID    int64
	State domain.State
}

func neighborhoodGenerator(ctx context.Context, cfg TaskConfig) <-chan Task {
	out := make(chan Task)

	getCoords := func(baseVal float64, radius float64, n int) []float64 {
		if n <= 1 {
			return []float64{baseVal}
		}

		coords := make([]float64, n)
		minVal := baseVal - radius
		maxVal := baseVal + radius
		stepSize := (maxVal - minVal) / float64(n-1)

		for i := 0; i < n; i++ {
			coords[i] = minVal + float64(i)*stepSize
		}

		return coords
	}

	go func() {
		defer close(out)
		var taskID int64 = 1

		xs := getCoords(cfg.BaseState[0], cfg.Radius, cfg.Steps)
		ys := getCoords(cfg.BaseState[1], cfg.Radius, cfg.Steps)
		zs := getCoords(cfg.BaseState[2], cfg.Radius, cfg.Steps)

		for _, x := range xs {
			for _, y := range ys {
				for _, z := range zs {
					task := Task{
						ID:    taskID,
						State: domain.State{x, y, z, cfg.BaseState[3], cfg.BaseState[4], cfg.BaseState[5]},
					}

					select {
					case <-ctx.Done():
						return
					case out <- task:
						taskID++
					}
				}
			}
		}
	}()

	return out
}

func generateControls(numIntervals int, rnd *rand.Rand) []domain.Control {
	initialControls := make([]domain.Control, numIntervals)

	for i := 0; i < numIntervals; i++ {
		initialControls[i] = domain.Control{
			rnd.Float64()*(math.Pi/9) - math.Pi/18,
			rnd.Float64()*(math.Pi/3) - math.Pi/6,
			rnd.Float64()*(math.Pi/9) - math.Pi/18,
			rnd.Float64()*4 + 8,
		}
	}

	return initialControls
}
