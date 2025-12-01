package main

import (
	"fmt"
	"log"
	"math"
	"strings"
	"time"

	"github.com/mkorobovv/drones/core/algorithms/annealing"
	"github.com/mkorobovv/drones/core/domain"
	"github.com/mkorobovv/drones/core/models/quadrotor"
	"github.com/mkorobovv/drones/core/services/costservice"
	"github.com/mkorobovv/drones/core/vars"
)

func main() {
	input := domain.Input{
		NumIntervals:       15,
		NumIterations:      1_500_000,
		InitialTemperature: 200,
		Time:               5.6,
		Cylinders: []domain.Cylinder{
			{Coordinates: [3]float64{1.5, 0.0, 2.5}, Radius: 2.5},
			{Coordinates: [3]float64{6.5, 0.0, 7.5}, Radius: 2.5},
		},
		CylinderPenalty: 0.9,
		Windows: []domain.Window{
			{Coordinates: [3]float64{4.0, 0.0, 5.0}, Radius: 0.1},
		},
		WindowPenalty:   0.9,
		TerminalState:   domain.State{5, 5, 10, 0, 0, 0},
		TerminalPenalty: 1.6,
		InitialState:    domain.State{0, 0, 0, 0, 0, 0},
		StepSize:        0.01,
	}

	initialControls := make([]domain.Control, input.NumIntervals)

	for i := 0; i < input.NumIntervals; i++ {
		initialControls[i] = domain.Control{
			vars.Seed.Float64()*(math.Pi/9) - math.Pi/18, // u1 bound
			vars.Seed.Float64()*(math.Pi/3) - math.Pi/6,  // u2 bound
			vars.Seed.Float64()*(math.Pi/9) - math.Pi/18, // u3 bound
			vars.Seed.Float64()*4 + 8,                    // u4 bound
		}
	}

	costService := costservice.New(
		costservice.Config{
			NumIntervals:    input.NumIntervals,
			RK45Step:        input.Time / float64(input.NumIntervals),
			TerminalState:   input.TerminalState,
			TerminalPenalty: input.TerminalPenalty,
			Cylinders:       input.Cylinders,
			CylinderPenalty: input.CylinderPenalty,
			Windows:         input.Windows,
			WindowPenalty:   input.WindowPenalty,
		},
		quadrotor.Model,
	)

	annealing := annealing.New(
		annealing.Config{
			NumIterations:      input.NumIterations,
			StepSize:           input.StepSize,
			InitialTemperature: input.InitialTemperature,
			InitialState:       input.InitialState,
			InitialControls:    initialControls,
		},
		costService,
	)

	start := time.Now()

	result := annealing.Optimize()

	log.Printf("Optimization took: %v\n", time.Since(start))

	FormatResult(input.InitialState, result.BestControls, result.BestScore)
}

func FormatResult(x0 domain.State, bestU []domain.Control, bestScore float64) {
	var stateBuilder strings.Builder

	stateBuilder.WriteString("np.array([")
	for i := range x0 {
		if i > 0 {
			stateBuilder.WriteString(", ")
		}
		stateBuilder.WriteString(fmt.Sprintf("%v", x0[i]))
	}
	stateBuilder.WriteString("])")

	var builder strings.Builder

	builder.WriteString("np.array([\n")

	for j := 0; j < 4; j++ {
		builder.WriteString("\t[")
		for i := 0; i < len(bestU); i++ {
			if i > 0 {
				builder.WriteString(", ")
			}
			builder.WriteString(fmt.Sprintf("%v", bestU[i][j]))
		}
		builder.WriteString("],\n")
	}

	builder.WriteString("])")

	fmt.Printf("Начальное состояние: %s\n", stateBuilder.String())
	fmt.Printf("Лучшее управление: %s\n", builder.String())
	fmt.Printf("Лучшее значение функционала качества: %f\n", bestScore)
}
