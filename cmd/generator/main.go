package main

import (
	"context"
	"log/slog"
	"os"

	"github.com/ControlField/controlfield/internal/app/generator"
	"github.com/ControlField/controlfield/internal/infrastructure/postgres"
	"github.com/ControlField/controlfield/internal/repositories/scorerepository"
	"github.com/ControlField/controlfield/internal/repositories/trajectoryrepository"
)

func main() {
	ctx := context.Background()
	logger := slog.New(slog.NewJSONHandler(os.Stdout, nil))

	cfg := generator.DefaultConfig()
	if err := cfg.Validate(); err != nil {
		logger.Error("invalid generator config", slog.String("error", err.Error()))
		os.Exit(1)
	}

	pgpool, err := postgres.New(ctx, cfg.Database)
	if err != nil {
		logger.Error("failed to initialize postgres", slog.String("error", err.Error()))
		os.Exit(1)
	}
	defer pgpool.Close()

	scoreRepository := scorerepository.New(pgpool)
	trajectoryRepository := trajectoryrepository.New(pgpool)

	runner := generator.NewRunner(logger, trajectoryRepository, scoreRepository)
	if err := runner.Run(ctx, cfg); err != nil {
		logger.Error("generator stopped with error", slog.String("error", err.Error()))
		os.Exit(1)
	}
}
