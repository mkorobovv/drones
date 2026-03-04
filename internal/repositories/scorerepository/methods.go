package scorerepository

import (
	"context"
	"time"

	"github.com/ControlField/controlfield/internal/domain"
)

func (repo *ScoreRepository) SaveScore(ctx context.Context, score domain.Score) error {
	ctx, cancel := context.WithTimeout(ctx, 10*time.Second)
	defer cancel()

	const query = `INSERT INTO scores (trajectory_id, score) VALUES ($1, $2)`

	_, err := repo.pool.Exec(ctx, query, score.TrajectoryID, score.Score)
	if err != nil {
		return err
	}

	return nil
}
