package domain

type Problem interface {
	Cost(state State, controls []Control) float64
}
