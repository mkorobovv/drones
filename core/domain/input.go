package domain

type Input struct {
	N, M                   int
	NumIntervals           int
	NumIterations          int
	T, Dt                  float64
	Alpha1, Alpha2, Alpha3 float64
	Cylinders              []Cylinder
	Window                 Window
	InitialState           State
	InitialControls        []Control
}
