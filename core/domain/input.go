package domain

type Input struct {
	NumIntervals       int
	NumIterations      int
	InitialTemperature float64
	Time               float64
	Cylinders          []Cylinder
	CylinderPenalty    float64
	Windows            []Window
	WindowPenalty      float64
	TerminalState      State
	TerminalPenalty    float64
	StepSize           float64
	InitialState       State
	InitialControls    []Control
}
