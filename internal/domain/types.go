package domain

type State [6]float64

func (s State) ToSlice() []float64 {
	st := make([]float64, len(s))

	for i := range s {
		st[i] = s[i]
	}

	return st
}

type Control [4]float64

func (c Control) ToSlice() []float64 {
	ct := make([]float64, len(c))

	for i := range c {
		ct[i] = c[i]
	}

	return ct
}
