package annealing

import (
	"reflect"
	"testing"

	"github.com/mkorobovv/drones/core/domain"
)

func TestAnnealing_GetNeighbor(t *testing.T) {
	type fields struct {
		config      Config
		costService costService
	}
	type args struct {
		controls []domain.Control
		stepSize float64
	}
	tests := []struct {
		name   string
		fields fields
		args   args
		want   []domain.Control
	}{
		// TODO: Add test cases.
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			a := &Annealing{
				config:      tt.fields.config,
				costService: tt.fields.costService,
			}
			if got := a.GetNeighbor(tt.args.controls, tt.args.stepSize); !reflect.DeepEqual(got, tt.want) {
				t.Errorf("GetNeighbor() = %v, want %v", got, tt.want)
			}
		})
	}
}

func TestAnnealing_Optimize(t *testing.T) {
	type fields struct {
		config      Config
		costService costService
	}
	tests := []struct {
		name   string
		fields fields
		want   domain.Output
	}{
		// TODO: Add test cases.
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			a := &Annealing{
				config:      tt.fields.config,
				costService: tt.fields.costService,
			}
			if got := a.Optimize(); !reflect.DeepEqual(got, tt.want) {
				t.Errorf("Optimize() = %v, want %v", got, tt.want)
			}
		})
	}
}
