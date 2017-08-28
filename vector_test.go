package physics

import "testing"

func TestVector_Equal(t *testing.T) {
	v1 := Vector{3.4, 9.8}
	v2 := Vector{3.4, 9.8}

	if !v1.Equal(v2) {
		t.Errorf("Not equal %v %v", v1, v2)
	}
}
