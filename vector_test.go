package cp

import (
	"testing"
)

func TestVector_Normalize(t *testing.T) {
	v := Vector{}
	u := v.Normalize()
	if u.X != 0.0 || u.Y != 0.0 {
		t.Errorf("Expected zero vector, got %v", u)
	}
}
