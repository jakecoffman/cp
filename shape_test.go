package cp

import (
	"math"
	"testing"
)

func TestShapeMass(t *testing.T) {
	body := NewBody(0, 0)
	circle := NewCircle(body, 5, Vector{0, 0})

	mass := 10.0
	circle.SetMass(mass)
	body.AddShape(circle)

	if circle.Mass() != mass {
		t.Fail()
	}
}

func TestShapeCircleArea(t *testing.T) {
	body := NewBody(0, 0)
	circle := NewCircle(body, 2, Vector{0, 0})

	if circle.Area() != 4 * math.Pi {
		t.Fail()
	}
}

func TestShapeCircleDensity(t *testing.T) {
	body := NewBody(0, 0)
	circle := NewCircle(body, 1, Vector{0, 0})

	circle.SetMass(math.Pi)

	if circle.Density() != 1.0 {
		t.Fail()
	}
}
