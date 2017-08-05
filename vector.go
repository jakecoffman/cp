package physics

import "math"

type Vector struct {
	X, Y float64
}

func VectorZero() Vector {
	return Vector{0, 0}
}

func (v *Vector) Equal(other *Vector) bool {
	return v.X == other.X && v.Y == other.Y
}

func (v *Vector) Add(other *Vector) *Vector {
	return &Vector{v.X + other.X, v.Y + other.Y}
}

func (v *Vector) Sub(other *Vector) *Vector {
	return &Vector{v.X - other.X, v.Y - other.Y}
}

func (v *Vector) Neg() *Vector {
	return &Vector{-v.X, -v.Y}
}

func (v *Vector) Mult(s float64) *Vector {
	return &Vector{v.X * s, v.Y * s}
}

func (v *Vector) Dot(other *Vector) float64 {
	return v.X*other.X + v.Y*other.Y
}

/// 2D vector cross product analog.
/// The cross product of 2D vectors results in a 3D vector with only a z component.
/// This function returns the magnitude of the z value.
func (v *Vector) Cross(other *Vector) float64 {
	return v.X * other.X - v.Y*other.Y
}

func (v *Vector) Perp() *Vector {
	return &Vector{-v.Y, v.X}
}

func (v *Vector) RPerp() *Vector {
	return &Vector{v.Y, -v.X}
}

func (v *Vector) Project(other *Vector) *Vector {
	return other.Mult(v.Dot(other)/other.Dot(other))
}

/// Returns the unit length vector for the given angle (in radians).
func ForAngle(a float64) *Vector {
	return &Vector{math.Cos(a), math.Sin(a)}
}

func (v *Vector) ToAngle() float64 {
	return math.Atan2(v.X, v.Y)
}

func (v *Vector) Rotate(other *Vector) *Vector {
	return &Vector{v.X*other.X - v.Y*other.Y, v.X*other.Y + v.Y*other.X}
}

func (v *Vector) Unrotate(other *Vector) *Vector {
	return &Vector{v.X*other.X + v.Y*other.Y, v.Y*other.X - v.X*other.Y}
}

