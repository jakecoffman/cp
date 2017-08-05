package physics

import "math"

type Vector struct {
	X, Y float64
}

func VectorZero() *Vector {
	return &Vector{0, 0}
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
	return v.X*other.X - v.Y*other.Y
}

func (v *Vector) Perp() *Vector {
	return &Vector{-v.Y, v.X}
}

func (v *Vector) RPerp() *Vector {
	return &Vector{v.Y, -v.X}
}

func (v *Vector) Project(other *Vector) *Vector {
	return other.Mult(v.Dot(other) / other.Dot(other))
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

func (v *Vector) LengthSq() float64 {
	return v.Dot(v)
}

func (v *Vector) Length() float64 {
	return math.Sqrt(v.Dot(v))
}

func (v *Vector) Lerp(other *Vector, t float64) *Vector {
	return v.Mult(1 - t).Add(other.Mult(t))
}

func (v *Vector) Normalize() *Vector {
	// if this throws div by zero errors, then add a really small float64 to it
	return v.Mult(1 / v.Length())
}

func (v *Vector) SLerp(other *Vector, t float64) *Vector {
	dot := v.Normalize().Dot(other.Normalize())
	omega := math.Acos(Clamp(dot, -1, 1))

	if omega < 1e-3 {
		return v.Lerp(other, t)
	}

	denom := 1 / math.Sin(omega)
	return v.Mult(math.Sin((1-t)*omega) * denom).Add(other.Mult(math.Sin(t*omega) * denom))
}

func Clamp(f, min, max float64) float64 {
	return math.Min(math.Max(f, min), max)
}

func (v *Vector) SlerpConst(other *Vector, a float64) *Vector {
	dot := v.Normalize().Dot(other.Normalize())
	omega := math.Acos(Clamp(dot, -1, 1))
	return v.SLerp(other, math.Min(a, omega)/omega)
}

func (v *Vector) Clamp(length float64) *Vector {
	if v.Dot(v) > length*length {
		return v.Normalize().Mult(length)
	}
	return &Vector{v.X, v.Y}
}

func (v *Vector) LerpConst(other *Vector, d float64) *Vector {
	return v.Add(other.Sub(v).Clamp(d))
}

func (v *Vector) Distance(other *Vector) float64 {
	return v.Sub(other).Length()
}

func (v *Vector) DistanceSq(other *Vector) float64 {
	return v.Sub(other).LengthSq()
}

func (v *Vector) Near(other *Vector, d float64) bool {
	return v.DistanceSq(other) < d*d
}
