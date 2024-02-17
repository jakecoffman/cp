package cp

import (
	"fmt"
	"math"
)

type Vector struct {
	X, Y float64
}

func (v Vector) String() string {
	return fmt.Sprintf("%f,%f", v.X, v.Y)
}

func (v Vector) Equal(other Vector) bool {
	return v.X == other.X && v.Y == other.Y
}

func (v Vector) Add(other Vector) Vector {
	return Vector{v.X + other.X, v.Y + other.Y}
}

func (v Vector) Sub(other Vector) Vector {
	return Vector{v.X - other.X, v.Y - other.Y}
}

func (v Vector) Neg() Vector {
	return Vector{-v.X, -v.Y}
}

func (v Vector) Mult(s float64) Vector {
	return Vector{v.X * s, v.Y * s}
}

func (v Vector) Dot(other Vector) float64 {
	return v.X*other.X + v.Y*other.Y
}

// Cross calculates the 2D vector cross product analog.
// The cross product of 2D vectors results in a 3D vector with only a z component.
// This function returns the magnitude of the z value.
func (v Vector) Cross(other Vector) float64 {
	return v.X*other.Y - v.Y*other.X
}

func (v Vector) Perp() Vector {
	return Vector{-v.Y, v.X}
}

func (v Vector) ReversePerp() Vector {
	return Vector{v.Y, -v.X}
}

func (v Vector) Project(other Vector) Vector {
	return other.Mult(v.Dot(other) / other.Dot(other))
}

// ForAngle returns the unit length vector for the given angle (in radians).
func ForAngle(a float64) Vector {
	return Vector{math.Cos(a), math.Sin(a)}
}

func (v Vector) ToAngle() float64 {
	return math.Atan2(v.Y, v.X)
}

func (v Vector) Rotate(other Vector) Vector {
	return Vector{v.X*other.X - v.Y*other.Y, v.X*other.Y + v.Y*other.X}
}

func (v Vector) Unrotate(other Vector) Vector {
	return Vector{v.X*other.X + v.Y*other.Y, v.Y*other.X - v.X*other.Y}
}

func (v Vector) LengthSq() float64 {
	return v.Dot(v)
}

func (v Vector) Length() float64 {
	return math.Sqrt(v.Dot(v))
}

func (v Vector) Lerp(other Vector, t float64) Vector {
	return v.Mult(1.0 - t).Add(other.Mult(t))
}

func (v Vector) Normalize() Vector {
	return v.Mult(1.0 / (v.Length() + 1e-15))
}

func (v Vector) SLerp(other Vector, t float64) Vector {
	dot := v.Normalize().Dot(other.Normalize())
	omega := math.Acos(Clamp(dot, -1, 1))

	if omega < 1e-3 {
		return v.Lerp(other, t)
	}

	denom := 1.0 / math.Sin(omega)
	return v.Mult(math.Sin((1.0-t)*omega) * denom).Add(other.Mult(math.Sin(t*omega) * denom))
}

func Clamp(f, min, max float64) float64 {
	if f > min {
		return math.Min(f, max)
	} else {
		return math.Min(min, max)
	}
}

func Clamp01(f float64) float64 {
	return math.Max(0, math.Min(f, 1))
}

func Lerp(f1, f2, t float64) float64 {
	return f1*(1.0-t) + f2*t
}

func LerpConst(f1, f2, d float64) float64 {
	return f1 + Clamp(f2-f1, -d, d)
}

func (v Vector) SlerpConst(other Vector, a float64) Vector {
	dot := v.Normalize().Dot(other.Normalize())
	omega := math.Acos(Clamp(dot, -1, 1))
	return v.SLerp(other, math.Min(a, omega)/omega)
}

func (v Vector) Clamp(length float64) Vector {
	if v.Dot(v) > length*length {
		return v.Normalize().Mult(length)
	}
	return Vector{v.X, v.Y}
}

func (v Vector) LerpConst(other Vector, d float64) Vector {
	return v.Add(other.Sub(v).Clamp(d))
}

func (v Vector) Distance(other Vector) float64 {
	return v.Sub(other).Length()
}

func (v Vector) DistanceSq(other Vector) float64 {
	return v.Sub(other).LengthSq()
}

func (v Vector) Near(other Vector, d float64) bool {
	return v.DistanceSq(other) < d*d
}

// Collision related below

func (v Vector) PointGreater(b, c Vector) bool {
	return (b.Y-v.Y)*(v.X+b.X-2*c.X) > (b.X-v.X)*(v.Y+b.Y-2*c.Y)
}

func (v Vector) CheckAxis(v1, p, n Vector) bool {
	return p.Dot(n) <= math.Max(v.Dot(n), v1.Dot(n))
}

func (v Vector) ClosestT(b Vector) float64 {
	delta := b.Sub(v)
	return -Clamp(delta.Dot(v.Add(b))/delta.LengthSq(), -1.0, 1.0)
}

func (v Vector) LerpT(b Vector, t float64) Vector {
	ht := 0.5 * t
	return v.Mult(0.5 - ht).Add(b.Mult(0.5 + ht))
}

func (v Vector) ClosestDist(v1 Vector) float64 {
	return v.LerpT(v1, v.ClosestT(v1)).LengthSq()
}

func (v Vector) ClosestPointOnSegment(a, b Vector) Vector {
	delta := a.Sub(b)
	t := Clamp01(delta.Dot(v.Sub(b)) / delta.LengthSq())
	return b.Add(delta.Mult(t))
}

func (v Vector) Clone() Vector {
	return Vector{v.X, v.Y}
}
