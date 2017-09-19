package cp

import (
	"math"
	"fmt"
)

type BB struct {
	L, B, R, T float64
}

func (bb BB) String() string {
	return fmt.Sprintf("%v %v %v %v", bb.L, bb.T, bb.R, bb.B)
}

func NewBBForExtents(c Vector, hw, hh float64) BB {
	return BB{
		L: c.X - hw,
		B: c.Y - hh,
		R: c.X + hw,
		T: c.Y + hh,
	}
}

func NewBBForCircle(p Vector, r float64) BB {
	return NewBBForExtents(p, r, r)
}

func (a BB) Intersects(b BB) bool {
	return a.L <= b.R && b.L <= a.R && a.B <= b.T && b.B <= a.T
}

func (bb BB) Contains(other BB) bool {
	return bb.L <= other.L && bb.R >= other.R && bb.B <= other.B && bb.T >= other.T
}

func (bb BB) ContainsVect(v Vector) bool {
	return bb.L <= v.X && bb.R >= v.X && bb.B <= v.Y && bb.T >= v.Y
}

func (a BB) Merge(b BB) BB {
	return BB{
		math.Min(a.L, b.L),
		math.Min(a.B, b.B),
		math.Max(a.R, b.R),
		math.Max(a.T, b.T),
	}
}

func (bb BB) Expand(v Vector) BB {
	return BB{
		math.Min(bb.L, v.X),
		math.Min(bb.B, v.Y),
		math.Max(bb.R, v.X),
		math.Max(bb.T, v.Y),
	}
}

func (bb BB) Center() Vector {
	return Vector{bb.L, bb.B}.Lerp(Vector{bb.R, bb.T}, 0.5)
}

func (bb BB) Area() float64 {
	return (bb.R - bb.L) * (bb.T - bb.B)
}

func (a BB) MergedArea(b BB) float64 {
	return (math.Max(a.R, b.R) - math.Min(a.L, b.L)) * (math.Max(a.T, b.T) - math.Min(a.B, b.B))
}

func (bb BB) SegmentQuery(a, b Vector) float64 {
	delta := b.Sub(a)
	tmin := -INFINITY
	tmax := INFINITY

	if delta.X == 0 {
		if a.X < bb.L || bb.R < a.X {
			return INFINITY
		}
	} else {
		t1 := (bb.L - a.X) / delta.X
		t2 := (bb.R - a.X) / delta.X
		tmin = math.Max(tmin, math.Min(t1, t2))
		tmax = math.Min(tmax, math.Max(t1, t2))
	}

	if delta.Y == 0 {
		if a.Y < bb.B || bb.T < a.Y {
			return INFINITY
		}
	} else {
		t1 := (bb.B - a.Y) / delta.Y
		t2 := (bb.T - a.Y) / delta.Y
		tmin = math.Max(tmin, math.Min(t1, t2))
		tmax = math.Min(tmax, math.Max(t1, t2))
	}

	if tmin <= tmax && 0 <= tmax && tmin <= 1.0 {
		return math.Max(tmin, 0.0)
	} else {
		return INFINITY
	}
}

func (bb BB) IntersectsSegment(a, b Vector) bool {
	return bb.SegmentQuery(a, b) != INFINITY
}

func (bb BB) ClampVect(v *Vector) Vector {
	return Vector{Clamp(v.X, bb.L, bb.R), Clamp(v.Y, bb.B, bb.T)}
}

func (bb BB) WrapVect(v Vector) Vector {
	dx := math.Abs(bb.R - bb.L)
	modx := math.Mod(v.X-bb.L, dx)
	var x float64
	if modx > 0 {
		x = modx
	} else {
		x = modx + dx
	}

	dy := math.Abs(bb.T - bb.B)
	mody := math.Mod(v.Y-bb.B, dy)
	var y float64
	if mody > 0 {
		y = mody
	} else {
		y = mody + dy
	}

	return Vector{x + bb.L, y + bb.B}
}

func (bb BB) Offset(v Vector) BB {
	return BB{
		bb.L + v.X,
		bb.B + v.Y,
		bb.R + v.X,
		bb.T + v.Y,
	}
}

func (a BB) Proximity(b BB) float64 {
	return math.Abs(a.L+a.R-b.L-b.R) + math.Abs(a.B+a.T-b.B-b.T)
}
