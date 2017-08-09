package physics

import "math"

type BB struct {
	l, b, r, t float64
}

func NewBBForExtents(c *Vector, hw, hh float64) *BB {
	return &BB{
		l: c.X - hw,
		b: c.Y - hh,
		r: c.X + hw,
		t: c.Y + hh,
	}
}

func NewBBForCircle(p *Vector, r float64) *BB {
	return NewBBForExtents(p, r, r)
}

func (a *BB) Intersects(b *BB) bool {
	return a.l <= b.r && b.l <= a.r && a.b <= b.t && b.b <= a.t
}

func (bb *BB) Contains(other *BB) bool {
	return bb.l <= other.l && bb.r >= other.r && bb.b <= other.b && bb.t >= other.t
}

func (bb *BB) ContainsVect(v *Vector) bool {
	return bb.l <= v.X && bb.r >= v.X && bb.b <= v.Y && bb.t >= v.Y
}

func (a *BB) Merge(b *BB) *BB {
	return &BB{
		math.Min(a.l, b.l),
		math.Min(a.b, b.b),
		math.Max(a.r, b.r),
		math.Max(a.t, b.t),
	}
}

func (bb *BB) Expand(v *Vector) *BB {
	return &BB{
		math.Min(bb.l, v.X),
		math.Min(bb.b, v.Y),
		math.Max(bb.r, v.X),
		math.Max(bb.t, v.Y),
	}
}

func (bb *BB) Center() *Vector {
	return (&Vector{bb.l, bb.b}).Lerp(&Vector{bb.r, bb.t}, 0.5)
}

func (bb *BB) Area() float64 {
	return (bb.r - bb.l) * (bb.t - bb.b)
}

func (a *BB) MergedArea(b *BB) float64 {
	return (math.Max(a.r, b.r) - math.Min(a.l, b.l)) * (math.Max(a.t, b.t) - math.Min(a.b, b.b))
}

func (bb *BB) SegmentQuery(a, b *Vector) float64 {
	delta := b.Sub(a)
	tmin := -INFINITY
	tmax := INFINITY

	if delta.X == 0 {
		if a.X < bb.l || bb.r < a.X {
			return INFINITY
		}
	} else {
		t1 := (bb.l - a.X) / delta.X
		t2 := (bb.r - a.X) / delta.X
		tmin = math.Max(tmin, math.Min(t1, t2))
		tmax = math.Min(tmax, math.Max(t1, t2))
	}

	if delta.Y == 0 {
		if a.Y < bb.b || bb.t < a.Y {
			return INFINITY
		}
	} else {
		t1 := (bb.b - a.Y) / delta.Y
		t2 := (bb.t - a.Y) / delta.Y
		tmin = math.Max(tmin, math.Min(t1, t2))
		tmax = math.Min(tmax, math.Max(t1, t2))
	}

	if tmin <= tmax && 0 <= tmax && tmin <= 1.0 {
		return math.Max(tmin, 0.0)
	} else {
		return INFINITY
	}
}

func (bb *BB) IntersectsSegment(a, b *Vector) bool {
	return bb.SegmentQuery(a, b) != INFINITY
}

func (bb *BB) ClampVect(v *Vector) *Vector {
	return &Vector{Clamp(v.X, bb.l, bb.r), Clamp(v.Y, bb.b, bb.t)}
}

func (bb *BB) WrapVect(v *Vector) *Vector {
	dx := math.Abs(bb.r - bb.l)
	modx := math.Mod(v.X-bb.l, dx)
	var x float64
	if modx > 0 {
		x = modx
	} else {
		x = modx + dx
	}

	dy := math.Abs(bb.t - bb.b)
	mody := math.Mod(v.Y-bb.b, dy)
	var y float64
	if mody > 0 {
		y = mody
	} else {
		y = mody + dy
	}

	return &Vector{x + bb.l, y + bb.b}
}

func (bb *BB) Offset(v *Vector) *BB {
	return &BB{
		bb.l + v.X,
		bb.b + v.Y,
		bb.r + v.X,
		bb.t + v.Y,
	}
}

func (a *BB) Proximity(b *BB) float64 {
	return math.Abs(a.l+a.r-b.l-b.r) + math.Abs(a.b+a.t-b.b-b.t)
}
