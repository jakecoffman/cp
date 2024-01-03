package cp

import (
	"fmt"
	"math"
)

// BB is Chipmunk's axis-aligned 2D bounding box type. (left, bottom, right, top)
type BB struct {
	L, B, R, T float64
}

// NewBB is convenience constructor for BB structs.
func NewBB(l, b, r, t float64) BB {
	return BB{
		L: l,
		B: b,
		R: r,
		T: t,
	}
}

func (bb BB) String() string {
	return fmt.Sprintf("%v %v %v %v", bb.L, bb.T, bb.R, bb.B)
}

// NewBBForExtents constructs a BB centered on a point with the given extents (half sizes).
func NewBBForExtents(c Vector, hw, hh float64) BB {
	return BB{
		L: c.X - hw,
		B: c.Y - hh,
		R: c.X + hw,
		T: c.Y + hh,
	}
}

// NewBBForCircle constructs a BB for a circle with the given position and radius.
func NewBBForCircle(p Vector, r float64) BB {
	return NewBBForExtents(p, r, r)
}

// Intersects returns true if a and b intersect.
func (a BB) Intersects(b BB) bool {
	return a.L <= b.R && b.L <= a.R && a.B <= b.T && b.B <= a.T
}

// Contains returns true if other lies completely within bb.
func (bb BB) Contains(other BB) bool {
	return bb.L <= other.L && bb.R >= other.R && bb.B <= other.B && bb.T >= other.T
}

// ContainsVect returns true if bb contains v.
func (bb BB) ContainsVect(v Vector) bool {
	return bb.L <= v.X && bb.R >= v.X && bb.B <= v.Y && bb.T >= v.Y
}

// Merge returns a bounding box that holds both bounding boxes.
func (a BB) Merge(b BB) BB {
	return BB{
		math.Min(a.L, b.L),
		math.Min(a.B, b.B),
		math.Max(a.R, b.R),
		math.Max(a.T, b.T),
	}
}

// Expand returns a bounding box that holds both bb and v.
func (bb BB) Expand(v Vector) BB {
	return BB{
		math.Min(bb.L, v.X),
		math.Min(bb.B, v.Y),
		math.Max(bb.R, v.X),
		math.Max(bb.T, v.Y),
	}
}

// Center returns the center of a bounding box.
func (bb BB) Center() Vector {
	return Vector{bb.L, bb.B}.Lerp(Vector{bb.R, bb.T}, 0.5)
}

// Area returns the area of the bounding box.
func (bb BB) Area() float64 {
	return (bb.R - bb.L) * (bb.T - bb.B)
}

// MergedArea merges a and b and returns the area of the merged bounding box.
func (a BB) MergedArea(b BB) float64 {
	return (math.Max(a.R, b.R) - math.Min(a.L, b.L)) * (math.Max(a.T, b.T) - math.Min(a.B, b.B))
}

// SegmentQuery returns the fraction along the segment query the BB is hit.
// Returns cp.INFINITY if it doesn't hit.
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

// IntersectsSegment returns true if the bounding box intersects the line segment with ends a and b.
func (bb BB) IntersectsSegment(a, b Vector) bool {
	return bb.SegmentQuery(a, b) != INFINITY
}

// ClampVect clamps a vector to bounding box.
func (bb BB) ClampVect(v *Vector) Vector {
	return Vector{Clamp(v.X, bb.L, bb.R), Clamp(v.Y, bb.B, bb.T)}
}

// WrapVect wraps a vector to bounding box.
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

// Offset returns a bounding box offseted by v.
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
