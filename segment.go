package physics

type Segment struct {
	*Shape

	a, b, n    *Vector
	ta, tb, tn *Vector
	r          float64

	a_tangent, b_tangent *Vector
}

func NewSegment(body *Body, a, b *Vector, r float64) *Segment {
	return &Segment{
		Shape: NewShape(body, NewSegmentMassInfo(0, a, b, r)),
		a:     a,
		b:     b,
		n:     b.Sub(a).Normalize().Perp(),

		r:         r,
		a_tangent: VectorZero(),
		b_tangent: VectorZero(),
	}
}

func NewSegmentMassInfo(mass float64, a, b *Vector, r float64) *ShapeMassInfo {
	return &ShapeMassInfo{
		m:    mass,
		i:    MomentForBox(1, a.Distance(b)+2*r, 2*r),
		cog:  a.Lerp(b, 0.5),
		area: AreaForSegment(a, b, r),
	}
}
