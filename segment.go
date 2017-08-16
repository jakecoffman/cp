package physics

type Segment struct {
	*Shape

	A, B, n    *Vector
	ta, tb, tn *Vector
	r          float64

	a_tangent, b_tangent *Vector
}

func (seg *Segment) CacheData(transform *Transform) *BB {
	seg.ta = transform.Point(seg.A)
	seg.tb = transform.Point(seg.B)
	seg.tn = transform.Vect(seg.n)

	var l, r, b, t float64

	if seg.ta.X < seg.tb.X {
		l = seg.ta.X
		r = seg.tb.X
	} else {
		l = seg.tb.X
		r = seg.ta.X
	}

	if seg.ta.Y < seg.tb.Y {
		b = seg.ta.Y
		t = seg.tb.Y
	} else {
		b = seg.tb.Y
		t = seg.ta.Y
	}

	rad := seg.r
	return &BB{l - rad, b - rad, r + rad, t + rad}
}

func (seg *Segment) Destroy() {
	panic("implement me")
}

func (seg *Segment) PointQuery(p Vector, info *PointQueryInfo) {
	closest := p.ClosestPointOnSegment(seg.ta, seg.tb)

	delta := p.Sub(closest)
	d := delta.Length()
	r := seg.r
	g := delta.Mult(1/d)

	info.shape = seg.Shape
	if d != 0 {
		info.point = closest.Add(g.Mult(r))
	} else {
		info.point = closest
	}
	info.distance = d - r

	// Use the segment's normal if the distance is very small.
	if d > MAGIC_EPSILON {
		info.gradient = g
	} else {
		info.gradient = seg.n
	}
}

func (seg *Segment) SegmentQuery(a, b Vector, radius float64, info *SegmentQueryInfo) {
	panic("implement me")
}

func NewSegment(body *Body, a, b *Vector, r float64) *Shape {
	segment := &Segment{
		A: a,
		B: b,
		n: b.Sub(a).Normalize().Perp(),

		r:         r,
		a_tangent: VectorZero(),
		b_tangent: VectorZero(),
	}
	segment.Shape = NewShape(segment, body, NewSegmentMassInfo(0, a, b, r))
	return segment.Shape
}

func NewSegmentMassInfo(mass float64, a, b *Vector, r float64) *ShapeMassInfo {
	return &ShapeMassInfo{
		m:    mass,
		i:    MomentForBox(1, a.Distance(b)+2*r, 2*r),
		cog:  a.Lerp(b, 0.5),
		area: AreaForSegment(a, b, r),
	}
}
