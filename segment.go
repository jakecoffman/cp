package cp

type Segment struct {
	*Shape

	a, b, n    Vector
	ta, tb, tn Vector
	r          float64

	a_tangent, b_tangent Vector
}

func (seg *Segment) CacheData(transform Transform) BB {
	seg.ta = transform.Point(seg.a)
	seg.tb = transform.Point(seg.b)
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
	return BB{l - rad, b - rad, r + rad, t + rad}
}

func (seg *Segment) Destroy() {
	panic("implement me")
}

func (seg *Segment) SetRadius(r float64) {
	seg.r = r

	mass := seg.massInfo.m
	seg.massInfo = NewSegmentMassInfo(seg.massInfo.m, seg.a, seg.b, seg.r)
	if mass > 0 {
		seg.body.AccumulateMassFromShapes()
	}
}

func (seg *Segment) Radius() float64 {
	return seg.r
}

func (seg *Segment) TransformA() Vector {
	return seg.ta
}

func (seg *Segment) TransformB() Vector {
	return seg.tb
}

func (seg *Segment) SetEndpoints(a, b Vector) {
	seg.a = a
	seg.b = b
	seg.n = b.Sub(a).Normalize().Perp()

	mass := seg.massInfo.m
	seg.massInfo = NewSegmentMassInfo(seg.massInfo.m, seg.a, seg.b, seg.r)
	if mass > 0 {
		seg.body.AccumulateMassFromShapes()
	}
}

func (seg *Segment) Normal() Vector {
	return seg.n
}

func (seg *Segment) A() Vector {
	return seg.a
}

func (seg *Segment) B() Vector {
	return seg.b
}

func (seg *Segment) PointQuery(p Vector, info *PointQueryInfo) {
	closest := p.ClosestPointOnSegment(seg.ta, seg.tb)

	delta := p.Sub(closest)
	d := delta.Length()
	r := seg.r
	g := delta.Mult(1 / d)

	info.Shape = seg.Shape
	if d != 0 {
		info.Point = closest.Add(g.Mult(r))
	} else {
		info.Point = closest
	}
	info.Distance = d - r

	// Use the segment's normal if the distance is very small.
	if d > MAGIC_EPSILON {
		info.Gradient = g
	} else {
		info.Gradient = seg.n
	}
}

func (seg *Segment) SegmentQuery(a, b Vector, r2 float64, info *SegmentQueryInfo) {
	n := seg.tn
	d := seg.ta.Sub(a).Dot(n)
	r := seg.r + r2

	var flippedN Vector
	if d > 0 {
		flippedN = n.Neg()
	} else {
		flippedN = n
	}
	segOffset := flippedN.Mult(r).Sub(a)

	// Make the endpoints relative to 'a' and move them by the thickness of the segment.
	segA := seg.ta.Add(segOffset)
	segB := seg.tb.Add(segOffset)
	delta := b.Sub(a)

	if delta.Cross(segA)*delta.Cross(segB) <= 0 {
		dOffset := d
		if d > 0 {
			dOffset -= r
		} else {
			dOffset += r
		}
		ad := -dOffset
		bd := delta.Dot(n) - dOffset

		if ad*bd < 0 {
			t := ad / (ad - bd)

			info.Shape = seg.Shape
			info.Point = a.Lerp(b, t).Sub(flippedN.Mult(r2))
			info.Normal = flippedN
			info.Alpha = t
		}
	} else if r != 0 {
		info1 := SegmentQueryInfo{nil, b, Vector{}, 1}
		info2 := SegmentQueryInfo{nil, b, Vector{}, 1}
		CircleSegmentQuery(seg.Shape, seg.ta, seg.r, a, b, r2, &info1)
		CircleSegmentQuery(seg.Shape, seg.tb, seg.r, a, b, r2, &info2)

		if info1.Alpha < info2.Alpha {
			*info = info1
		} else {
			*info = info2
		}
	}
}

func NewSegment(body *Body, a, b Vector, r float64) *Shape {
	segment := &Segment{
		a: a,
		b: b,
		n: b.Sub(a).Normalize().ReversePerp(),

		r:         r,
		a_tangent: Vector{},
		b_tangent: Vector{},
	}
	segment.Shape = NewShape(segment, body, NewSegmentMassInfo(0, a, b, r))
	return segment.Shape
}

func NewSegmentMassInfo(mass float64, a, b Vector, r float64) *ShapeMassInfo {
	return &ShapeMassInfo{
		m:    mass,
		i:    MomentForBox(1, a.Distance(b)+2*r, 2*r),
		cog:  a.Lerp(b, 0.5),
		area: AreaForSegment(a, b, r),
	}
}
