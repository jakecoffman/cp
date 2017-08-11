package physics

import "math"

type PolyShape struct {
	*Shape

	r float64

	count uint

	// The untransformed planes are appended at the end of the transformed planes.
	planes []*SplittingPlane
}

func (poly *PolyShape) CacheData(transform *Transform) *BB {
	count := poly.count
	dst := poly.planes
	src := dst[count:]

	l := INFINITY
	r := -INFINITY
	b := INFINITY
	t := -INFINITY

	var i uint
	for i=0; i<count; i++ {
		v := transform.Point(src[i].v0)
		n := transform.Vect(src[i].n)

		dst[i].v0 = v
		dst[i].n = n

		l = math.Min(l, v.X)
		r = math.Max(r, v.X)
		b = math.Min(b, v.Y)
		t = math.Max(t, v.Y)
	}

	radius := poly.r
	poly.Shape.bb = &BB{l-radius, b-radius, r+radius, t+radius}
	return poly.Shape.bb
}

func (poly *PolyShape) Destroy() {
	panic("implement me")
}

func (poly *PolyShape) PointQuery(p Vector, info *PointQueryInfo) {
	panic("implement me")
}

func (poly *PolyShape) SegmentQuery(a, b Vector, radius float64, info *SegmentQueryInfo) {
	panic("implement me")
}

func NewPolyShape() *PolyShape {
	return &PolyShape{}
}

func NewBox(body *Body, w, h, r float64) *Shape {
	hw := w / 2
	hh := h / 2
	bb := &BB{-hw, -hh, hw, hh}
	verts := []*Vector{
		{bb.r, bb.b},
		{bb.r, bb.t},
		{bb.l, bb.t},
		{bb.l, bb.b},
	}
	poly := &PolyShape{
		r: r,
	}
	poly.Shape = NewShape(poly, body, PolyShapeMassInfo(0, verts, r))
	poly.SetVerts(verts)
	return poly.Shape
}

func (p *PolyShape) SetVerts(verts []*Vector) {
	count := len(verts)
	p.planes = make([]*SplittingPlane, 2*count)

	for i := range verts {
		a := verts[(i-1+count)%count]
		b := verts[i]
		n := b.Sub(a).Perp().Normalize()

		p.planes[i+count] = &SplittingPlane{v0: b, n: n}
	}
}

func PolyShapeMassInfo(mass float64, verts []*Vector, r float64) *ShapeMassInfo {
	centroid := CentroidForPoly(verts)
	return &ShapeMassInfo{
		m:    mass,
		i:    MomentForPoly(1, verts, centroid.Neg(), r),
		cog:  centroid,
		area: AreaForPoly(verts, r),
	}
}
