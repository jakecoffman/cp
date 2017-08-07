package physics

type PolyShape struct {
	*Shape

	r float64

	count int

	// The untransformed planes are appended at the end of the transformed planes.
	planes []*SplittingPlane
}

func NewPolyShape() *PolyShape {
	return &PolyShape{
	}
}

func NewBox(body *Body, w, h, r float64) *PolyShape {
	hw := w/2
	hh := h/2
	bb := &BB{-hw, -hh, hw, hh}
	verts := []*Vector{
		{bb.r, bb.b},
		{bb.r, bb.t},
		{bb.l, bb.t},
		{bb.l, bb.b},
	}
	poly := &PolyShape {
		Shape: NewShape(body, PolyShapeMassInfo(0, verts, r)),
		r: r,
	}
	poly.SetVerts(verts)
	return poly
}

func (p *PolyShape) SetVerts(verts []*Vector) {
	count := len(verts)
	p.planes = make([]*SplittingPlane, 2*count)

	for i := range verts {
		a := verts[(i - 1 + count)%count]
		b := verts[i]
		n := b.Sub(a).Perp().Normalize()

		p.planes[i + count] = &SplittingPlane{v0: b, n: n}
	}
}

func PolyShapeMassInfo(mass float64, verts []*Vector, r float64) *ShapeMassInfo {
	centroid := CentroidForPoly(verts)
	return &ShapeMassInfo{
		m: mass,
		i: MomentForPoly(1, verts, centroid.Neg(), r),
		cog: centroid,
		area: AreaForPoly(verts, r),
	}
}
