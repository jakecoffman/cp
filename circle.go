package cp

import "math"

type Circle struct {
	*Shape
	c, tc Vector
	r     float64
}

func NewCircle(body *Body, radius float64, offset Vector) *Shape {
	circle := &Circle{
		c: offset,
		r: radius,
	}
	circle.Shape = NewShape(circle, body, CircleShapeMassInfo(0, radius, offset))
	return circle.Shape
}

func CircleShapeMassInfo(mass, radius float64, center Vector) *ShapeMassInfo {
	return &ShapeMassInfo{
		m:    mass,
		i:    MomentForCircle(1, 0, radius, Vector{}),
		cog:  center,
		area: AreaForCircle(0, radius),
	}
}

func (circle *Circle) CacheData(transform Transform) BB {
	circle.tc = transform.Point(circle.c)
	return NewBBForCircle(circle.tc, circle.r)
}

func (*Circle) Destroy() {
	panic("implement me")
}

func (circle *Circle) Radius() float64 {
	return circle.r
}

func (circle *Circle) SetRadius(r float64) {
	circle.r = r

	mass := circle.massInfo.m
	circle.massInfo = CircleShapeMassInfo(mass, circle.r, circle.c)
	if mass > 0 {
		circle.body.AccumulateMassFromShapes()
	}
}

func (circle *Circle) TransformC() Vector {
	return circle.tc
}

func (circle *Circle) PointQuery(p Vector, info *PointQueryInfo) {
	delta := p.Sub(circle.tc)
	d := delta.Length()
	r := circle.r

	info.Shape = circle.Shape
	info.Point = circle.tc.Add(delta.Mult(r / d))
	info.Distance = d - r

	if d > MAGIC_EPSILON {
		info.Gradient = delta.Mult(1 / d)
	} else {
		info.Gradient = Vector{0, 1}
	}
}

func (circle *Circle) SegmentQuery(a, b Vector, radius float64, info *SegmentQueryInfo) {
	CircleSegmentQuery(circle.Shape, circle.tc, circle.r, a, b, radius, info)
}

func CircleSegmentQuery(shape *Shape, center Vector, r1 float64, a, b Vector, r2 float64, info *SegmentQueryInfo) {
	da := a.Sub(center)
	db := b.Sub(center)
	rsum := r1 + r2

	qa := da.Dot(da) - 2*da.Dot(db) + db.Dot(db)
	qb := da.Dot(db) - da.Dot(da)
	det := qb*qb - qa*(da.Dot(da)-rsum*rsum)

	if det >= 0 {
		t := (-qb - math.Sqrt(det))/qa
		if 0 <= t && t <= 1 {
			n := da.Lerp(db, t).Normalize()

			info.Shape = shape
			info.Point = a.Lerp(b, t).Sub(n.Mult(r2))
			info.Normal = n
			info.Alpha = t
		}
	}
}
