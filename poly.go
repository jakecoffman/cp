package physics

import "math"

type PolyShape struct {
	*Shape

	r float64

	count uint

	// The untransformed planes are appended at the end of the transformed planes.
	planes []SplittingPlane
}

func (poly *PolyShape) CacheData(transform Transform) BB {
	count := poly.count
	dst := poly.planes
	src := poly.planes[count:]

	l := INFINITY
	r := -INFINITY
	b := INFINITY
	t := -INFINITY

	var i uint
	for i = 0; i < count; i++ {
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
	poly.Shape.bb = BB{l - radius, b - radius, r + radius, t + radius}
	return poly.Shape.bb
}

func (poly *PolyShape) Destroy() {
	panic("implement me")
}

func (poly *PolyShape) PointQuery(p Vector, info *PointQueryInfo) {
	count := poly.count
	planes := poly.planes
	r := poly.r

	v0 := planes[count-1].v0
	minDist := INFINITY
	closestPoint := VectorZero()
	closestNormal := VectorZero()
	outside := false

	var i uint
	for i = 0; i<count; i++ {
		v1 := planes[i].v0
		if !outside {
			outside = planes[i].n.Dot(p.Sub(v1)) > 0
		}

		closest := p.ClosestPointOnSegment(v0, v1)

		dist := p.Distance(closest)
		if dist < minDist {
			minDist = dist
			closestPoint = closest
			closestNormal = planes[i].n
		}

		v0 = v1
	}

	var dist float64
	if outside {
		dist = minDist
	} else {
		dist = -minDist
	}
	g := p.Sub(closestPoint).Mult(1.0/dist)

	info.Shape = poly.Shape
	info.Point = closestPoint.Add(g.Mult(r))
	info.Distance = dist-r

	if minDist > MAGIC_EPSILON {
		info.Gradient = g
 	} else {
		info.Gradient = closestNormal
	}
}

func (poly *PolyShape) SegmentQuery(a, b Vector, radius float64, info *SegmentQueryInfo) {
	panic("implement me")
}

func NewPolyShape(body *Body, verts []Vector, transform Transform, radius float64) *Shape {
	hullVerts := []Vector{}
	for _, vert := range verts {
		hullVerts = append(hullVerts, transform.Point(vert))
	}

	hullCount := ConvexHull(len(hullVerts), hullVerts, nil, 0)
	return NewPolyShapeRaw(body, uint(hullCount), hullVerts, radius)
}

func NewPolyShapeRaw(body *Body, count uint, verts []Vector, radius float64) *Shape {
	poly := &PolyShape{
		r:      radius,
		count:  count,
		planes: []SplittingPlane{},
	}
	poly.Shape = NewShape(poly, body, PolyShapeMassInfo(0, verts, radius))
	poly.SetVerts(verts)
	return poly.Shape
}

func NewBox(body *Body, w, h, r float64) *Shape {
	hw := w / 2.0
	hh := h / 2.0
	bb := &BB{-hw, -hh, hw, hh}
	verts := []Vector{
		{bb.R, bb.B},
		{bb.R, bb.T},
		{bb.L, bb.T},
		{bb.L, bb.B},
	}
	return NewPolyShapeRaw(body, 4, verts, r)
}

func (p *PolyShape) SetVerts(verts []Vector) {
	count := len(verts)
	p.count = uint(count)
	p.planes = make([]SplittingPlane, count*2)

	for i := range p.planes {
		p.planes[i] = SplittingPlane{}
	}

	for i := 0; i < count; i++ {
		a := verts[(i-1+count)%count]
		b := verts[i]
		n := b.Sub(a).ReversePerp().Normalize()

		p.planes[i+count].v0 = b
		p.planes[i+count].n = n
	}
}

func PolyShapeMassInfo(mass float64, verts []Vector, r float64) *ShapeMassInfo {
	centroid := CentroidForPoly(verts)
	return &ShapeMassInfo{
		m:    mass,
		i:    MomentForPoly(1, verts, centroid.Neg(), r),
		cog:  centroid,
		area: AreaForPoly(verts, r),
	}
}

// QuickHull seemed like a neat algorithm, and efficient-ish for large input sets.
// My implementation performs an in place reduction using the result array as scratch space.
func ConvexHull(count int, verts []Vector, first *int, tol float64) int {
	start, end := LoopIndexes(verts, count)
	if start == end {
		if first != nil {
			*first = 0
		}
		return 1
	}

	verts[0], verts[1] = verts[1], verts[0]
	if end == 0 {
		verts[1], verts[start] = verts[start], verts[1]
	} else {
		verts[1], verts[end] = verts[end], verts[1]
	}

	a := verts[0]
	b := verts[1]

	if first != nil {
		*first = start
	}

	return QHullReduce(tol, verts[2:], count-2, a, b, a, verts[1:]) + 1
}

func LoopIndexes(verts []Vector, count int) (int, int) {
	start := 0
	end := 0

	min := verts[0]
	max := min

	for i := 0; i < count; i++ {
		v := verts[i]

		if v.X < min.X || (v.X == min.X && v.Y < min.Y) {
			min = v
			start = i
		} else if v.X > max.X || (v.X == max.X && v.Y > max.Y) {
			max = v
			end = i
		}
	}

	return start, end
}

func QHullReduce(tol float64, verts []Vector, count int, a, pivot, b Vector, result []Vector) int {
	if count < 0 {
		return 0
	}

	if count == 0 {
		result[0] = pivot
		return 1
	}

	leftCount := QHullPartition(verts, count, a, pivot, tol)
	index := QHullReduce(tol, verts[1:], leftCount-1, a, verts[0], pivot, result)

	result[index] = pivot
	index++

	rightCount := QHullPartition(verts[leftCount:], count-leftCount, pivot, b, tol)

	// Go doesn't let you just walk off the end of an array, so added a short circuit here
	if rightCount-1 < 0 {
		return index
	}

	return index + QHullReduce(tol, verts[leftCount+1:], rightCount-1, pivot, verts[leftCount], b, result[index:])
}

func QHullPartition(verts []Vector, count int, a, b Vector, tol float64) int {
	if count == 0 {
		return 0
	}

	max := 0.0
	pivot := 0

	delta := b.Sub(a)
	valueTol := tol * delta.Length()

	head := 0
	for tail := count - 1; head <= tail; {
		value := verts[head].Sub(a).Cross(delta)
		if value > valueTol {
			if value > max {
				max = value
				pivot = head
			}

			head++
		} else {
			verts[head], verts[tail] = verts[tail], verts[head]
			tail--
		}
	}

	// move the new pivot to the front if it's not already there.
	if pivot != 0 {
		verts[0], verts[pivot] = verts[pivot], verts[0]
	}
	return head
}
