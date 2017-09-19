package cp

import (
	"fmt"
	"math"
	"os"
)

const (
	MAX_GJK_ITERATIONS  = 30
	MAX_EPA_ITERATIONS  = 30
	WARN_GJK_ITERATIONS = 20
	WARN_EPA_ITERATIONS = 20
)

type SupportPoint struct {
	p Vector
	// Save an index of the point so it can be cheaply looked up as a starting point for the next frame.
	index uint32
}

func NewSupportPoint(p Vector, index uint32) *SupportPoint {
	return &SupportPoint{p, uint32(index)}
}

type SupportPointFunc func(shape *Shape, n Vector) *SupportPoint

func PolySupportPoint(shape *Shape, n Vector) *SupportPoint {
	poly := shape.Class.(*PolyShape)
	planes := poly.planes
	i := PolySupportPointIndex(poly.count, planes, n)
	return NewSupportPoint(planes[i].v0, uint32(i))
}

func SegmentSupportPoint(shape *Shape, n Vector) *SupportPoint {
	seg := shape.Class.(*Segment)
	if seg.ta.Dot(n) > seg.tb.Dot(n) {
		return NewSupportPoint(seg.ta, 0)
	} else {
		return NewSupportPoint(seg.tb, 1)
	}
}

func CircleSupportPoint(shape *Shape, n Vector) *SupportPoint {
	return NewSupportPoint(shape.Class.(*Circle).tc, 0)
}

func PolySupportPointIndex(count int, planes []SplittingPlane, n Vector) int {
	max := -INFINITY
	var index int
	for i := 0; i < count; i++ {
		v := planes[i].v0
		d := v.Dot(n)
		if d > max {
			max = d
			index = i
		}
	}

	return index
}

type SupportContext struct {
	shape1, shape2 *Shape
	func1, func2   SupportPointFunc
}

// Calculate the maximal point on the minkowski difference of two shapes along a particular axis.
func (ctx *SupportContext) Support(n Vector) MinkowskiPoint {
	a := ctx.func1(ctx.shape1, n.Neg())
	b := ctx.func2(ctx.shape2, n)
	return NewMinkowskiPoint(a, b)
}

type ClosestPoints struct {
	// Surface points in absolute coordinates.
	a, b Vector
	// Minimum separating axis of the two shapes.
	n Vector
	// Signed distance between the points.
	d float64
	// Concatenation of the id's of the minkoski points.
	collisionId uint32
}

type CollisionFunc func(a, b *Shape, info *CollisionInfo)

func CircleToCircle(a, b *Shape, info *CollisionInfo) {
	c1 := a.Class.(*Circle)
	c2 := b.Class.(*Circle)

	mindist := c1.r + c2.r
	delta := c2.tc.Sub(c1.tc)
	distsq := delta.LengthSq()

	if distsq < mindist*mindist {
		dist := math.Sqrt(distsq)
		if dist != 0 {
			info.n = delta.Mult(1.0 / dist)
		} else {
			info.n = Vector{1, 0}
		}
		info.PushContact(c1.tc.Add(info.n.Mult(c1.r)), c2.tc.Add(info.n.Mult(-c2.r)), 0)
	}
}

func CollisionError(a, b *Shape, info *CollisionInfo) {
	panic("Shape types are not sorted")
}

func CircleToSegment(a, b *Shape, info *CollisionInfo) {
	circle := a.Class.(*Circle)
	segment := b.Class.(*Segment)

	seg_a := segment.ta
	seg_b := segment.tb
	center := circle.tc

	seg_delta := seg_b.Sub(seg_a)
	closest_t := Clamp01(seg_delta.Dot(center.Sub(seg_a)) / seg_delta.LengthSq())
	closest := seg_a.Add(seg_delta.Mult(closest_t))

	mindist := circle.r + segment.r
	delta := closest.Sub(center)
	distsq := delta.LengthSq()
	if distsq < mindist*mindist {
		dist := math.Sqrt(distsq)
		if dist != 0 {
			info.n = delta.Mult(1 / dist)
		} else {
			info.n = segment.tn
		}
		n := info.n

		rot := segment.Shape.body.Rotation()
		if (closest_t != 0.0 || n.Dot(segment.a_tangent.Rotate(rot)) >= 0.0) &&
			(closest_t != 1.0 || n.Dot(segment.b_tangent.Rotate(rot)) >= 0.0) {
			info.PushContact(center.Add(n.Mult(circle.r)), closest.Add(n.Mult(-segment.r)), 0)
		}
	}
}

func SegmentToSegment(a, b *Shape, info *CollisionInfo) {
	seg1 := a.Class.(*Segment)
	seg2 := b.Class.(*Segment)

	context := SupportContext{a, b, SegmentSupportPoint, SegmentSupportPoint}
	points := GJK(&context, &info.collisionId)

	n := points.n
	rot1 := seg1.body.Rotation()
	rot2 := seg2.body.Rotation()

	if points.d > (seg1.r + seg2.r) {
		return
	}

	if (!points.a.Equal(seg1.ta) || n.Dot(seg1.a_tangent.Rotate(rot1)) <= 0) &&
		(!points.a.Equal(seg1.tb) || n.Dot(seg1.b_tangent.Rotate(rot1)) <= 0) &&
		(!points.b.Equal(seg2.ta) || n.Dot(seg2.a_tangent.Rotate(rot2)) >= 0) &&
		(!points.b.Equal(seg2.tb) || n.Dot(seg2.b_tangent.Rotate(rot2)) >= 0) {
		ContactPoints(SupportEdgeForSegment(seg1, n), SupportEdgeForSegment(seg2, n.Neg()), points, info)
	}
}

func CircleToPoly(a, b *Shape, info *CollisionInfo) {
	context := &SupportContext{a, b, CircleSupportPoint, PolySupportPoint}
	points := GJK(context, &info.collisionId)

	circle := a.Class.(*Circle)
	poly := b.Class.(*PolyShape)

	if points.d <= circle.r+poly.r {
		info.n = points.n
		info.PushContact(points.a.Add(info.n.Mult(circle.r)), points.b.Add(info.n.Mult(poly.r)), 0)
	}
}

func SegmentToPoly(seg, poly *Shape, info *CollisionInfo) {
	context := &SupportContext{seg, poly, SegmentSupportPoint, PolySupportPoint}
	points := GJK(context, &info.collisionId)

	n := points.n
	rot := seg.body.Rotation()

	segment := seg.Class.(*Segment)
	polyshape := poly.Class.(*PolyShape)

	// If the closest points are nearer than the sum of the radii...
	if points.d-segment.r-polyshape.r <= 0 && (
	// Reject endcap collisions if tangents are provided.
	(!points.a.Equal(segment.ta) || n.Dot(segment.a_tangent.Rotate(rot)) <= 0) &&
		(!points.a.Equal(segment.tb) || n.Dot(segment.b_tangent.Rotate(rot)) <= 0)) {
		ContactPoints(SupportEdgeForSegment(segment, n), SupportEdgeForPoly(polyshape, n.Neg()), points, info)
	}
}

func PolyToPoly(a, b *Shape, info *CollisionInfo) {
	context := &SupportContext{a, b, PolySupportPoint, PolySupportPoint}
	points := GJK(context, &info.collisionId)

	// TODO: add debug drawing logic like chipmunk does

	poly1 := a.Class.(*PolyShape)
	poly2 := b.Class.(*PolyShape)
	if points.d-poly1.r-poly2.r <= 0 {
		ContactPoints(SupportEdgeForPoly(poly1, points.n), SupportEdgeForPoly(poly2, points.n.Neg()), points, info)
	}
}

// A point on the surface of two shape's minkowski difference.
type MinkowskiPoint struct {
	// Cache the two original support points.
	a, b Vector
	// b - a
	ab Vector
	// Concatenate the two support point indexes.
	collisionId uint32
}

func NewMinkowskiPoint(a, b *SupportPoint) MinkowskiPoint {
	return MinkowskiPoint{a.p, b.p, b.p.Sub(a.p), (a.index&0xFF)<<8 | (b.index & 0xFF)}
}

// Calculate the closest points on two shapes given the closest edge on their minkowski difference to (0, 0)
func (v0 MinkowskiPoint) ClosestPoints(v1 MinkowskiPoint) ClosestPoints {
	// Find the closest p(t) on the minkowski difference to (0, 0)
	t := v0.ab.ClosestT(v1.ab)
	p := v0.ab.LerpT(v1.ab, t)

	// Interpolate the original support points using the same 't' value as above.
	// This gives you the closest surface points in absolute coordinates. NEAT!
	pa := v0.a.LerpT(v1.a, t)
	pb := v0.b.LerpT(v1.b, t)
	id := (v0.collisionId&0xFFFF)<<16 | (v1.collisionId & 0xFFFF)

	// First try calculating the MSA from the minkowski difference edge.
	// This gives us a nice, accurate MSA when the surfaces are close together.
	delta := v1.ab.Sub(v0.ab)
	n := delta.ReversePerp().Normalize()
	d := n.Dot(p)

	if d <= 0 || (-1 < t && t < 1) {
		// If the shapes are overlapping, or we have a regular vertex/edge collision, we are done.
		return ClosestPoints{pa, pb, n, d, id}
	}

	// Vertex/vertex collisions need special treatment since the MSA won't be shared with an axis of the minkowski difference.
	d2 := p.Length()
	n2 := p.Mult(1 / (d2 + math.SmallestNonzeroFloat64))

	return ClosestPoints{pa, pb, n2, d2, id}
}

type EdgePoint struct {
	p Vector
	// Keep a hash value for Chipmunk's collision hashing mechanism.
	hash HashValue
}

type Edge struct {
	a, b *EdgePoint
	r    float64
	n    Vector
}

func SupportEdgeForSegment(seg *Segment, n Vector) *Edge {
	hashid := seg.Shape.hashid
	if seg.tn.Dot(n) > 0 {
		return &Edge{
			a: &EdgePoint{seg.ta, HashPair(hashid, 0)},
			b: &EdgePoint{seg.tb, HashPair(hashid, 1)},
			r: seg.r,
			n: seg.tn,
		}
	}

	return &Edge{
		a: &EdgePoint{seg.tb, HashPair(hashid, 1)},
		b: &EdgePoint{seg.ta, HashPair(hashid, 0)},
		r: seg.r,
		n: seg.tn.Neg(),
	}
}

func SupportEdgeForPoly(poly *PolyShape, n Vector) *Edge {
	count := poly.count
	i1 := PolySupportPointIndex(poly.count, poly.planes, n)

	i0 := (i1 - 1 + count) % count
	i2 := (i1 + 1) % count

	planes := poly.planes
	hashId := poly.hashid

	if n.Dot(planes[i1].n) > n.Dot(planes[i2].n) {
		return &Edge{
			&EdgePoint{planes[i0].v0, HashPair(hashId, HashValue(i0))},
			&EdgePoint{planes[i1].v0, HashPair(hashId, HashValue(i1))},
			poly.r,
			planes[i1].n,
		}
	}

	return &Edge{
		&EdgePoint{planes[i1].v0, HashPair(hashId, HashValue(i1))},
		&EdgePoint{planes[i2].v0, HashPair(hashId, HashValue(i2))},
		poly.r,
		planes[i2].n,
	}
}

// Given two support edges, find contact point pairs on their surfaces.
func ContactPoints(e1, e2 *Edge, points ClosestPoints, info *CollisionInfo) {
	mindist := e1.r + e2.r

	if points.d > mindist {
		return
	}

	n := points.n
	info.n = points.n

	d_e1_a := e1.a.p.Cross(n)
	d_e1_b := e1.b.p.Cross(n)
	d_e2_a := e2.a.p.Cross(n)
	d_e2_b := e2.b.p.Cross(n)

	// TODO + min isn't a complete fix
	e1_denom := 1 / (d_e1_b - d_e1_a + math.SmallestNonzeroFloat64)
	e2_denom := 1 / (d_e2_b - d_e2_a + math.SmallestNonzeroFloat64)

	// Project the endpoints of the two edges onto the opposing edge, clamping them as necessary.
	// Compare the projected points to the collision normal to see if the shapes overlap there.
	{
		p1 := n.Mult(e1.r).Add(e1.a.p.Lerp(e1.b.p, Clamp01((d_e2_b-d_e1_a)*e1_denom)))
		p2 := n.Mult(-e2.r).Add(e2.a.p.Lerp(e2.b.p, Clamp01((d_e1_a-d_e2_a)*e2_denom)))
		dist := p2.Sub(p1).Dot(n)
		if dist <= 0 {
			hash_1a2b := HashPair(e1.a.hash, e2.b.hash)
			info.PushContact(p1, p2, hash_1a2b)
		}
	}
	{
		p1 := n.Mult(e1.r).Add(e1.a.p.Lerp(e1.b.p, Clamp01((d_e2_a-d_e1_a)*e1_denom)))
		p2 := n.Mult(-e2.r).Add(e2.a.p.Lerp(e2.b.p, Clamp01((d_e1_b-d_e2_a)*e2_denom)))
		dist := p2.Sub(p1).Dot(n)
		if dist <= 0 {
			hash_1b2a := HashPair(e1.b.hash, e2.a.hash)
			info.PushContact(p1, p2, hash_1b2a)
		}
	}
}

const HASH_COEF = 3344921057

func HashPair(a, b HashValue) HashValue {
	return a*HASH_COEF ^ b*HASH_COEF
}

// Find the closest points between two shapes using the GJK algorithm.
func GJK(ctx *SupportContext, collisionId *uint32) ClosestPoints {
	var v0, v1 MinkowskiPoint

	if *collisionId != 0 {
		// Use the minkowski points from the last frame as a starting point using the cached indexes.
		v0 = NewMinkowskiPoint(ctx.shape1.Point((*collisionId>>24)&0xFF), ctx.shape2.Point((*collisionId>>16)&0xFF))
		v1 = NewMinkowskiPoint(ctx.shape1.Point((*collisionId>>8)&0xFF), ctx.shape2.Point((*collisionId)&0xFF))
	} else {
		// No cached indexes, use the shapes' bounding box centers as a guess for a starting axis.
		axis := ctx.shape1.bb.Center().Sub(ctx.shape2.bb.Center()).Perp()
		v0 = ctx.Support(axis)
		v1 = ctx.Support(axis.Neg())
	}

	points := GJKRecurse(ctx, v0, v1, 1)
	*collisionId = points.collisionId
	return points
}

// Recursive implementation of the GJK loop.
func GJKRecurse(ctx *SupportContext, v0, v1 MinkowskiPoint, iteration int) ClosestPoints {
	if iteration > MAX_GJK_ITERATIONS {
		return v0.ClosestPoints(v1)
	}

	if v1.ab.PointGreater(v0.ab, Vector{}) {
		// Origin is behind axis. Flip and try again.
		return GJKRecurse(ctx, v1, v0, iteration)
	}
	t := v0.ab.ClosestT(v1.ab)
	var n Vector
	if -1.0 < t && t < 1.0 {
		n = v1.ab.Sub(v0.ab).Perp()
	} else {
		n = v0.ab.LerpT(v1.ab, t).Neg()
	}
	p := ctx.Support(n)

	// Draw debug

	if p.ab.PointGreater(v0.ab, Vector{}) && v1.ab.PointGreater(p.ab, Vector{}) {
		return EPA(ctx, v0, p, v1)
	}

	if v0.ab.CheckAxis(v1.ab, p.ab, n) {
		return v0.ClosestPoints(v1)
	}

	if v0.ab.ClosestDist(p.ab) < p.ab.ClosestDist(v1.ab) {
		return GJKRecurse(ctx, v0, p, iteration+1)
	}

	return GJKRecurse(ctx, p, v1, iteration+1)
}

// Find the closest points on the surface of two overlapping shapes using the EPA algorithm.
// EPA is called from GJK when two shapes overlap.
// This is a moderately expensive step! Avoid it by adding radii to your shapes so their inner polygons won't overlap.
func EPA(ctx *SupportContext, v0 MinkowskiPoint, v1 MinkowskiPoint, v2 MinkowskiPoint) ClosestPoints {
	// TODO: allocate a NxM array here and do an in place convex hull reduction in EPARecurse?
	hull := []MinkowskiPoint{v0, v1, v2}
	return EPARecurse(ctx, 3, hull, 1)
}

// Recursive implementation of the EPA loop.
// Each recursion adds a point to the convex hull until it's known that we have the closest point on the surface.
func EPARecurse(ctx *SupportContext, count int, hull []MinkowskiPoint, iteration int) ClosestPoints {
	mini := 0
	minDist := INFINITY

	// TODO: precalculate this when building the hull and save a step.
	// Find the closest segment hull[i] and hull[i + 1] to (0, 0)
	i := count - 1
	j := 0
	for j < count {
		d := hull[i].ab.ClosestDist(hull[j].ab)
		if d < minDist {
			minDist = d
			mini = i
		}
		i = j
		j++
	}

	v0 := hull[mini]
	v1 := hull[(mini+1)%count]

	p := ctx.Support(v1.ab.Sub(v0.ab).Perp())

	duplicate := p.collisionId == v0.collisionId || p.collisionId == v1.collisionId

	if !duplicate && v0.ab.PointGreater(v1.ab, p.ab) && iteration < MAX_EPA_ITERATIONS {
		// Rebuild the convex hull by inserting p.
		hull2 := make([]MinkowskiPoint, count+1)
		count2 := 1
		hull2[0] = p

		for i := 0; i < count; i++ {
			index := (mini + 1 + i) % count

			h0 := hull2[count2-1].ab
			h1 := hull[index].ab
			var h2 Vector
			if i+1 < count {
				h2 = hull[(index+1)%count].ab
			} else {
				h2 = p.ab
			}

			if h0.PointGreater(h2, h1) {
				hull2[count2] = hull[index]
				count2++
			}
		}

		return EPARecurse(ctx, count2, hull2, iteration+1)
	}

	if iteration > WARN_EPA_ITERATIONS {
		fmt.Fprintln(os.Stderr, "Warning: High EPA iterations:", iteration)
	}

	// Could not find a new point to insert, so we have found the closest edge of the minkowski difference.
	return v0.ClosestPoints(v1)
}

var BuiltinCollisionFuncs [9]CollisionFunc = [9]CollisionFunc{
	CircleToCircle,
	CollisionError,
	CollisionError,
	CircleToSegment,
	SegmentToSegment,
	CollisionError,
	CircleToPoly,
	SegmentToPoly,
	PolyToPoly,
}

func Collide(a, b *Shape, collisionId uint32, contacts []Contact) *CollisionInfo {
	info := &CollisionInfo{a, b, collisionId, Vector{}, 0, contacts}

	// Make sure the shape types are in order.
	if a.Order() > b.Order() {
		info.a = b
		info.b = a
	}

	BuiltinCollisionFuncs[info.a.Order()+info.b.Order()*SHAPE_TYPE_NUM](info.a, info.b, info)

	return info
}
