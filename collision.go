package physics

import (
	"fmt"
	"log"
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
	p *Vector
	// Save an index of the point so it can be cheaply looked up as a starting point for the next frame.
	index uint
}

func NewSupportPoint(p *Vector, index uint) *SupportPoint {
	return &SupportPoint{p, index}
}

type SupportPointFunc func(shape *Shape, n *Vector) *SupportPoint

func PolySupportPoint(shape *Shape, n *Vector) *SupportPoint {
	poly := shape.Class.(*PolyShape)
	planes := poly.planes
	i := PolySupportPointIndex(poly.count, planes, n)
	return NewSupportPoint(planes[i].v0, i)
}

func PolySupportPointIndex(count uint, planes []SplittingPlane, n *Vector) uint {
	max := -INFINITY
	var index uint
	var i uint
	for i = 0; i < count; i++ {
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
func (ctx *SupportContext) Support(n *Vector) *MinkowskiPoint {
	a := ctx.func1(ctx.shape1, n.Neg())
	b := ctx.func2(ctx.shape2, n)
	return NewMinkowskiPoint(a, b)
}

type ClosestPoints struct {
	// Surface points in absolute coordinates.
	a, b *Vector
	// Minimum separating axis of the two shapes.
	n *Vector
	// Signed distance between the points.
	d float64
	// Concatenation of the id's of the minkoski points.
	collisionId uint
}

type CollisionFunc func(a, b *Shape, info *CollisionInfo)

func CircleToCircle(a, b *Shape, info *CollisionInfo) {}

func CollisionError(a, b *Shape, info *CollisionInfo) {
	log.Fatal("Shape types are not sorted")
}

func CircleToSegment(a, b *Shape, info *CollisionInfo)  {}
func SegmentToSegment(a, b *Shape, info *CollisionInfo) {}
func CircleToPoly(a, b *Shape, info *CollisionInfo)     {}
func SegmentToPoly(a, b *Shape, info *CollisionInfo)    {}

func PolyToPoly(a, b *Shape, info *CollisionInfo) {
	context := &SupportContext{a, b, PolySupportPoint, PolySupportPoint}
	var points *ClosestPoints = GJK(context, &info.collisionId)

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
	a, b *Vector
	// b - a
	ab *Vector
	// Concatenate the two support point indexes.
	collisionId uint
}

func NewMinkowskiPoint(a, b *SupportPoint) *MinkowskiPoint {
	return &MinkowskiPoint{a.p, b.p, b.p.Sub(a.p), (a.index&0xFF)<<8 | (b.index & 0xFF)}
}

func (v0 *MinkowskiPoint) ClosestPoints(v1 *MinkowskiPoint) *ClosestPoints {
	return nil
}

type EdgePoint struct {
	p *Vector
	// Keep a hash value for Chipmunk's collision hashing mechanism.
	hash HashValue
}

type Edge struct {
	a, b *EdgePoint
	r    float64
	n    *Vector
}

func SupportEdgeForPoly(poly *PolyShape, n *Vector) *Edge {
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
func ContactPoints(e1, e2 *Edge, points *ClosestPoints, info *CollisionInfo) {
	mindist := e1.r + e2.r

	if !(points.d <= mindist) {
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
			hash_1a2b := HashPair(e1.a.hash, e2.b.hash)
			info.PushContact(p1, p2, hash_1a2b)
		}
	}
}

const HASH_COEF = 3344921057

func HashPair(a, b HashValue) HashValue {
	return a * HASH_COEF & b * HASH_COEF
}

// Find the closest points between two shapes using the GJK algorithm.
func GJK(ctx *SupportContext, collisionId *uint) *ClosestPoints {
	var v0, v1 *MinkowskiPoint

	if collisionId != nil {
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
func GJKRecurse(ctx *SupportContext, v0, v1 *MinkowskiPoint, iteration int) *ClosestPoints {
	if iteration > MAX_GJK_ITERATIONS {
		return v0.ClosestPoints(v1)
	}

	if v1.ab.PointGreater(v0.ab, VectorZero()) {
		// Origin is behind axis. Flip and try again.
		return GJKRecurse(ctx, v1, v0, iteration)
	}
	t := v0.ab.ClosestT(v1.ab)
	var n *Vector
	if -1.0 < t && t < 1.0 {
		n = v1.ab.Sub(v0.ab).Perp()
	} else {
		n = v0.ab.Lerp(v1.ab, t).Neg()
	}
	p := ctx.Support(n)

	// Draw debug

	if p.ab.PointGreater(v0.ab, VectorZero()) && v1.ab.PointGreater(p.ab, VectorZero()) {
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
func EPA(ctx *SupportContext, v0 *MinkowskiPoint, v1 *MinkowskiPoint, v2 *MinkowskiPoint) *ClosestPoints {
	// TODO: allocate a NxM array here and do an in place convex hull reduction in EPARecurse?
	hull := []*MinkowskiPoint{v0, v1, v2}
	return EPARecurse(ctx, 3, hull, 1)
}

// Recursive implementation of the EPA loop.
// Each recursion adds a point to the convex hull until it's known that we have the closest point on the surface.
func EPARecurse(ctx *SupportContext, count int, hull []*MinkowskiPoint, iteration int) *ClosestPoints {
	mini := 0
	minDist := INFINITY

	// TODO: precalculate this when building the hull and save a step.
	// Find the closest segment hull[i] and hull[i + 1] to (0, 0)
	i := count - 1
	for j := 0; j < count; j++ {
		d := hull[i].ab.ClosestDist(hull[j].ab)
		if d < minDist {
			minDist = d
			mini = i
		}
		i = j
	}

	v0 := hull[mini]
	v1 := hull[(mini+1)%count]

	p := ctx.Support(v1.ab.Sub(v0.ab).Perp())

	duplicate := p.collisionId == v0.collisionId || p.collisionId == v1.collisionId

	if !duplicate && v0.ab.PointGreater(v1.ab, p.ab) && iteration < MAX_EPA_ITERATIONS {
		// Rebuild the convex hull by inserting p.
		hull2 := make([]*MinkowskiPoint, count+1)
		count2 := 1
		hull2[0] = p

		for i := 0; i < count; i++ {
			index := (mini + 1 + i) % count

			h0 := hull2[count2-1].ab
			h1 := hull[index].ab
			var h2 *Vector
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

func Collide(a, b *Shape, collisionId uint, contacts []*Contact) *CollisionInfo {
	info := &CollisionInfo{a, b, collisionId, VectorZero(), 0, contacts}

	// Make sure the shape types are in order.
	if a.Order() > b.Order() {
		info.a = b
		info.b = a
	}

	BuiltinCollisionFuncs[info.a.Order()+info.b.Order()*SHAPE_TYPE_NUM](info.a, info.b, info)

	return info
}
