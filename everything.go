package physics

import (
	"math"
	"time"
)

const INFINITY = math.MaxFloat64

type CollisionHandler func()

// Arbiter states
const (
	// Arbiter is active and its the first collision.
	CP_ARBITER_STATE_FIRST_COLLISION = iota
	// Arbiter is active and its not the first collision.
	CP_ARBITER_STATE_NORMAL
	// Collision has been explicitly ignored.
	// Either by returning false from a begin collision handler or calling cpArbiterIgnore().
	CP_ARBITER_STATE_IGNORE
	// Collison is no longer active. A space will cache an arbiter for up to cpSpace.collisionPersistence more steps.
	CP_ARBITER_STATE_CACHED
	// Collison arbiter is invalid because one of the shapes was removed.
	CP_ARBITER_STATE_INVALIDATED
)

type Contact struct {
	r1, r2 Vector

	nMass, tMass float64
	bounce       float64 // TODO: look for an alternate bounce solution

	jnAcc, jtAcc, jBias float64
	bias                float64

	hash uint
}

type CollisionInfo struct {
	a, b        *Shape
	collisionId uint

	n     Vector
	count int
	arr   []*Contact
}

type Arbiter struct {
	e, u       float64
	surface_vr Vector

	data interface{}

	a, b           *Shape
	body_a, body_b *Body

	count    int
	contacts []*Contact
	n        Vector

	// Regular, wildcard A and wildcard B collision handlers.
	handler, handlerA, handlerB CollisionHandler
	swapped                     bool

	stamp time.Time
	state int // Arbiter state enum
}

type ShapeMassInfo struct {
	m, i, area float64
	cog        *Vector
}

// Shape Class
const (
	SHAPE_CLASS_CIRCLE = iota
	SHAPE_CLASS_SEGMENT
	SHAPE_CLASS_POLY
	SHAPE_CLASS_NUM
)

type PointQueryInfo struct {
	/// The nearest shape, NULL if no shape was within range.
	shape *Shape
	/// The closest point on the shape's surface. (in world space coordinates)
	point Vector
	/// The distance to the point. The distance is negative if the point is inside the shape.
	distance float64
	/// The gradient of the signed distance function.
	/// The value should be similar to info.p/info.d, but accurate even for very small values of info.d.
	gradient Vector
}

type SegmentQueryInfo struct {
	/// The shape that was hit, or NULL if no collision occured.
	shape *Shape
	/// The point of impact.
	point Vector
	/// The normal of the surface hit.
	normal Vector
	/// The normalized distance along the query segment in the range [0, 1].
	alpha float64
}

type Circle struct {
	shape Shape
	c, tc Vector
	r     float64
}

type SplittingPlane struct {
	v0, n *Vector
}

const POLY_SHAPE_INLINE_ALLOC = 6

type Constrainer interface {
	PreStep()
	ApplyCachedImpulse()
	ApplyImpulse()
	GetImpulse()
}

type ConstraintPreSolveFunc func(*Constraint, *Space)
type ConstraintPostSolveFunc func(*Constraint, *Space)

type Constraint struct {
	space *Space

	a, b           *Body
	next_a, next_b *Constraint

	maxForce, errorBias, maxBias float64

	collideBodies bool
	preSolve      ConstraintPreSolveFunc
	postSolve     ConstraintPostSolveFunc

	userData interface{}
}

type PinJoint struct {
	constraint       Constraint
	anchorA, anchorB Vector
	dist             float64

	r1, r2 Vector
	n      Vector
	nMass  float64

	jnAcc, bias float64
}

//
//const (
//	NO_GROUP = 0
//	GRABBABLE_MASK_BIT = 1<<31
//
//)

type ShapeFilter struct {
	/// Two objects with the same non-zero group value do not collide.
	/// This is generally used to group objects in a composite object together to disable self collisions.
	group uintptr
	/// A bitmask of user definable categories that this object belongs to.
	/// The category/mask combinations of both objects in a collision must agree for a collision to occur.
	categories uint
	/// A bitmask of user definable category types that this object object collides with.
	/// The category/mask combinations of both objects in a collision must agree for a collision to occur.
	mask int64
}

//var GrabFilter *ShapeFilter = &ShapeFilter{NO_GROUP, GRABBABLE_MASK_BIT, GRABBABLE_MASK_BIT}
//var NotGrabbableFilter *ShapeFilter = &ShapeFilter{NO_GROUP, ^GRABBABLE_MASK_BIT, ^GRABBABLE_MASK_BIT}

func MomentForCircle(m, r1, r2 float64, offset *Vector) float64 {
	return m * (0.5*(r1*r1+r2*r2) + offset.LengthSq())
}

func AreaForCircle(r1, r2 float64) float64 {
	return math.Pi * math.Abs(r1*r1-r2*r2)
}

func MomentForSegment(m float64, a, b *Vector, r float64) float64 {
	offset := a.Lerp(b, 0.5)
	length := b.Distance(a) + 2*r
	return m * ((length*length+4*r*r)/12 + offset.LengthSq())
}

func AreaForSegment(a, b *Vector, r float64) float64 {
	return r * (math.Pi*r + 2*a.Distance(b))
}

func MomentForPoly(m float64, verts []*Vector, offset *Vector, r float64) float64 {
	if len(verts) == 2 {
		return MomentForSegment(m, verts[0], verts[1], 0)
	}

	var sum1 float64
	var sum2 float64
	for i := range verts {
		v1 := verts[i].Add(offset)
		v2 := verts[(i+1)%len(verts)].Add(offset)

		a := v2.Cross(v1)
		b := v1.Dot(v1) + v1.Dot(v2) + v2.Dot(v2)

		sum1 += a * b
		sum2 += a
	}

	return (m * sum1) / (6 * sum2)
}

func AreaForPoly(verts []*Vector, r float64) float64 {
	var area float64
	var perimiter float64
	for i := range verts {
		v1 := verts[i]
		v2 := verts[(i+1)%len(verts)]

		area += v1.Cross(v2)
		perimiter += v1.Distance(v2)
	}

	return r*(math.Pi*math.Abs(r)+perimiter) + area/2
}

func CentroidForPoly(verts []*Vector) *Vector {
	var sum float64 = 0
	vsum := VectorZero()

	for i := range verts {
		v1 := verts[i]
		v2 := verts[(i+1)%len(verts)]
		cross := v1.Cross(v2)

		sum += cross
		vsum = vsum.Add(v1.Add(v2).Mult(cross))
	}

	return vsum.Mult(1 / (3 * sum))
}

func MomentForBox(m, width, height float64) float64 {
	return m * (width*width + height*height) / 12
}

func MomentForBox2(m float64, box *BB) float64 {
	width := box.r - box.l
	height := box.t - box.b
	offset := (&Vector{box.l + box.r, box.b + box.t}).Mult(0.5)

	// TODO: NaN when offset is 0 and m is INFINITY
	return MomentForBox(m, width, height) + m*offset.LengthSq()
}
