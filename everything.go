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
	v0, n Vector
}

const POLY_SHAPE_INLINE_ALLOC = 6

type PolyShape struct {
	shape Shape

	r float64

	count int

	// The untransformed planes are appended at the end of the transformed planes.
	planes *SplittingPlane

	// Allocate a small number of splitting planes internally for simple poly.
	_planes []SplittingPlane
}

func NewPolyShape() *PolyShape {
	return &PolyShape{
		_planes: make([]SplittingPlane, 2*POLY_SHAPE_INLINE_ALLOC),
	}
}

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

type BB struct{}

type ShapeFilter struct {
	/// Two objects with the same non-zero group value do not collide.
	/// This is generally used to group objects in a composite object together to disable self collisions.
	group uint
	/// A bitmask of user definable categories that this object belongs to.
	/// The category/mask combinations of both objects in a collision must agree for a collision to occur.
	categories uint
	/// A bitmask of user definable category types that this object object collides with.
	/// The category/mask combinations of both objects in a collision must agree for a collision to occur.
	mask uint
}
