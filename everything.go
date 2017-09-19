package cp

import (
	"fmt"
	"math"
)

const (
	INFINITY      = math.MaxFloat64
	MAGIC_EPSILON = 1e-5

	RadianConst = math.Pi / 180
	DegreeConst = 180 / math.Pi

	POOLED_BUFFER_SIZE = 1024
)

type CollisionBeginFunc func(arb *Arbiter, space *Space, userData interface{}) bool
type CollisionPreSolveFunc func(arb *Arbiter, space *Space, userData interface{}) bool
type CollisionPostSolveFunc func(arb *Arbiter, space *Space, userData interface{})
type CollisionSeparateFunc func(arb *Arbiter, space *Space, userData interface{})

type CollisionType uintptr

/// Struct that holds function callback pointers to configure custom collision handling.
/// Collision handlers have a pair of types; when a collision occurs between two shapes that have these types, the collision handler functions are triggered.
type CollisionHandler struct {
	/// Collision type identifier of the first shape that this handler recognizes.
	/// In the collision handler callback, the shape with this type will be the first argument. Read only.
	TypeA CollisionType
	/// Collision type identifier of the second shape that this handler recognizes.
	/// In the collision handler callback, the shape with this type will be the second argument. Read only.
	TypeB CollisionType
	/// This function is called when two shapes with types that match this collision handler begin colliding.
	BeginFunc CollisionBeginFunc
	/// This function is called each step when two shapes with types that match this collision handler are colliding.
	/// It's called before the collision solver runs so that you can affect a collision's outcome.
	PreSolveFunc CollisionPreSolveFunc
	/// This function is called each step when two shapes with types that match this collision handler are colliding.
	/// It's called after the collision solver runs so that you can read back information about the collision to trigger events in your game.
	PostSolveFunc CollisionPostSolveFunc
	/// This function is called when two shapes with types that match this collision handler stop colliding.
	SeparateFunc CollisionSeparateFunc
	/// This is a user definable context pointer that is passed to all of the collision handler functions.
	UserData interface{}
}

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

	hash HashValue
}

func (c *Contact) Clone() Contact {
	return Contact{
		r1:     c.r1,
		r2:     c.r2,
		nMass:  c.nMass,
		tMass:  c.tMass,
		bounce: c.bounce,
		jnAcc:  c.jnAcc,
		jtAcc:  c.jtAcc,
		jBias:  c.jBias,
		bias:   c.bias,
		hash:   c.hash,
	}
}

type CollisionInfo struct {
	a, b        *Shape
	collisionId uint32

	n     Vector
	count int
	arr   []Contact
}

func (info *CollisionInfo) PushContact(p1, p2 Vector, hash HashValue) {
	assert(info.count < MAX_CONTACTS_PER_ARBITER, "Internal error: Tried to push too many contacts.")

	con := &info.arr[info.count]
	con.r1 = p1
	con.r2 = p2
	con.hash = hash

	info.count++
}

type ShapeMassInfo struct {
	m, i, area float64
	cog        Vector
}

type PointQueryInfo struct {
	/// The nearest shape, NULL if no shape was within range.
	Shape *Shape
	/// The closest point on the shape's surface. (in world space coordinates)
	Point Vector
	/// The distance to the point. The distance is negative if the point is inside the shape.
	Distance float64
	/// The gradient of the signed distance function.
	/// The value should be similar to info.p/info.d, but accurate even for very small values of info.d.
	Gradient Vector
}

type SegmentQueryInfo struct {
	/// The shape that was hit, or NULL if no collision occurred.
	Shape *Shape
	/// The point of impact.
	Point Vector
	/// The normal of the surface hit.
	Normal Vector
	/// The normalized distance along the query segment in the range [0, 1].
	Alpha float64
}

type SplittingPlane struct {
	v0, n Vector
}

var (
	NO_GROUP       uint = 0
	ALL_CATEGORIES uint = ^uint(0)
)

var SHAPE_FILTER_ALL = ShapeFilter{NO_GROUP, ALL_CATEGORIES, ALL_CATEGORIES}
var SHAPE_FILTER_NONE = ShapeFilter{NO_GROUP, ^ALL_CATEGORIES, ^ALL_CATEGORIES}

type ShapeFilter struct {
	/// Two objects with the same non-zero group value do not collide.
	/// This is generally used to group objects in a composite object together to disable self collisions.
	Group uint
	/// A bitmask of user definable categories that this object belongs to.
	/// The category/mask combinations of both objects in a collision must agree for a collision to occur.
	Categories uint
	/// A bitmask of user definable category types that this object object collides with.
	/// The category/mask combinations of both objects in a collision must agree for a collision to occur.
	Mask uint
}

func NewShapeFilter(group, categories, mask uint) ShapeFilter {
	return ShapeFilter{group, categories, mask}
}

func (a ShapeFilter) Reject(b ShapeFilter) bool {
	// Reject the collision if:
	return (a.Group != 0 && a.Group == b.Group) ||
		// One of the category/mask combinations fails.
		(a.Categories&b.Mask) == 0 ||
		(b.Categories&a.Mask) == 0
}

func MomentForCircle(m, r1, r2 float64, offset Vector) float64 {
	return m * (0.5*(r1*r1+r2*r2) + offset.LengthSq())
}

func AreaForCircle(r1, r2 float64) float64 {
	return math.Pi * math.Abs(r1*r1-r2*r2)
}

func MomentForSegment(m float64, a, b Vector, r float64) float64 {
	offset := a.Lerp(b, 0.5)
	length := b.Distance(a) + 2.0*r
	return m * ((length*length+4.0*r*r)/12.0 + offset.LengthSq())
}

func AreaForSegment(a, b Vector, r float64) float64 {
	return r * (math.Pi*r + 2.0*a.Distance(b))
}

func MomentForPoly(m float64, count int, verts []Vector, offset Vector, r float64) float64 {
	if count == 2 {
		return MomentForSegment(m, verts[0], verts[1], 0)
	}

	var sum1 float64
	var sum2 float64
	for i := 0; i < count; i++ {
		v1 := verts[i].Add(offset)
		v2 := verts[(i+1)%count].Add(offset)

		a := v2.Cross(v1)
		b := v1.Dot(v1) + v1.Dot(v2) + v2.Dot(v2)

		sum1 += a * b
		sum2 += a
	}

	return (m * sum1) / (6.0 * sum2)
}

func AreaForPoly(count int, verts []Vector, r float64) float64 {
	var area float64
	var perimeter float64
	for i := 0; i < count; i++ {
		v1 := verts[i]
		v2 := verts[(i+1)%count]

		area += v1.Cross(v2)
		perimeter += v1.Distance(v2)
	}

	return r*(math.Pi*math.Abs(r)+perimeter) + area/2.0
}

func CentroidForPoly(count int, verts []Vector) Vector {
	var sum float64
	vsum := Vector{}

	for i := 0; i < count; i++ {
		v1 := verts[i]
		v2 := verts[(i+1)%count]
		cross := v1.Cross(v2)

		sum += cross
		vsum = vsum.Add(v1.Add(v2).Mult(cross))
	}

	return vsum.Mult(1.0 / (3.0 * sum))
}

func MomentForBox(m, width, height float64) float64 {
	return m * (width*width + height*height) / 12.0
}

func MomentForBox2(m float64, box BB) float64 {
	width := box.R - box.L
	height := box.T - box.B
	offset := Vector{box.L + box.R, box.B + box.T}.Mult(0.5)

	// TODO: NaN when offset is 0 and m is INFINITY
	return MomentForBox(m, width, height) + m*offset.LengthSq()
}

func assert(truth bool, msg ...interface{}) {
	if !truth {
		panic(fmt.Sprint("Assertion failed: ", msg))
	}
}

func k_scalar_body(body *Body, r, n Vector) float64 {
	rcn := r.Cross(n)
	return body.m_inv + body.i_inv*rcn*rcn
}

func k_scalar(a, b *Body, r1, r2, n Vector) float64 {
	return k_scalar_body(a, r1, n) + k_scalar_body(b, r2, n)
}

func normal_relative_velocity(a, b *Body, r1, r2, n Vector) float64 {
	return relative_velocity(a, b, r1, r2).Dot(n)
}

func k_tensor(a, b *Body, r1, r2 Vector) Mat2x2 {
	m_sum := a.m_inv + b.m_inv

	// start with Identity*m_sum
	k11 := m_sum
	k12 := 0.0
	k21 := 0.0
	k22 := m_sum

	// add the influence from r1
	a_i_inv := a.i_inv
	r1xsq := r1.X * r1.X * a_i_inv
	r1ysq := r1.Y * r1.Y * a_i_inv
	r1nxy := -r1.X * r1.Y * a_i_inv
	k11 += r1ysq
	k12 += r1nxy
	k21 += r1nxy
	k22 += r1xsq

	// add the influence from r2
	b_i_inv := b.i_inv
	r2xsq := r2.X * r2.X * b_i_inv
	r2ysq := r2.Y * r2.Y * b_i_inv
	r2nxy := -r2.X * r2.Y * b_i_inv
	k11 += r2ysq
	k12 += r2nxy
	k21 += r2nxy
	k22 += r2xsq

	// invert
	det := k11*k22 - k12*k21
	assert(det != 0.0, "Unsolvable constraint")

	det_inv := 1.0 / det
	return Mat2x2{
		k22 * det_inv, -k12 * det_inv,
		-k21 * det_inv, k11 * det_inv,
	}
}

func bias_coef(errorBias, dt float64) float64 {
	return 1.0 - math.Pow(errorBias, dt)
}

type Mat2x2 struct {
	a, b, c, d float64
}

func (m *Mat2x2) Transform(v Vector) Vector {
	return Vector{v.X*m.a + v.Y*m.b, v.X*m.c + v.Y*m.d}
}

var maxArbiters, maxPoints, maxConstraints int

func DebugInfo(space *Space) string {
	arbiters := len(space.arbiters)
	points := 0

	for i := 0; i < arbiters; i++ {
		points += int(space.arbiters[i].count)
	}

	constraints := len(space.constraints) + points*int(space.Iterations)
	if arbiters > maxArbiters {
		maxArbiters = arbiters
	}
	if points > maxPoints {
		maxPoints = points
	}
	if constraints > maxConstraints {
		maxConstraints = constraints
	}

	var ke float64
	for _, body := range space.dynamicBodies {
		if body.m == INFINITY || body.i == INFINITY {
			continue
		}
		ke += body.m*body.v.Dot(body.v) + body.i*body.w*body.w
	}

	return fmt.Sprintf(`Arbiters: %d (%d) - Contact Points: %d (%d)
Other Constraints: %d, Iterations: %d
Constraints x Iterations: %d (%d)
KE: %e`, arbiters, maxArbiters,
		points, maxPoints, len(space.constraints), space.Iterations, constraints, maxConstraints, ke)
}
