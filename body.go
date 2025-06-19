package cp

import (
	"fmt"
	"math"
)

// Body types
const (
	BODY_DYNAMIC = iota
	BODY_KINEMATIC
	BODY_STATIC
)

var bodyCur int = 0

// BodyVelocityFunc is rigid body velocity update function type.
type BodyVelocityFunc func(body *Body, gravity Vector, damping float64, dt float64)

// BodyPositionFunc is rigid body position update function type.
type BodyPositionFunc func(body *Body, dt float64)

type Body struct {
	// UserData is an object that this constraint is associated with.
	//
	// You can use this get a reference to your game object or controller object from within callbacks.
	UserData any

	id            int              // Body id
	velocity_func BodyVelocityFunc // Integration function
	position_func BodyPositionFunc // Integration function

	m     float64 // Mass
	m_inv float64 // Mass inverse

	i     float64 // Moment of inertia
	i_inv float64 // Inverse of moment of inertia i

	cog Vector // Center of gravity

	p Vector // Position
	v Vector // Velocity
	f Vector // Force

	a float64 // Angle (radians)
	w float64 // Angular velocity,
	t float64 // Torque (radians)

	transform Transform

	// "pseudo-velocities" used for eliminating overlap.
	// Erin Catto has some papers that talk about what these are.
	v_bias Vector
	w_bias float64

	space *Space

	shapeList      []*Shape
	arbiterList    *Arbiter
	constraintList *Constraint

	sleepingRoot     *Body
	sleepingNext     *Body
	sleepingIdleTime float64
}

// String returns body id as string
func (b Body) String() string {
	return fmt.Sprint("Body ", b.id)
}

// NewBody Initializes a rigid body with the given mass and moment of inertia.
//
// Guessing the moment of inertia is usually a bad idea. Use the moment estimation functions MomentFor*().
func NewBody(mass, moment float64) *Body {
	body := &Body{
		id:            bodyCur,
		cog:           Vector{},
		p:             Vector{},
		v:             Vector{},
		f:             Vector{},
		v_bias:        Vector{},
		transform:     NewTransformIdentity(),
		velocity_func: BodyUpdateVelocity,
		position_func: BodyUpdatePosition,
	}
	bodyCur++

	body.SetMass(mass)
	body.SetMoment(moment)
	body.SetAngle(0)

	return body
}

// NewStaticBody allocates and initializes a Body, and set it as a static body.
func NewStaticBody() *Body {
	body := NewBody(0, 0)
	body.SetType(BODY_STATIC)
	return body
}

// NewKinematicBody allocates and initializes a Body, and set it as a kinematic body.
func NewKinematicBody() *Body {
	body := NewBody(0, 0)
	body.SetType(BODY_KINEMATIC)
	return body
}

// SetAngle sets the angle of body.
func (body *Body) SetAngle(angle float64) {
	body.Activate()
	body.a = angle
	body.SetTransform(body.p, angle)
}

// Moment returns moment of inertia of the body.
func (body Body) Moment() float64 {
	return body.i
}

// SetMoment sets moment of inertia of the body.
func (body *Body) SetMoment(moment float64) {
	body.Activate()
	body.i = moment
	body.i_inv = 1 / moment
}

// Mass returns mass of the body
func (body *Body) Mass() float64 {
	return body.m
}

// SetMass sets mass of the body
func (body *Body) SetMass(mass float64) {
	body.Activate()
	body.m = mass
	body.m_inv = 1 / mass
}

// IdleTime returns sleeping idle time of the body
func (body *Body) IdleTime() float64 {
	return body.sleepingIdleTime
}

// SetType sets the type of the body.
func (body *Body) SetType(newType int) {
	oldType := body.GetType()
	if oldType == newType {
		return
	}

	if newType == BODY_STATIC {
		body.sleepingIdleTime = INFINITY
	} else {
		body.sleepingIdleTime = 0
	}

	if newType == BODY_DYNAMIC {
		body.m = 0
		body.i = 0
		body.m_inv = INFINITY
		body.i_inv = INFINITY

		body.AccumulateMassFromShapes()
	} else {
		body.m = INFINITY
		body.i = INFINITY
		body.m_inv = 0
		body.i_inv = 0

		body.v = Vector{}
		body.w = 0
	}

	// If the body is added to a space already, we'll need to update some space data structures.
	if body.space == nil {
		return
	}
	assert(body.space.locked == 0, "Space is locked")

	if oldType != BODY_STATIC {
		body.Activate()
	}

	if oldType == BODY_STATIC {
		for i, b := range body.space.staticBodies {
			if b == body {
				body.space.staticBodies = append(body.space.staticBodies[:i], body.space.staticBodies[i+1:]...)
				break
			}
		}
		body.space.dynamicBodies = append(body.space.dynamicBodies, body)
	} else if newType == BODY_STATIC {
		for i, b := range body.space.dynamicBodies {
			if b == body {
				body.space.dynamicBodies = append(body.space.dynamicBodies[:i], body.space.dynamicBodies[i+1:]...)
				break
			}
		}
		body.space.staticBodies = append(body.space.staticBodies, body)
	}

	var fromIndex, toIndex *SpatialIndex
	if oldType == BODY_STATIC {
		fromIndex = body.space.staticShapes
	} else {
		fromIndex = body.space.dynamicShapes
	}

	if newType == BODY_STATIC {
		toIndex = body.space.staticShapes
	} else {
		toIndex = body.space.dynamicShapes
	}

	if oldType != newType {
		for _, shape := range body.shapeList {
			fromIndex.class.Remove(shape, shape.hashid)
			toIndex.class.Insert(shape, shape.hashid)
		}
	}
}

// GetType returns the type of the body.
func (body *Body) GetType() int {
	if body.sleepingIdleTime == INFINITY {
		return BODY_STATIC
	}
	if body.m == INFINITY {
		return BODY_KINEMATIC
	}
	return BODY_DYNAMIC
}

// AccumulateMassFromShapes should *only* be called when shapes with mass info are modified, added or removed.
func (body *Body) AccumulateMassFromShapes() {
	if body == nil || body.GetType() != BODY_DYNAMIC {
		return
	}

	body.m = 0
	body.i = 0
	body.cog = Vector{}

	// cache position, realign at the end
	pos := body.Position()

	for _, shape := range body.shapeList {
		info := shape.MassInfo()
		m := info.m

		if info.m > 0 {
			msum := body.m + m
			body.i += m*info.i + body.cog.DistanceSq(info.cog)*(m*body.m)/msum
			body.cog = body.cog.Lerp(info.cog, m/msum)
			body.m = msum
		}
	}

	body.m_inv = 1.0 / body.m
	body.i_inv = 1.0 / body.i

	body.SetPosition(pos)
}

// CenterOfGravity returns the offset of the center of gravity in body local coordinates.
func (body Body) CenterOfGravity() Vector {
	return body.cog
}

// Angle returns the angle of the body.
func (body *Body) Angle() float64 {
	return body.a
}

// Rotation returns the rotation vector of the body.
//
// (The x basis vector of it's transform.)
func (body *Body) Rotation() Vector {
	return Vector{body.transform.a, body.transform.b}
}

// Position returns the position of the body.
func (body *Body) Position() Vector {
	return body.transform.Point(Vector{})
}

// SetPosition sets the position of the body.
func (body *Body) SetPosition(position Vector) {
	body.Activate()
	body.p = body.transform.Vect(body.cog).Add(position)
	body.SetTransform(body.p, body.a)
}

// Velocity returns the velocity of the body.
func (body *Body) Velocity() Vector {
	return body.v
}

// SetVelocity sets the velocity of the body.
//
// Shorthand for Body.SetVelocityVector()
func (body *Body) SetVelocity(x, y float64) {
	body.Activate()
	body.v = Vector{x, y}
}

// SetVelocityVector sets the velocity of the body
func (body *Body) SetVelocityVector(v Vector) {
	body.Activate()
	body.v = v
}

// UpdateVelocity is the default velocity integration function.
func (body *Body) UpdateVelocity(gravity Vector, damping, dt float64) {
	if body.GetType() == BODY_KINEMATIC {
		return
	}

	assert(body.m > 0 && body.i > 0, "Body's mass and moment must be positive")

	body.v = body.v.Mult(damping).Add(gravity.Add(body.f.Mult(body.m_inv)).Mult(dt))
	body.w = body.w*damping + body.t*body.i_inv*dt

	body.f = Vector{}
	body.t = 0
}

// Force returns the force applied to the body for the next time step.
func (body *Body) Force() Vector {
	return body.f
}

// SetForce sets the force applied to the body for the next time step.
func (body *Body) SetForce(force Vector) {
	body.Activate()
	body.f = force
}

// Torque returns the torque applied to the body for the next time step.
func (body *Body) Torque() float64 {
	return body.t
}

// SetTorque sets the torque applied to the body for the next time step.
func (body *Body) SetTorque(torque float64) {
	body.Activate()
	body.t = torque
}

// AngularVelocity returns the angular velocity of the body.
func (body *Body) AngularVelocity() float64 {
	return body.w
}

// SetAngularVelocity sets the angular velocity of the body.
func (body *Body) SetAngularVelocity(angularVelocity float64) {
	body.Activate()
	body.w = angularVelocity
}

// SetTransform sets transform
func (body *Body) SetTransform(p Vector, a float64) {
	rot := Vector{math.Cos(a), math.Sin(a)}
	c := body.cog

	body.transform = NewTransformTranspose(
		rot.X, -rot.Y, p.X-(c.X*rot.X-c.Y*rot.Y),
		rot.Y, rot.X, p.Y-(c.X*rot.Y+c.Y*rot.X),
	)
}

// Activate wakes up a sleeping or idle body.
func (body *Body) Activate() {
	if !(body != nil && body.GetType() == BODY_DYNAMIC) {
		return
	}

	body.sleepingIdleTime = 0

	root := body.ComponentRoot()
	if root != nil && root.IsSleeping() {
		assert(root.GetType() == BODY_DYNAMIC, "Non-dynamic root")
		space := root.space
		// in the chipmunk code they shadow body, so here I am not
		bodyToo := root
		for bodyToo != nil {
			next := bodyToo.sleepingNext
			bodyToo.sleepingIdleTime = 0
			bodyToo.sleepingRoot = nil
			bodyToo.sleepingNext = nil
			space.Activate(bodyToo)

			bodyToo = next
		}

		for i := range space.sleepingComponents {
			if space.sleepingComponents[i] == root {
				space.sleepingComponents = append(space.sleepingComponents[:i], space.sleepingComponents[i+1:]...)
				break
			}
		}
	}

	for arbiter := body.arbiterList; arbiter != nil; arbiter = arbiter.Next(body) {
		// Reset the idle timer of things the body is touching as well.
		// That way things don't get left hanging in the air.
		var other *Body
		if arbiter.body_a == body {
			other = arbiter.body_b
		} else {
			other = arbiter.body_a
		}
		if other.GetType() != BODY_STATIC {
			other.sleepingIdleTime = 0
		}
	}
}

// ActivateStatic wakes up any sleeping or idle bodies touching a static body.
func (body *Body) ActivateStatic(filter *Shape) {
	assert(body.GetType() == BODY_STATIC)

	for arb := body.arbiterList; arb != nil; arb = arb.Next(body) {
		if filter == nil || filter == arb.a || filter == arb.b {
			if arb.body_a == body {
				arb.body_b.Activate()
			} else {
				arb.body_a.Activate()
			}
		}
	}
}

// IsSleeping returns true if the body is sleeping.
func (body *Body) IsSleeping() bool {
	return body.sleepingRoot != nil
}

// AddShape adds shape to the body and returns added shape
func (body *Body) AddShape(shape *Shape) *Shape {
	body.shapeList = append(body.shapeList, shape)
	if shape.MassInfo().m > 0 {
		body.AccumulateMassFromShapes()
	}
	return shape
}

// KineticEnergy returns the kinetic energy of this body.
func (body *Body) KineticEnergy() float64 {
	// Need to do some fudging to avoid NaNs
	vsq := body.v.Dot(body.v)
	wsq := body.w * body.w
	var a, b float64
	if vsq != 0 {
		a = vsq * body.m
	}
	if wsq != 0 {
		b = wsq * body.i
	}
	return a + b
}

func (body *Body) PushArbiter(arb *Arbiter) {
	next := body.arbiterList
	arb.ThreadForBody(body).next = next
	if next != nil {
		next.ThreadForBody(body).prev = arb
	}
	body.arbiterList = arb
}

func (root *Body) ComponentAdd(body *Body) {
	body.sleepingRoot = root

	if body != root {
		body.sleepingNext = root.sleepingNext
		root.sleepingNext = body
	}
}

func (body *Body) ComponentRoot() *Body {
	if body != nil {
		return body.sleepingRoot
	}
	return nil
}

// WorldToLocal converts from world to body local Coordinates.
//
// Convert a point in body local coordinates to world (absolute) coordinates.
func (body *Body) WorldToLocal(point Vector) Vector {
	return NewTransformRigidInverse(body.transform).Point(point)
}

// LocalToWorld converts from body local to world coordinates.
//
// Convert a point in world (absolute) coordinates to body local coordinates affected by the position and rotation of the rigid body.
func (body *Body) LocalToWorld(point Vector) Vector {
	return body.transform.Point(point)
}

// ApplyForceAtWorldPoint applies a force at world point.
func (body *Body) ApplyForceAtWorldPoint(force, point Vector) {
	body.Activate()
	body.f = body.f.Add(force)

	r := point.Sub(body.transform.Point(body.cog))
	body.t += r.Cross(force)
}

// ApplyForceAtLocalPoint applies a force at local point.
func (body *Body) ApplyForceAtLocalPoint(force, point Vector) {
	body.ApplyForceAtWorldPoint(body.transform.Vect(force), body.transform.Point(point))
}

// ApplyImpulseAtWorldPoint applies impulse at world point
func (body *Body) ApplyImpulseAtWorldPoint(impulse, point Vector) {
	body.Activate()

	r := point.Sub(body.transform.Point(body.cog))
	apply_impulse(body, impulse, r)
}

// ApplyImpulseAtLocalPoint applies impulse at local point
func (body *Body) ApplyImpulseAtLocalPoint(impulse, point Vector) {
	body.ApplyImpulseAtWorldPoint(body.transform.Vect(impulse), body.transform.Point(point))
}

// VelocityAtLocalPoint returns the velocity of a point on a body.
//
// Get the world (absolute) velocity of a point on a rigid body specified in body local coordinates.
func (body *Body) VelocityAtLocalPoint(point Vector) Vector {
	r := body.transform.Vect(point.Sub(body.cog))
	return body.v.Add(r.Perp().Mult(body.w))
}

// VelocityAtWorldPoint returns the velocity of a point on a body.
//
// Get the world (absolute) velocity of a point on a rigid body specified in world coordinates.
func (body *Body) VelocityAtWorldPoint(point Vector) Vector {
	r := point.Sub(body.transform.Point(body.cog))
	return body.v.Add(r.Perp().Mult(body.w))
}

// RemoveConstraint removes constraint from the body.
func (body *Body) RemoveConstraint(constraint *Constraint) {
	body.constraintList = filterConstraints(body.constraintList, body, constraint)
}

// RemoveShape removes collision shape from the body.
func (body *Body) RemoveShape(shape *Shape) {
	for i, s := range body.shapeList {
		if s == shape {
			// leak-free delete from slice
			last := len(body.shapeList) - 1
			body.shapeList[i] = body.shapeList[last]
			body.shapeList[last] = nil
			body.shapeList = body.shapeList[:last]
			break
		}
	}
	if body.GetType() == BODY_DYNAMIC && shape.massInfo.m > 0 {
		body.AccumulateMassFromShapes()
	}
}

// SetVelocityUpdateFunc sets the callback used to update a body's velocity.
func (body *Body) SetVelocityUpdateFunc(f BodyVelocityFunc) {
	body.velocity_func = f
}

// SetPositionUpdateFunc sets the callback used to update a body's position.
func (body *Body) SetPositionUpdateFunc(f BodyPositionFunc) {
	body.position_func = f
}

// EachArbiter calls f once for each arbiter that is currently active on the body.
func (body *Body) EachArbiter(f func(*Arbiter)) {
	arb := body.arbiterList
	for arb != nil {
		next := arb.Next(body)
		swapped := arb.swapped

		arb.swapped = body == arb.body_b
		f(arb)

		arb.swapped = swapped
		arb = next
	}
}

// EachShape calls f once for each shape attached to this body
func (body *Body) EachShape(f func(*Shape)) {
	for i := range body.shapeList {
		f(body.shapeList[i])
	}
}

// EachConstraint calls f once for each constraint attached to this body
func (body *Body) EachConstraint(f func(*Constraint)) {
	constraint := body.constraintList
	for constraint != nil {
		next := constraint.Next(body)
		f(constraint)
		constraint = next
	}
}

func filterConstraints(node *Constraint, body *Body, filter *Constraint) *Constraint {
	if node == filter {
		return node.Next(body)
	} else if node.a == body {
		node.next_a = filterConstraints(node.next_a, body, filter)
	} else {
		node.next_b = filterConstraints(node.next_b, body, filter)
	}
	return node
}

// BodyUpdateVelocity is default velocity integration function.
func BodyUpdateVelocity(body *Body, gravity Vector, damping, dt float64) {
	if body.GetType() == BODY_KINEMATIC {
		return
	}

	body.v = body.v.Mult(damping).Add(gravity.Add(body.f.Mult(body.m_inv)).Mult(dt))
	body.w = body.w*damping + body.t*body.i_inv*dt

	body.f = Vector{}
	body.t = 0
}

// BodyUpdatePosition is default position integration function.
func BodyUpdatePosition(body *Body, dt float64) {
	body.p = body.p.Add(body.v.Add(body.v_bias).Mult(dt))
	body.a = body.a + (body.w+body.w_bias)*dt
	body.SetTransform(body.p, body.a)

	body.v_bias = Vector{}
	body.w_bias = 0
}
