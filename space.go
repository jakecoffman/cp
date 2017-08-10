package physics

import (
	"log"
	"math"
)

type Space struct {
	Iterations uint // must be non-zero

	gravity *Vector
	damping float64

	idleSpeedThreshold float64
	SleepTimeThreshold float64

	collisionSlop        float64
	collisionBias        float64
	collisionPersistence uint

	userData interface{}

	stamp   uint
	curr_dt float64

	dynamicBodies      []*Body
	staticBodies       []*Body
	rousedBodies       []*Body
	sleepingComponents []interface{} // <- IDK what this is

	shapeIDCounter uint
	staticShapes   *SpatialIndex
	dynamicShapes  *SpatialIndex

	constraints []*Constraint

	arbiters           []*Arbiter
	contactBuffersHead *ContactBuffer
	cachedArbiters     map[int]*Arbiter
	pooledArbiters     []*Arbiter

	allocatedBuffers []*ContactBuffer
	locked           int

	usesWildcards     bool
	collisionHandlers map[int]*CollisionHandler
	defaultHandler    CollisionHandler

	skipPostStep      bool
	postStepCallbacks []PostStepCallback

	*Body // staticBody
}

func NewSpace() *Space {
	staticBody := NewBody(0, 0)
	staticBody.SetType(BODY_STATIC)
	space := &Space{
		Iterations:           10,
		gravity:              VectorZero(),
		damping:              1.0,
		collisionSlop:        0.1,
		collisionBias:        math.Pow(0.9, 60),
		collisionPersistence: 3,
		locked:               0,
		stamp:                0,
		shapeIDCounter:       0,
		staticShapes:         &SpatialIndex{},
		dynamicShapes:        &SpatialIndex{},
		allocatedBuffers:     []*ContactBuffer{},
		dynamicBodies:        []*Body{},
		staticBodies:         []*Body{},
		sleepingComponents:   []interface{}{},
		rousedBodies:         []*Body{},
		SleepTimeThreshold:   math.MaxFloat64,
		idleSpeedThreshold:   0.0,
		arbiters:             []*Arbiter{},
		pooledArbiters:       []*Arbiter{},
		cachedArbiters:       map[int]*Arbiter{},
		constraints:          []*Constraint{},
		collisionHandlers:    map[int]*CollisionHandler{},
		postStepCallbacks:    []PostStepCallback{},
	}
	space.SetStaticBody(staticBody)
	return space
}

func (space *Space) SetStaticBody(body *Body) {
	if space.Body != nil {
		log.Println("Internal Error: Changing the designated static body while the old one still had shapes attached.")
		space.Body.space = nil
	}
	space.Body = body
	body.space = space
}

func (space *Space) PushFreshContactBuffer() {
	stamp := space.stamp
	head := space.contactBuffersHead

	if head == nil {
		buffer := space.AllocContactBuffer()
		buffer.InitHeader(stamp, nil)
		space.contactBuffersHead = buffer
	} else if stamp-head.next.stamp > space.collisionPersistence {
		tail := head.next
		space.contactBuffersHead = tail.InitHeader(stamp, tail)
	} else {
		buffer := space.AllocContactBuffer().InitHeader(stamp, head)
		space.contactBuffersHead = buffer
		head.next = buffer
	}
}

func (space *Space) Activate(body *Body) {
	if space.locked > 0 {
		if !Contains(space.rousedBodies, body) {
			space.rousedBodies = append(space.rousedBodies, body)
		}
		return
	}

	space.dynamicBodies = append(space.dynamicBodies, body)

	for _, shape := range body.shapeList {
		space.staticShapes.class.Remove(shape, shape.HashId())
		space.dynamicShapes.class.Insert(shape, shape.HashId())
	}

	//for _, arbiter := range body.arbiterList {
	//	bodyA := arbiter.body_a
	//
	//	// Arbiters are shared between two bodies that are always woken up together.
	//	// You only want to restore the arbiter once, so bodyA is arbitrarily chosen to own the arbiter.
	//	// The edge case is when static bodies are involved as the static bodies never actually sleep.
	//	// If the static body is bodyB then all is good. If the static body is bodyA, that can easily be checked.
	//	if body == bodyA || bodyA.GetType() == BODY_STATIC {
	//		numContacts := arbiter.count
	//		contacts := arbiter.contacts
	//
	//		// Restore contact values back to the space's contact buffer memory
	//		arbiter.contacts = space.ContactBufferGetArray()
	//
	//		panic("Not implemented") // TODO need to figure out what contact buffer array stuff is
	//	}
	//}

	for _, constraint := range body.constraintList {
		if body == constraint.a || constraint.a.GetType() == BODY_STATIC {
			space.constraints = append(space.constraints, constraint)
		}
	}
}

func (space *Space) Deactivate(body *Body) {
	for i, v := range space.dynamicBodies {
		if v == body {
			space.dynamicBodies = append(space.dynamicBodies[:i], space.dynamicBodies[i+1:]...)
			break
		}
	}

	for _, shape := range body.shapeList {
		space.dynamicShapes.class.Remove(shape, shape.hashid)
		space.staticShapes.class.Insert(shape, shape.hashid)
	}

	for arb := body.arbiterList; arb != nil; arb = ArbiterNext(arb, body) {
		bodyA := arb.body_a
		if body == bodyA || bodyA.GetType() == BODY_STATIC {
			space.UncacheArbiter(arb)

			// Save contact values to a new block of memory so they won't time out

		}
	}
}

type PostStepCallback struct {
	callback PostStepCallbackFunc
	key      interface{}
	data     interface{}
}

type PostStepCallbackFunc func(space *Space, key interface{}, data interface{})

func Contains(bodies []*Body, body *Body) bool {
	for i := 0; i < len(bodies); i++ {
		if bodies[i] == body {
			return true
		}
	}
	return false
}

func (space *Space) AddShape(shape *Shape) *Shape {
	var body *Body = shape.Body()

	// TODO assertions

	isStatic := body.GetType() == BODY_STATIC
	if !isStatic {
		body.Activate()
	}

	body.AddShape(shape)

	space.shapeIDCounter += 1
	shape.SetHashId(space.shapeIDCounter)

	shape.Update(body.transform)

	if isStatic {
		space.staticShapes.class.Insert(shape, shape.HashId())
	} else {
		space.dynamicShapes.class.Insert(shape, shape.HashId())
	}

	shape.SetSpace(space)
	return shape
}

func (space *Space) AddBody(body *Body) *Body {
	if body.GetType() == BODY_STATIC {
		space.staticBodies = append(space.staticBodies, body)
	} else {
		space.dynamicBodies = append(space.dynamicBodies, body)
	}
	body.space = space
	return body
}

var ShapeUpdateFunc = func(shape interface{}, _ interface{}) {
	(shape.(*Shape)).CacheBB()
}

var SpaceCollideShapesFunc = func(va, vb interface{}, collisionId uint, vspace interface{}) uint {
	a := va.(*Shape)
	b := vb.(*Shape)
	space := vspace.(*Space)

	// Reject any of the simple cases
	if QueryReject(a, b) {
		return collisionId
	}

	// Narrow-phase collision detection.
	info := Collide(a, b, collisionId, space.ContactBufferGetArray())

	if info.count == 0 {
		// shapes are not colliding
		return info.collisionId
	}

	space.contactBuffersHead.numContacts += info.count

	// Get an arbiter from space->arbiterSet for the two shapes.
	// This is where the persistent contact magic comes from.
	_ = []*Shape{info.a, info.b}
	_ = HashPair(uint(info.a), uint(info.b))

	return 0 // TODO IMPLEMENT
}

func QueryReject(a, b *Shape) bool {
	return !a.bb.Intersects(b.bb) || a.body == b.body || a.Filter.Reject(b.Filter) || QueryRejectConstraints(a.body, b.body)
}

func QueryRejectConstraints(a, b *Body) bool {
	for _, constraint := range a.constraintList {
		if !constraint.collideBodies && ((constraint.a == a && constraint.b == b) ||
			(constraint.a == b && constraint.b == a)) {
			return true
		}
	}

	return false
}

func (space *Space) ProcessComponents(dt float64) {
	sleep := space.SleepTimeThreshold != INFINITY
	bodies := space.dynamicBodies

	/* TODO: Debug? */
	for _, body := range bodies {
		if body.sleeping.next != nil {
			log.Println("Dangling pointer in contact graph (next)")
		}
		if body.sleeping.root != nil {
			log.Println("Dangling pointer in contact graph (root)")
		}
	}
	/* end TODO */

	// calculate the kinetic energy of all the bodies
	if sleep {
		dv := space.idleSpeedThreshold
		var dvsq float64
		if dv {
			dvsq = dv*dv
		} else {
			dvsq = space.gravity.LengthSq()*dt*dt
		}

		// update idling and reset component nodes
		for _, body := range bodies {
			if body.GetType() != BODY_DYNAMIC {
				continue
			}

			// Need to deal with infinite mass objects
			var keThreshold float64
			if dvsq != 0 {
				keThreshold = body.m*dvsq
			}
			if body.KineticEnergy() > keThreshold {
				body.sleeping.idleTime = 0
			} else {
				body.sleeping.idleTime += dt
			}
		}

		// Awaken any sleeping bodies found and then push arbiters to the bodies' lists.
		for _, arb := range space.arbiters {
			a := arb.body_a
			b := arb.body_b

			if sleep {
				if b.GetType() == BODY_KINEMATIC || a.IsSleeping() {
					a.Activate()
				}
				if a.GetType() == BODY_KINEMATIC || b.IsSleeping() {
					b.Activate()
				}
			}

			a.PushArbiter(arb)
			b.PushArbiter(arb)
		}

		if sleep {
			// Bodies should be held active if connected by a joint to a kinematic.
			constraints := space.constraints
			for _, constraint := range constraints {
				a := constraint.a
				b := constraint.b

				if b.GetType() == BODY_KINEMATIC {
					b.Activate()
				}
				if a.GetType() == BODY_KINEMATIC {
					a.Activate()
				}
			}

			for i := 0; i<len(bodies); {
				body := bodies[i]

				if body.ComponentRoot() == nil {
					// Body not in a component yet. Perform a DFS to flood fill mark
					// the component in the contact graph using this body as the root.
					FloodFillComponent(body, body)

					// Check if the component should be put to sleep.
					if !ComponentActive(body, space.SleepTimeThreshold) {
						space.sleepingComponents = append(space.sleepingComponents, body)
						for item := space.sleeping.root; item != nil; item = space.sleeping.next {
							space.Deactivate(item)
						}

						// cpSpaceDeactivateBody() removed the current body from the list.
						// Skip incrementing the index counter.
						continue
					}
				}

				i++

				// Only sleeping bodies retain their component node pointers.
				body.sleeping.root = nil
				body.sleeping.next = nil
			}
		}
	}
}

func ComponentActive(root *Body, threshold float64) bool {
	for item := root; item != nil; item = root.sleeping.next {
		if item.sleeping.idleTime < threshold {
			return true
		}
	}
	return false
}

func FloodFillComponent(root *Body, body *Body) {
	if body.GetType() == BODY_DYNAMIC {
		other_root := body.ComponentRoot()
		if other_root == nil {
			root.ComponentAdd(body)
			for arb := body.arbiterList; arb != nil; arb = ArbiterNext(arb, body) {
				if body == arb.body_a {
					FloodFillComponent(root, arb.body_b)
				} else {
					FloodFillComponent(root, arb.body_a)
				}
			}

			for _, constraint := range body.constraintList {
				if body == constraint.a {
					FloodFillComponent(root, constraint.b)
				} else {
					FloodFillComponent(root, constraint.a)
				}
			}
		} else {
			if other_root == root {
				log.Println("Inconsistency detected in the contact graph (FFC)")
			}
		}
	}
}

func ArbiterNext(arb *Arbiter, body *Body) *Arbiter {
	if arb.body_a == body {
		return arb.thread_a.next
	}
	return arb.thread_b.next
}

func (space *Space) Step(dt float64) {
	if dt == 0 {
		return
	}

	space.stamp++

	prev_dt := space.curr_dt
	space.curr_dt = dt

	bodies := space.dynamicBodies
	constraints := space.constraints
	arbiters := space.arbiters

	// reset and empty the arbiter lists
	for _, arb := range arbiters {
		arb.state = CP_ARBITER_STATE_NORMAL

		// If both bodies are awake, unthread the arbiter from the contact graph.
		if !arb.body_a.IsSleeping() && !arb.body_b.IsSleeping() {
			arb.Unthread()
		}
	}
	space.arbiters = space.arbiters[0:0]

	space.Lock()
	{
		for _, body := range bodies {
			body.position_func(body, dt)
		}
		space.PushFreshContactBuffer()
		space.dynamicShapes.class.Each(ShapeUpdateFunc, nil)
		space.dynamicShapes.class.ReindexQuery(SpaceCollideShapesFunc, space)
	}
	space.Unlock(false)

	// Rebuild the contact graph (and detect sleeping components if sleeping is enabled)
	space.ProcessComponents(dt)

	space.Lock()
	{
		// Clear out old cached arbiters and call separate callbacks
		for k, v := range space.cachedArbiters {
			if SpaceArbiterSetFilter(v, space) == false {
				delete(space.cachedArbiters, k)
			}
		}

		// Prestep the arbiters and constraints.
		slop := space.collisionSlop
		biasCoef := 1 - math.Pow(space.collisionBias, dt)
		for _, arbiter := range arbiters {
			arbiter.PreStep(dt, slop, biasCoef)
		}

		for _, constraint := range constraints {
			preSolve := constraint.preSolve
			if preSolve != nil {
				(*preSolve)(constraint, space)
			}

			constraint.class.PreStep(constraint, dt)
		}

		// Integrate velocities.
		damping := math.Pow(space.damping, dt)
		gravity := space.gravity
		for _, body := range bodies {
			body.velocity_func(body, gravity, damping, dt)
		}

		// Apply cached impulses
		var dt_coef float64
		if prev_dt == 0 {
			dt_coef = 0
		} else {
			dt_coef = dt/prev_dt
		}
		for _, arbiter := range arbiters {
			arbiter.ApplyCachedImpulse(dt_coef)
		}

		for _, constraint := range constraints {
			constraint.class.ApplyCachedImpulse(constraint, dt_coef)
		}

		// Run the impulse solver.
		var i uint
		for i = 0; i<space.Iterations; i++ {
			for j := 0; j<len(arbiters); j++ {
				arbiters[j].ApplyImpulse()
			}

			for _, constraint := range constraints {
				constraint.class.ApplyImpulse(constraint, dt)
			}
		}

		// Run the constraint post-solve callbacks
		for _, constraint := range constraints {
			if constraint.postSolve != nil {
				(*constraint.postSolve)(constraint, space)
			}
		}

		// run the post-solve callbacks
		for _, arb := range arbiters {
			arb.handler.postSolveFunc(arb, space, arb.handler)
		}
	}
	space.Unlock(true)
}

// Hashset filter func to throw away old arbiters.
func SpaceArbiterSetFilter(arb *Arbiter, space *Space) bool {
	ticks := space.stamp - arb.stamp
	a := arb.body_a
	b := arb.body_b

	// TODO: should make an arbiter state for this so it doesn't require filtering arbiters for dangling body pointers on body removal.
	// Preserve arbiters on sensors and rejected arbiters for sleeping objects.
	// This prevents errant separate callbacks from happening.

	if (a.GetType() == BODY_STATIC || a.IsSleeping()) && (b.GetType() == BODY_STATIC || b.IsSleeping()) {
		return true
	}

	if ticks >= 1 && arb.state != CP_ARBITER_STATE_CACHED {
		arb.state = CP_ARBITER_STATE_CACHED
		handler := arb.handler
		handler.separateFunc(arb, space, handler.userData)
	}

	if ticks >= space.collisionPersistence {
		arb.contacts = nil
		arb.count = 0

		space.pooledArbiters = append(space.pooledArbiters, arb)
		return false
	}

	return true
}

func (space *Space) Lock() {
	space.locked++
}

func (space *Space) Unlock(runPostStep bool) {
	space.locked--

	if space.locked >= 0 {
		log.Fatal("Space lock underflow")
	}

	if space.locked != 0 {
		return
	}

	waking := space.rousedBodies

	for i := 0; i < len(waking); i++ {
		space.Activate(waking[i])
		waking[i] = nil
	}
	waking = waking[0:0]
}

func (space *Space) AllocContactBuffer() *ContactBuffer {
	buffer := &ContactBuffer{
		contacts: [CONTACTS_BUFFER_SIZE]*Contact{},
	}
	space.allocatedBuffers = append(space.allocatedBuffers, buffer)
	return buffer
}

const MAX_CONTACTS_PER_ARBITER = 2
const CONTACTS_BUFFER_SIZE = 256

func (space *Space) ContactBufferGetArray() []*Contact {
	if space.contactBuffersHead.numContacts+MAX_CONTACTS_PER_ARBITER > CONTACTS_BUFFER_SIZE {
		space.PushFreshContactBuffer()
	}

	head := space.contactBuffersHead
	return head.contacts[head.numContacts]
}
