package cp

import "math"

var wildcardCollisionType = ^CollisionType(0)

type Arbiter struct {
	e, u      float64
	surfaceVr Vector

	UserData interface{}

	a, b               *Shape
	bodyA, bodyB     *Body
	threadA, threadB ArbiterThread

	count int
	// a slice onto the current buffer array of contacts
	contacts []Contact
	n        Vector

	// Regular, wildcard A and wildcard B collision handlers.
	handler, handlerA, handlerB *CollisionHandler
	swapped                     bool

	stamp uint
	state int // Arbiter state enum
}

func (arbiter *Arbiter) Init(a, b *Shape) *Arbiter {
	arbiter.handler = nil
	arbiter.swapped = false

	arbiter.handlerA = nil
	arbiter.handlerB = nil

	arbiter.e = 0
	arbiter.u = 0
	arbiter.surfaceVr = Vector{}

	arbiter.count = 0
	arbiter.contacts = nil

	arbiter.a = a
	arbiter.bodyA = a.body
	arbiter.b = b
	arbiter.bodyB = b.body

	arbiter.threadA.next = nil
	arbiter.threadB.next = nil
	arbiter.threadA.prev = nil
	arbiter.threadB.prev = nil

	arbiter.stamp = 0
	arbiter.state = CP_ARBITER_STATE_FIRST_COLLISION

	arbiter.UserData = nil
	return arbiter
}

type ArbiterThread struct {
	next, prev *Arbiter
}

func (arbiter *Arbiter) Next(body *Body) *Arbiter {
	if arbiter.bodyA == body {
		return arbiter.threadA.next
	} else {
		return arbiter.threadB.next
	}
}

func (arbiter *Arbiter) Unthread() {
	arbiter.unthreadHelper(arbiter.bodyA)
	arbiter.unthreadHelper(arbiter.bodyB)
}

func (arbiter *Arbiter) unthreadHelper(body *Body) {
	thread := arbiter.ThreadForBody(body)
	prev := thread.prev
	next := thread.next

	if prev != nil {
		prev.ThreadForBody(body).next = next
	} else if body.arbiterList == arbiter {
		// IFF prev is NULL and body->arbiterList == arb, is arb at the head of the list.
		// This function may be called for an arbiter that was never in a list.
		// In that case, we need to protect it from wiping out the body->arbiterList pointer.
		body.arbiterList = next
	}

	if next != nil {
		next.ThreadForBody(body).prev = prev
	}

	thread.next = nil
	thread.prev = nil
}

func (arbiter *Arbiter) ThreadForBody(body *Body) *ArbiterThread {
	if arbiter.bodyA == body {
		return &arbiter.threadA
	} else {
		return &arbiter.threadB
	}
}

func (arbiter *Arbiter) ApplyCachedImpulse(dtCoef float64) {
	if arbiter.IsFirstContact() {
		return
	}

	for i := 0; i < arbiter.count; i++ {
		contact := arbiter.contacts[i]
		j := arbiter.n.Rotate(Vector{contact.jnAcc, contact.jtAcc})
		applyImpulses(arbiter.bodyA, arbiter.bodyB, contact.r1, contact.r2, j.Mult(dtCoef))
	}
}

func (arbiter *Arbiter) ApplyImpulse() {
	a := arbiter.bodyA
	b := arbiter.bodyB
	n := arbiter.n
	surfaceVr := arbiter.surfaceVr
	friction := arbiter.u

	for i := 0; i < arbiter.count; i++ {
		con := &arbiter.contacts[i]
		nMass := con.nMass
		r1 := con.r1
		r2 := con.r2

		vb1 := a.v_bias.Add(r1.Perp().Mult(a.w_bias))
		vb2 := b.v_bias.Add(r2.Perp().Mult(b.w_bias))
		vr := relativeVelocity(a, b, r1, r2).Add(surfaceVr)

		vbn := vb2.Sub(vb1).Dot(n)
		vrn := vr.Dot(n)
		vrt := vr.Dot(n.Perp())

		jbn := (con.bias - vbn) * nMass
		jbnOld := con.jBias
		con.jBias = math.Max(jbnOld+jbn, 0)

		jn := -(con.bounce + vrn) * nMass
		jnOld := con.jnAcc
		con.jnAcc = math.Max(jnOld+jn, 0)

		jtMax := friction * con.jnAcc
		jt := -vrt * con.tMass
		jtOld := con.jtAcc
		con.jtAcc = Clamp(jtOld+jt, -jtMax, jtMax)

		applyBiasImpulses(a, b, r1, r2, n.Mult(con.jBias-jbnOld))
		applyImpulses(a, b, r1, r2, n.Rotate(Vector{con.jnAcc - jnOld, con.jtAcc - jtOld}))
	}
}

func (arbiter *Arbiter) IsFirstContact() bool {
	return arbiter.state == CP_ARBITER_STATE_FIRST_COLLISION
}

func (arbiter *Arbiter) PreStep(dt, slop, bias float64) {
	a := arbiter.bodyA
	b := arbiter.bodyB
	n := arbiter.n
	bodyDelta := b.p.Sub(a.p)

	for i := 0; i < arbiter.count; i++ {
		con := &arbiter.contacts[i]

		// Calculate the mass normal and mass tangent.
		con.nMass = 1.0 / k_scalar(a, b, con.r1, con.r2, n)
		con.tMass = 1.0 / k_scalar(a, b, con.r1, con.r2, n.Perp())

		// Calculate the target bias velocity.
		dist := con.r2.Sub(con.r1).Add(bodyDelta).Dot(n)
		con.bias = -bias * math.Min(0, dist+slop) / dt
		con.jBias = 0.0

		// Calculate the target bounce velocity.
		con.bounce = normal_relative_velocity(a, b, con.r1, con.r2, n) * arbiter.e
	}
}

func (arbiter *Arbiter) Update(info *CollisionInfo, space *Space) {
	a := info.a
	b := info.b

	// For collisions between two similar primitive types, the order could have been swapped since the last frame.
	arbiter.a = a
	arbiter.bodyA = a.body
	arbiter.b = b
	arbiter.bodyB = b.body

	// Iterate over the possible pairs to look for hash value matches.
	for i := 0; i < info.count; i++ {
		con := &info.arr[i]

		// r1 and r2 store absolute offsets at init time.
		// Need to convert them to relative offsets.
		con.r1 = con.r1.Sub(a.body.p)
		con.r2 = con.r2.Sub(b.body.p)

		// Cached impulses are not zeroed at init time.
		con.jnAcc = 0
		con.jtAcc = 0

		for j := 0; j < arbiter.count; j++ {
			old := arbiter.contacts[j]

			// This could trigger false positives, but is fairly unlikely nor serious if it does.
			if con.hash == old.hash {
				// Copy the persistent contact information.
				con.jnAcc = old.jnAcc
				con.jtAcc = old.jtAcc
			}
		}
	}

	arbiter.contacts = info.arr[:info.count]
	arbiter.count = info.count
	arbiter.n = info.n

	arbiter.e = a.e * b.e
	arbiter.u = a.u * b.u

	surfaceVr := b.surfaceV.Sub(a.surfaceV)
	arbiter.surfaceVr = surfaceVr.Sub(info.n.Mult(surfaceVr.Dot(info.n)))

	typeA := info.a.collisionType
	typeB := info.b.collisionType
	handler := space.LookupHandler(typeA, typeB, space.defaultHandler)
	arbiter.handler = handler

	// Check if the types match, but don't swap for a default handler which use the wildcard for type A.
	swapped := typeA != handler.TypeA && handler.TypeA != wildcardCollisionType
	arbiter.swapped = swapped

	if handler != space.defaultHandler || space.usesWildcards {
		// The order of the main handler swaps the wildcard handlers too. Uffda.
		if swapped {
			arbiter.handlerA = space.LookupHandler(typeB, wildcardCollisionType, &CollisionHandlerDoNothing)
			arbiter.handlerB = space.LookupHandler(typeA, wildcardCollisionType, &CollisionHandlerDoNothing)
		} else {
			arbiter.handlerA = space.LookupHandler(typeA, wildcardCollisionType, &CollisionHandlerDoNothing)
			arbiter.handlerB = space.LookupHandler(typeB, wildcardCollisionType, &CollisionHandlerDoNothing)
		}
	}

	// mark it as new if it's been cached
	if arbiter.state == CP_ARBITER_STATE_CACHED {
		arbiter.state = CP_ARBITER_STATE_FIRST_COLLISION
	}
}

func (arbiter *Arbiter) Ignore() bool {
	arbiter.state = CP_ARBITER_STATE_IGNORE
	return false
}

func (arbiter *Arbiter) CallWildcardBeginA(space *Space) bool {
	handler := arbiter.handlerA
	return handler.BeginFunc(arbiter, space, handler.UserData)
}

func (arbiter *Arbiter) CallWildcardBeginB(space *Space) bool {
	handler := arbiter.handlerB
	arbiter.swapped = !arbiter.swapped
	retVal := handler.BeginFunc(arbiter, space, handler.UserData)
	arbiter.swapped = !arbiter.swapped
	return retVal
}

func (arbiter *Arbiter) CallWildcardPreSolveA(space *Space) bool {
	handler := arbiter.handlerA
	return handler.PreSolveFunc(arbiter, space, handler.UserData)
}

func (arbiter *Arbiter) CallWildcardPreSolveB(space *Space) bool {
	handler := arbiter.handlerB
	arbiter.swapped = !arbiter.swapped
	retval := handler.PreSolveFunc(arbiter, space, handler.UserData)
	arbiter.swapped = !arbiter.swapped
	return retval
}

func (arbiter *Arbiter) CallWildcardPostSolveA(space *Space) {
	handler := arbiter.handlerA
	handler.PostSolveFunc(arbiter, space, handler.UserData)
}

func (arbiter *Arbiter) CallWildcardPostSolveB(space *Space) {
	handler := arbiter.handlerB
	arbiter.swapped = !arbiter.swapped
	handler.PostSolveFunc(arbiter, space, handler.UserData)
	arbiter.swapped = !arbiter.swapped
}

func (arbiter *Arbiter) CallWildcardSeparateA(space *Space) {
	handler := arbiter.handlerA
	handler.SeparateFunc(arbiter, space, handler.UserData)
}

func (arbiter *Arbiter) CallWildcardSeparateB(space *Space) {
	handler := arbiter.handlerB
	arbiter.swapped = !arbiter.swapped
	handler.SeparateFunc(arbiter, space, handler.UserData)
	arbiter.swapped = !arbiter.swapped
}

func applyImpulses(a, b *Body, r1, r2, j Vector) {
	b.v.X += j.X * b.m_inv
	b.v.Y += j.Y * b.m_inv
	b.w += b.i_inv * (r2.X*j.Y - r2.Y*j.X)

	j.X = -j.X
	j.Y = -j.Y
	a.v.X += j.X * a.m_inv
	a.v.Y += j.Y * a.m_inv
	a.w += a.i_inv * (r1.X*j.Y - r1.Y*j.X)
}

func applyImpulse(body *Body, j, r Vector) {
	body.v.X += j.X * body.m_inv
	body.v.Y += j.Y * body.m_inv
	body.w += body.i_inv * r.Cross(j)
}

func applyBiasImpulses(a, b *Body, r1, r2, j Vector) {
	b.v_bias.X += j.X * b.m_inv
	b.v_bias.Y += j.Y * b.m_inv
	b.w_bias += b.i_inv * (r2.X*j.Y - r2.Y*j.X)

	j.X = -j.X
	j.Y = -j.Y
	a.v_bias.X += j.X * a.m_inv
	a.v_bias.Y += j.Y * a.m_inv
	a.w_bias += a.i_inv * (r1.X*j.Y - r1.Y*j.X)
}

func relativeVelocity(a, b *Body, r1, r2 Vector) Vector {
	return r2.Perp().Mult(b.w).Add(b.v).Sub(r1.Perp().Mult(a.w).Add(a.v))
}

var CollisionHandlerDoNothing = CollisionHandler{
	wildcardCollisionType,
	wildcardCollisionType,
	AlwaysCollide,
	AlwaysCollide,
	DoNothing,
	DoNothing,
	nil,
}

var CollisionHandlerDefault = CollisionHandler{
	wildcardCollisionType,
	wildcardCollisionType,
	DefaultBegin,
	DefaultPreSolve,
	DefaultPostSolve,
	DefaultSeparate,
	nil,
}

func AlwaysCollide(_ *Arbiter, _ *Space, _ interface{}) bool {
	return true
}

func DoNothing(_ *Arbiter, _ *Space, _ interface{}) {

}

func DefaultBegin(arb *Arbiter, space *Space, _ interface{}) bool {
	return arb.CallWildcardBeginA(space) && arb.CallWildcardBeginB(space)
}

func DefaultPreSolve(arb *Arbiter, space *Space, _ interface{}) bool {
	return arb.CallWildcardPreSolveA(space) && arb.CallWildcardPreSolveB(space)
}

func DefaultPostSolve(arb *Arbiter, space *Space, _ interface{}) {
	arb.CallWildcardPostSolveA(space)
	arb.CallWildcardPostSolveB(space)
}

func DefaultSeparate(arb *Arbiter, space *Space, _ interface{}) {
	arb.CallWildcardSeparateA(space)
	arb.CallWildcardSeparateB(space)
}

func (arbiter *Arbiter) TotalImpulse() Vector {
	var sum Vector

	count := arbiter.Count()
	for i := 0; i < count; i++ {
		con := arbiter.contacts[i]
		sum = sum.Add(arbiter.n.Rotate(Vector{con.jnAcc, con.jtAcc}))
	}

	if arbiter.swapped {
		return sum
	}
	return sum.Neg()
}

func (arbiter *Arbiter) Count() int {
	if arbiter.state < CP_ARBITER_STATE_CACHED {
		return arbiter.count
	}
	return 0
}

func (arbiter *Arbiter) Shapes() (*Shape, *Shape) {
	if arbiter.swapped {
		return arbiter.b, arbiter.a
	} else {
		return arbiter.a, arbiter.b
	}
}

func (arbiter *Arbiter) Bodies() (*Body, *Body) {
	shapeA, shapeB := arbiter.Shapes()
	return shapeA.body, shapeB.body
}

func (arbiter *Arbiter) Normal() Vector {
	if arbiter.swapped {
		return arbiter.n.Mult(-1)
	} else {
		return arbiter.n
	}
}

type ContactPointSet struct {
	Count  int
	Normal Vector

	Points [maxContactsPerArbiter]struct {
		/// The position of the contact on the surface of each shape.
		PointA, PointB Vector
		/// Penetration distance of the two shapes. Overlapping means it will be negative.
		/// This value is calculated as cpvdot(cpvsub(point2, point1), normal) and is ignored by cpArbiterSetContactPointSet().
		Distance float64
	}
}

func (arbiter *Arbiter) ContactPointSet() ContactPointSet {
	var set ContactPointSet
	set.Count = arbiter.Count()

	swapped := arbiter.swapped
	n := arbiter.n
	if swapped {
		set.Normal = n.Neg()
	} else {
		set.Normal = n
	}

	for i := 0; i < set.Count; i++ {
		// Contact points are relative to body CoGs;
		p1 := arbiter.bodyA.p.Add(arbiter.contacts[i].r1)
		p2 := arbiter.bodyB.p.Add(arbiter.contacts[i].r2)

		if swapped {
			set.Points[i].PointA = p2
			set.Points[i].PointB = p1
		} else {
			set.Points[i].PointA = p1
			set.Points[i].PointB = p2
		}

		set.Points[i].Distance = p2.Sub(p1).Dot(n)
	}

	return set
}

func (arbiter *Arbiter) SetContactPointSet(set *ContactPointSet) {
	count := set.Count
	assert(count == arbiter.count)

	swapped := arbiter.swapped
	if swapped {
		arbiter.n = set.Normal.Neg()
	} else {
		arbiter.n = set.Normal
	}

	for i := 0; i < count; i++ {
		p1 := set.Points[i].PointA
		p2 := set.Points[i].PointB

		if swapped {
			arbiter.contacts[i].r1 = p2.Sub(arbiter.bodyA.p)
			arbiter.contacts[i].r2 = p1.Sub(arbiter.bodyB.p)
		} else {
			arbiter.contacts[i].r1 = p1.Sub(arbiter.bodyA.p)
			arbiter.contacts[i].r2 = p2.Sub(arbiter.bodyB.p)
		}
	}
}
