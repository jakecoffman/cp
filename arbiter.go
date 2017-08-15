package physics

import (
	"math"
)

const WILDCARD_COLLISION_TYPE = math.MaxUint64

type Arbiter struct {
	e, u       float64
	surface_vr *Vector

	data interface{}

	a, b               *Shape
	body_a, body_b     *Body
	thread_a, thread_b ArbiterThread

	count    uint
	contacts []*Contact
	n        *Vector

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
	arbiter.surface_vr = VectorZero()

	arbiter.count = 0
	arbiter.contacts = nil

	arbiter.a = a
	arbiter.body_a = a.body
	arbiter.b = b
	arbiter.body_b = b.body

	arbiter.thread_a.next = nil
	arbiter.thread_b.next = nil
	arbiter.thread_a.prev = nil
	arbiter.thread_b.prev = nil

	arbiter.stamp = 0
	arbiter.state = CP_ARBITER_STATE_FIRST_COLLISION

	arbiter.data = nil
	return arbiter
}

type ArbiterThread struct {
	next, prev *Arbiter
}

func (node *Arbiter) Next(body *Body) *Arbiter {
	if node.body_a == body {
		return node.thread_a.next
	} else {
		return node.thread_b.next
	}
}

func (arbiter *Arbiter) Unthread() {
	arbiter.unthreadHelper(arbiter.body_a)
	arbiter.unthreadHelper(arbiter.body_b)
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
	if arbiter.body_a == body {
		return &arbiter.thread_a
	} else {
		return &arbiter.thread_b
	}
}

func (arbiter *Arbiter) ApplyCachedImpulse(dt_coef float64) {
	if arbiter.IsFirstContact() {
		return
	}

	for _, contact := range arbiter.contacts {
		j := arbiter.n.Rotate(&Vector{contact.jnAcc, contact.jtAcc})
		apply_impulses(arbiter.body_a, arbiter.body_b, contact.r1, contact.r2, j.Mult(dt_coef))
	}
}

func (arbiter *Arbiter) ApplyImpulse() {
	a := arbiter.body_a
	b := arbiter.body_b
	n := arbiter.n
	surface_vr := arbiter.surface_vr
	friction := arbiter.u

	for _, con := range arbiter.contacts {
		nMass := con.nMass
		r1 := con.r1
		r2 := con.r2

		vb1 := a.v_bias.Add(r1.Perp().Mult(a.w_bias))
		vb2 := b.v_bias.Add(r2.Perp().Mult(b.w_bias))
		vr := relative_velocity(a, b, r1, r2).Add(surface_vr)

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

		apply_bias_impulses(a, b, r1, r2, n.Mult(con.jBias-jbnOld))
		apply_impulses(a, b, r1, r2, n.Rotate(&Vector{con.jnAcc - jnOld, con.jtAcc - jtOld}))
	}
}

func (arbiter *Arbiter) IsFirstContact() bool {
	return arbiter.state == CP_ARBITER_STATE_FIRST_COLLISION
}

func (arbiter *Arbiter) PreStep(i float64, i2 float64, i3 float64) {
	panic("Arbiter PreStep")
}

func (arb *Arbiter) Update(info *CollisionInfo, space *Space) {
	a := info.a
	b := info.b

	// For collisions between two similar primitive types, the order could have been swapped since the last frame.
	arb.a = a
	arb.body_a = a.body
	arb.b = b
	arb.body_b = b.body

	var i uint
	for i = 0; i < info.count; i++ {
		con := info.arr[i]

		// r1 and r2 store absolute offsets at init time.
		// Need to convert them to relative offsets.
		con.r1 = con.r1.Sub(a.body.p)
		con.r2 = con.r2.Sub(b.body.p)

		// Cached impulses are not zeroed at init time.
		con.jnAcc = 0
		con.jtAcc = 0

		var j uint
		for j = 0; j < arb.count; j++ {
			old := arb.contacts[j]

			// This could trigger false positives, but is fairly unlikely nor serious if it does.
			con.jnAcc = old.jnAcc
			con.jtAcc = old.jtAcc
		}
	}

	arb.contacts = info.arr
	arb.count = info.count
	arb.n = info.n

	arb.e = a.E * b.E
	arb.u = a.U * b.U

	surfaceVr := b.surfaceV.Sub(a.surfaceV)
	arb.surface_vr = surfaceVr.Sub(info.n.Mult(surfaceVr.Dot(info.n)))

	typeA := info.a.collisionType
	typeB := info.b.collisionType
	defaultHandler := space.defaultHandler
	handler := space.LookupHandler(typeA, typeB, defaultHandler)
	arb.handler = handler

	// Check if the types match, but don't swap for a default handler which use the wildcard for type A.
	swapped := typeA != handler.typeA && handler.typeA != WILDCARD_COLLISION_TYPE
	arb.swapped = swapped

	if handler != defaultHandler || space.usesWildcards {
		// The order of the main handler swaps the wildcard handlers too. Uffda.
		if swapped {
			arb.handlerA = space.LookupHandler(typeB, WILDCARD_COLLISION_TYPE, &CollisionHandlerDoNothing)
			arb.handlerB = space.LookupHandler(typeA, WILDCARD_COLLISION_TYPE, &CollisionHandlerDoNothing)
		} else {
			arb.handlerA = space.LookupHandler(typeA, WILDCARD_COLLISION_TYPE, &CollisionHandlerDoNothing)
			arb.handlerB = space.LookupHandler(typeB, WILDCARD_COLLISION_TYPE, &CollisionHandlerDoNothing)
		}
	}

	// mark it as new if it's been cached
	if arb.state == CP_ARBITER_STATE_CACHED {
		arb.state = CP_ARBITER_STATE_FIRST_COLLISION
	}
}

func (arb *Arbiter) Ignore() bool {
	arb.state = CP_ARBITER_STATE_IGNORE
	return false
}

func apply_impulses(a, b *Body, r1, r2, j *Vector) {
	apply_impulse(a, j.Neg(), r1)
	apply_impulse(b, j, r2)
}

func apply_bias_impulses(a, b *Body, r1, r2, j *Vector) {
	apply_bias_impulse(a, j.Neg(), r1)
	apply_bias_impulse(b, j, r2)
}

func apply_bias_impulse(body *Body, j, r *Vector) {
	body.v_bias = body.v_bias.Add(j.Mult(body.m_inv))
	body.w_bias = body.i_inv * r.Cross(j)
}

func apply_impulse(body *Body, j, r *Vector) {
	body.v = body.v.Add(j.Mult(body.m_inv))
	body.w += body.i_inv * r.Cross(j)
}

func relative_velocity(a, b *Body, r1, r2 *Vector) *Vector {
	v1_sum := a.v.Add(r1.Perp().Mult(a.w))
	v2_sum := b.v.Add(r2.Perp().Mult(b.w))
	return v2_sum.Sub(v1_sum)
}

var CollisionHandlerDoNothing CollisionHandler = CollisionHandler{
	WILDCARD_COLLISION_TYPE,
	WILDCARD_COLLISION_TYPE,
	AlwaysCollide,
	AlwaysCollide,
	DoNothing,
	DoNothing,
	nil,
}

func AlwaysCollide(_ *Arbiter, _ *Space, _ interface{}) bool {
	return true
}

func DoNothing(_ *Arbiter, _ *Space, _ interface{}) {

}
