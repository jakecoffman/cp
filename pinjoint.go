package cp

import "math"

type PinJoint struct {
	*Constraint
	AnchorA, AnchorB Vector
	Dist float64

	r1, r2, n Vector
	nMass, jnAcc, bias float64
}

func NewPinJoint(a, b *Body, anchorA, anchorB Vector) *Constraint {
	joint := &PinJoint{
		AnchorA: anchorA,
		AnchorB: anchorB,
	}

	// static body check
	var p1, p2 Vector
	if a != nil {
		p1 = a.transform.Point(anchorA)
	} else {
		p1 = anchorA
	}
	if b != nil {
		p2 = b.transform.Point(anchorB)
	} else {
		p2 = anchorB
	}
	joint.Dist = p2.Sub(p1).Length()

	// TODO: warn about joint.dist > 0 being unstable, use pivot joint

	joint.jnAcc = 0

	joint.Constraint = NewConstraint(joint, a, b)
	return joint.Constraint
}

func (joint *PinJoint) PreStep(dt float64) {
	a := joint.a
	b := joint.b

	joint.r1 = a.transform.Vect(joint.AnchorA.Sub(a.cog))
	joint.r2 = b.transform.Vect(joint.AnchorB.Sub(b.cog))

	delta := b.p.Add(joint.r2.Sub(a.p.Add(joint.r1)))
	dist := delta.Length()
	if dist != 0 {
		joint.n = delta.Mult(1 / dist)
	} else {
		joint.n = delta.Mult(1 / INFINITY)
	}

	joint.nMass = 1/k_scalar(a, b, joint.r1, joint.r2, joint.n)

	maxBias := joint.maxBias
	joint.bias = Clamp(-bias_coef(joint.errorBias, dt)*(dist - joint.Dist)/dt, -maxBias, maxBias)
}

func (joint *PinJoint) ApplyCachedImpulse(dt_coef float64) {
	j := joint.n.Mult(joint.jnAcc*dt_coef)
	apply_impulses(joint.a, joint.b, joint.r1, joint.r2, j)
}

func (joint *PinJoint) ApplyImpulse(dt float64) {
	a := joint.a
	b := joint.b
	n := joint.n

	vrn := normal_relative_velocity(a, b, joint.r1, joint.r2, n)

	jnMax := joint.maxForce*dt

	jn := (joint.bias - vrn)*joint.nMass
	jnOld := joint.jnAcc
	joint.jnAcc = Clamp(jnOld+jn, -jnMax, jnMax)
	jn = joint.jnAcc - jnOld

	apply_impulses(a, b, joint.r1, joint.r2, n.Mult(jn))
}

func (joint *PinJoint) GetImpulse() float64 {
	return math.Abs(joint.jnAcc)
}
