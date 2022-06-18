package cp

import "math"

type SlideJoint struct {
	*Constraint

	AnchorA, AnchorB Vector
	Min, Max         float64

	r1, r2, n Vector
	nMass     float64

	jnAcc, bias float64
}

func NewSlideJoint(a, b *Body, anchorA, anchorB Vector, min, max float64) *Constraint {
	joint := &SlideJoint{
		AnchorA: anchorA,
		AnchorB: anchorB,
		Min:     min,
		Max:     max,
		jnAcc:   0,
	}
	joint.Constraint = NewConstraint(joint, a, b)
	return joint.Constraint
}

func (joint *SlideJoint) PreStep(dt float64) {
	a := joint.a
	b := joint.b

	joint.r1 = a.transform.Vect(joint.AnchorA.Sub(a.cog))
	joint.r2 = b.transform.Vect(joint.AnchorB.Sub(b.cog))

	delta := b.p.Add(joint.r2).Sub(a.p.Add(joint.r1))
	dist := delta.Length()
	pdist := 0.0
	if dist > joint.Max {
		pdist = dist - joint.Max
		joint.n = delta.Normalize()
	} else if dist < joint.Min {
		pdist = joint.Min - dist
		joint.n = delta.Normalize().Neg()
	} else {
		joint.n = Vector{}
		joint.jnAcc = 0
	}

	// calculate the mass normal
	joint.nMass = 1.0 / k_scalar(a, b, joint.r1, joint.r2, joint.n)

	// calculate bias velocity
	maxBias := joint.maxBias
	joint.bias = Clamp(-bias_coef(joint.errorBias, dt)*pdist/dt, -maxBias, maxBias)
}

func (joint *SlideJoint) ApplyCachedImpulse(dt_coef float64) {
	a := joint.a
	b := joint.b

	j := joint.n.Mult(joint.jnAcc * dt_coef)
	apply_impulses(a, b, joint.r1, joint.r2, j)
}

func (joint *SlideJoint) ApplyImpulse(dt float64) {
	if joint.n.Equal(Vector{}) {
		return
	}

	a := joint.a
	b := joint.b
	n := joint.n
	r1 := joint.r1
	r2 := joint.r2

	vr := relative_velocity(a, b, r1, r2)
	vrn := vr.Dot(n)

	jn := (joint.bias - vrn) * joint.nMass
	jnOld := joint.jnAcc
	joint.jnAcc = Clamp(jnOld+jn, -joint.maxForce*dt, 0)
	jn = joint.jnAcc - jnOld

	apply_impulses(a, b, joint.r1, joint.r2, n.Mult(jn))
}

func (joint *SlideJoint) GetImpulse() float64 {
	return math.Abs(joint.jnAcc)
}
