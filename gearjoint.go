package physics

import "math"

type GearJoint struct {
	*Constraint
	phase, ratio float64
	ratio_inv    float64

	iSum float64

	bias, jAcc float64
}

func NewGearJoint(a, b *Body, phase, ratio float64) *Constraint {
	joint := &GearJoint{
		phase:     phase,
		ratio:     ratio,
		ratio_inv: 1.0 / ratio,
	}
	constraint := NewConstraint(joint, a, b)
	joint.Constraint = constraint
	return constraint
}

func (joint *GearJoint) PreStep(constraint *Constraint, dt float64) {
	a := joint.Constraint.a
	b := joint.Constraint.b

	// calculate moment of inertia coefficient.
	joint.iSum = 1.0 / (a.i_inv*joint.ratio_inv + joint.ratio*b.i_inv)

	// calculate bias velocity
	maxBias := joint.Constraint.maxBias
	joint.bias = Clamp(-bias_coef(joint.Constraint.errorBias, dt)*(b.a*joint.ratio-a.a-joint.phase)/dt, -maxBias, maxBias)
}

func (joint *GearJoint) ApplyCachedImpulse(constraint *Constraint, dt_coef float64) {
	a := joint.Constraint.a
	b := joint.Constraint.b

	j := joint.jAcc * dt_coef
	a.w -= j * a.i_inv * joint.ratio_inv
	b.w += j * b.i_inv
}

func (joint *GearJoint) ApplyImpulse(constraint *Constraint, dt float64) {
	a := joint.Constraint.a
	b := joint.Constraint.b

	// compute relative rotational velocity
	wr := b.w*joint.ratio - a.w

	jMax := joint.Constraint.maxForce * dt

	// compute normal impulse
	j := (joint.bias - wr) * joint.iSum
	jOld := joint.jAcc
	joint.jAcc = Clamp(jOld+j, -jMax, jMax)
	j = joint.jAcc - jOld

	// apply impulse
	a.w -= j * a.i_inv * joint.ratio_inv
	b.w += j * b.i_inv
}

func (joint *GearJoint) GetImpulse() float64 {
	return math.Abs(joint.jAcc)
}
