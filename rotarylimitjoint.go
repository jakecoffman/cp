package cp

import "math"

type RotaryLimitJoint struct {
	*Constraint

	Min, Max float64

	iSum, bias, jAcc float64
}

func NewRotaryLimitJoint(a, b *Body, min, max float64) *Constraint {
	joint := &RotaryLimitJoint{
		Min: min,
		Max: max,
	}
	joint.Constraint = NewConstraint(joint, a, b)
	return joint.Constraint
}

func (joint *RotaryLimitJoint) PreStep(dt float64) {
	a := joint.a
	b := joint.b

	dist := b.a - a.a
	pdist := 0.0
	if dist > joint.Max {
		pdist = joint.Max - dist
	} else if dist < joint.Min {
		pdist = joint.Min - dist
	}

	joint.iSum = 1.0/(a.i_inv + b.i_inv)

	maxBias := joint.maxBias
	joint.bias = Clamp(-bias_coef(joint.errorBias, dt)*pdist/dt, -maxBias, maxBias)

	if joint.bias == 0 {
		joint.jAcc = 0
	}
}

func (joint *RotaryLimitJoint) ApplyCachedImpulse(dt_coef float64) {
	a := joint.a
	b := joint.b

	j := joint.jAcc*dt_coef
	a.w -= j*a.i_inv
	b.w += j*b.i_inv
}

func (joint *RotaryLimitJoint) ApplyImpulse(dt float64) {
	if joint.bias == 0 {
		return
	}

	a := joint.a
	b := joint.b

	wr := b.w - a.w

	jMax := joint.maxForce*dt

	j := -(joint.bias + wr)*joint.iSum
	jOld := joint.jAcc
	if joint.bias < 0 {
		joint.jAcc = Clamp(jOld + j, 0, jMax)
	} else {
		joint.jAcc = Clamp(jOld + j, -jMax, 0)
	}
	j = joint.jAcc - jOld

	a.w -= j*a.i_inv
	b.w += j*b.i_inv
}

func (joint *RotaryLimitJoint) GetImpulse() float64 {
	return math.Abs(joint.jAcc)
}

