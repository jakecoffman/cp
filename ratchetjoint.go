package cp

import "math"

type RatchetJoint struct {
	*Constraint

	Angle, Phase, Ratchet float64

	iSum, bias, jAcc float64
}

func NewRatchetJoint(a, b *Body, phase, ratchet float64) *Constraint {
	joint := &RatchetJoint{
		Phase: phase,
		Ratchet: ratchet,
	}
	if b != nil {
		joint.Angle = b.a
	}
	if a != nil {
		joint.Angle -= a.a
	}
	joint.Constraint = NewConstraint(joint, a, b)
	return joint.Constraint
}

func (joint *RatchetJoint) PreStep(dt float64) {
	a := joint.a
	b := joint.b

	angle := joint.Angle
	phase := joint.Phase
	ratchet := joint.Ratchet

	delta := b.a - a.a
	diff := angle-delta
	pdist := 0.0

	if diff*ratchet > 0 {
		pdist = diff
	} else {
		joint.Angle = math.Floor((delta-phase)/ratchet)*ratchet+phase
	}

	joint.iSum = 1.0/(a.i_inv+b.i_inv)

	maxBias := joint.maxBias
	joint.bias = Clamp(-bias_coef(joint.errorBias, dt)*pdist/dt, -maxBias, maxBias)

	if joint.bias == 0 {
		joint.jAcc = 0
	}
}

func (joint *RatchetJoint) ApplyCachedImpulse(dt_coef float64) {
	a := joint.a
	b := joint.b

	j := joint.jAcc*dt_coef
	a.w -= j*a.i_inv
	b.w += j*b.i_inv
}

func (joint *RatchetJoint) ApplyImpulse(dt float64) {
	if joint.bias == 0 {
		return
	}

	a := joint.a
	b := joint.b

	wr := b.w - a.w
	ratchet := joint.Ratchet

	jMax := joint.maxForce*dt

	j := -(joint.bias+wr)*joint.iSum
	jOld := joint.jAcc
	joint.jAcc = Clamp((jOld+j)*ratchet, 0, jMax*math.Abs(ratchet))/ratchet
	j = joint.jAcc - jOld

	a.w -= j*a.i_inv
	b.w += j*b.i_inv
}

func (joint *RatchetJoint) GetImpulse() float64 {
	return math.Abs(joint.jAcc)
}
