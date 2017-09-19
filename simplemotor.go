package cp

import "math"

type SimpleMotor struct {
	*Constraint

	Rate float64

	iSum, jAcc float64
}

func NewSimpleMotor(a, b *Body, rate float64) *Constraint {
	motor := &SimpleMotor{
		Rate: rate,
	}
	motor.Constraint = NewConstraint(motor, a, b)
	return motor.Constraint
}

func (motor *SimpleMotor) PreStep(dt float64) {
	a := motor.a
	b := motor.b

	// moment of inertia coefficient
	motor.iSum = 1.0 / (a.i_inv + b.i_inv)
}

func (motor *SimpleMotor) ApplyCachedImpulse(dt_coef float64) {
	a := motor.a
	b := motor.b

	j := motor.jAcc * dt_coef
	a.w -= j * a.i_inv
	b.w += j * b.i_inv
}

func (motor *SimpleMotor) ApplyImpulse(dt float64) {
	a := motor.a
	b := motor.b

	wr := b.w - a.w + motor.Rate

	jMax := motor.maxForce * dt

	j := -wr * motor.iSum
	jOld := motor.jAcc
	motor.jAcc = Clamp(jOld+j, -jMax, jMax)
	j = motor.jAcc - jOld

	a.w -= j * a.i_inv
	b.w += j * b.i_inv
}

func (motor *SimpleMotor) GetImpulse() float64 {
	return math.Abs(motor.jAcc)
}
