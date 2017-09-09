package physics

import "math"

type Constrainer interface {
	PreStep(constraint *Constraint, dt float64)
	ApplyCachedImpulse(constraint *Constraint, dt_coef float64)
	ApplyImpulse(constraint *Constraint, dt float64)
	GetImpulse() float64
}

type ConstraintPreSolveFunc func(*Constraint, *Space)
type ConstraintPostSolveFunc func(*Constraint, *Space)

type Constraint struct {
	Class Constrainer
	space *Space

	a, b *Body
	next_a, next_b *Constraint

	maxForce, errorBias, maxBias float64

	collideBodies bool
	preSolve      ConstraintPreSolveFunc
	postSolve     ConstraintPostSolveFunc

	userData interface{}
}

func NewConstraint(class Constrainer, a, b *Body) *Constraint {
	return &Constraint{
		Class: class,
		a:     a,
		b:     b,
		space: nil,

		maxForce:  INFINITY,
		errorBias: math.Pow(1.0-0.1, 60.0),
		maxBias:   INFINITY,

		collideBodies: true,
		preSolve:      nil,
		postSolve:     nil,
	}
}

func (c *Constraint) ActivateBodies() {
	c.a.Activate()
	c.b.Activate()
}

func (c *Constraint) SetMaxForce(max float64) {
	assert(max >= 0.0, "Must be positive")
	c.ActivateBodies()
	c.maxForce = max
}

func (c *Constraint) SetMaxBias(max float64) {
	assert(max >= 0, "Must be positive")
	c.ActivateBodies()
	c.maxBias = max
}

func (c *Constraint) SetErrorBias(errorBias float64) {
	assert(errorBias >= 0, "Must be positive")
	c.ActivateBodies()
	c.errorBias = errorBias
}

func (c *Constraint) Next(body *Body) *Constraint {
	if c.a == body {
		return c.next_a
	} else {
		return c.next_b
	}
}
