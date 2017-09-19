package main

import (
	"math"

	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

var balanceBody *Body
var balanceSin float64

var wheelBody *Body

func biasCoef(errorBias, dt float64) float64 {
	return 1.0 - math.Pow(errorBias, dt)
}

func motorPreSolve(motor *Constraint, space *Space) {
	dt := space.TimeStep()

	targetX := examples.Mouse.X
	examples.DrawSegment(Vector{targetX, -10000}, Vector{targetX, 1000}, FColor{1, 0, 0, 1})

	maxV := 500.0
	targetV := Clamp(biasCoef(0.5, dt/1.2)*(targetX-balanceBody.Position().X)/dt, -maxV, maxV)
	errorV := targetV - balanceBody.Velocity().X
	targetSin := 3.0e-3 * biasCoef(0.1, dt) * errorV / dt

	maxSin := math.Sin(0.6)
	balanceSin = Clamp(balanceSin-6.0e-5*biasCoef(0.2, dt)*errorV/dt, -maxSin, maxSin)
	targetA := math.Asin(Clamp(-targetSin+balanceSin, -maxSin, maxSin))
	angularDiff := math.Asin(balanceBody.Rotation().Cross(ForAngle(targetA)))
	targetW := biasCoef(0.1, dt/0.4) * angularDiff / dt

	maxRate := 50.0
	rate := Clamp(wheelBody.AngularVelocity()+balanceBody.AngularVelocity()-targetW, -maxRate, maxRate)
	motor.Class.(*SimpleMotor).Rate = Clamp(rate, -maxRate, maxRate)
}

func main() {
	space := NewSpace()
	space.Iterations = 30
	space.SetGravity(Vector{0, -500})

	{
		shape := space.AddShape(NewSegment(space.StaticBody, Vector{-3200, -240}, Vector{3200, -240}, 0))
		shape.SetElasticity(1)
		shape.SetFriction(1)
		shape.SetFilter(examples.NotGrabbableFilter)

		shape = space.AddShape(NewSegment(space.StaticBody, Vector{0, -200}, Vector{240, -240}, 0))
		shape.SetElasticity(1)
		shape.SetFriction(1)
		shape.SetFilter(examples.NotGrabbableFilter)

		shape = space.AddShape(NewSegment(space.StaticBody, Vector{-240, -240}, Vector{0, -200}, 0))
		shape.SetElasticity(1)
		shape.SetFriction(1)
		shape.SetFilter(examples.NotGrabbableFilter)
	}

	{
		radius := 20.0
		mass := 1.0

		moment := MomentForCircle(mass, 0, radius, Vector{})
		wheelBody = space.AddBody(NewBody(mass, moment))
		wheelBody.SetPosition(Vector{0, -160 + radius})

		shape := space.AddShape(NewCircle(wheelBody, radius, Vector{}))
		shape.SetFriction(0.7)
		shape.SetFilter(NewShapeFilter(1, ALL_CATEGORIES, ALL_CATEGORIES))
	}

	{
		cogOffset := 30.0

		bb1 := BB{-5, -cogOffset, 5, cogOffset*1.2 - cogOffset}
		bb2 := BB{-25, bb1.T, 25, bb1.T + 10}

		mass := 3.0
		moment := MomentForBox2(mass, bb1) + MomentForBox2(mass, bb2)

		balanceBody = space.AddBody(NewBody(mass, moment))
		balanceBody.SetPosition(Vector{0, wheelBody.Position().Y + cogOffset})

		shape := space.AddShape(NewBox2(balanceBody, bb1, 0))
		shape.SetFriction(1)
		shape.SetFilter(NewShapeFilter(1, ALL_CATEGORIES, ALL_CATEGORIES))

		shape = space.AddShape(NewBox2(balanceBody, bb2, 0))
		shape.SetFriction(1)
		shape.SetFilter(NewShapeFilter(1, ALL_CATEGORIES, ALL_CATEGORIES))
	}

	anchorA := balanceBody.WorldToLocal(wheelBody.Position())
	grooveA := anchorA.Add(Vector{0, 30})
	grooveB := anchorA.Add(Vector{0, -10})
	space.AddConstraint(NewGrooveJoint(balanceBody, wheelBody, grooveA, grooveB, Vector{}))
	space.AddConstraint(NewDampedSpring(balanceBody, wheelBody, anchorA, Vector{}, 0, 6.0e2, 30))

	motor := space.AddConstraint(NewSimpleMotor(wheelBody, balanceBody, 0))
	motor.PreSolve = motorPreSolve

	{
		width := 100.0
		height := 20.0
		mass := 3.0

		boxBody := space.AddBody(NewBody(mass, MomentForBox(mass, width, height)))
		boxBody.SetPosition(Vector{200, -100})

		shape := space.AddShape(NewBox(boxBody, width, height, 0))
		shape.SetFriction(0.7)
	}

	examples.Main(space, 1.0/60.0, update, examples.DefaultDraw)
}

func update(space *Space, dt float64) {
	space.Step(dt)
	examples.DrawString(Vector{-250, 100}, "This unicycle is completely driven and balanced by a single cpSimpleMotor.\nMove the mouse to make the unicycle follow it.")
}
