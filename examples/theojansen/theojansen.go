package main

import (
	"math"

	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

var motor *SimpleMotor

func main() {
	space := NewSpace()
	space.Iterations = 20
	space.SetGravity(Vector{0, -500})

	var shape *Shape
	var a, b Vector

	walls := []Vector{
		{-320, -240}, {-320, 240},
		{320, -240}, {320, 240},
		{-320, -240}, {320, -240},
	}

	for i := 0; i < len(walls)-1; i += 2 {
		shape = space.AddShape(NewSegment(space.StaticBody, walls[i], walls[i+1], 0))
		shape.SetElasticity(1)
		shape.SetFriction(1)
		shape.SetFilter(examples.NotGrabbableFilter)
	}

	offset := 30.0
	chassisMass := 2.0
	a = Vector{-offset, 0}
	b = Vector{offset, 0}
	chassis := space.AddBody(NewBody(chassisMass, MomentForSegment(chassisMass, a, b, 0)))

	shape = space.AddShape(NewSegment(chassis, a, b, segRadius))
	shape.SetFilter(NewShapeFilter(1, ALL_CATEGORIES, ALL_CATEGORIES))

	crankMass := 1.0
	crankRadius := 13.0
	crank := space.AddBody(NewBody(crankMass, MomentForCircle(crankMass, crankRadius, 0, VectorZero())))

	shape = space.AddShape(NewCircle(crank, crankRadius, VectorZero()))
	shape.SetFilter(NewShapeFilter(1, ALL_CATEGORIES, ALL_CATEGORIES))

	space.AddConstraint(NewPivotJoint2(chassis, crank, VectorZero(), VectorZero()))

	side := 30.0

	const numLegs = 2
	for i := 0; i < numLegs; i++ {
		makeLeg(space, side, offset, chassis, crank, ForAngle(float64(2*i+0)/numLegs*math.Pi).Mult(crankRadius))
		makeLeg(space, side, -offset, chassis, crank, ForAngle(float64(2*i+1)/numLegs*math.Pi).Mult(crankRadius))
	}

	motor = space.AddConstraint(NewSimpleMotor(chassis, crank, 6)).Class.(*SimpleMotor)

	examples.Main(space, 640, 480, 1/180.0, update)
}

const segRadius = 3.0

func makeLeg(space *Space, side, offset float64, chassis, crank *Body, anchor Vector) {
	var a, b Vector
	var shape *Shape

	legMass := 1.0

	// make a leg
	a = VectorZero()
	b = Vector{0, side}
	upperLeg := space.AddBody(NewBody(legMass, MomentForSegment(legMass, a, b, 0)))
	upperLeg.SetPosition(Vector{offset, 0})

	shape = space.AddShape(NewSegment(upperLeg, a, b, segRadius))
	shape.SetFilter(NewShapeFilter(1, ALL_CATEGORIES, ALL_CATEGORIES))

	space.AddConstraint(NewPivotJoint2(chassis, upperLeg, Vector{offset, 0}, VectorZero()))

	// lower leg
	a = VectorZero()
	b = Vector{0, -1 * side}
	lowerLeg := space.AddBody(NewBody(legMass, MomentForSegment(legMass, a, b, 0)))
	lowerLeg.SetPosition(Vector{offset, -side})

	shape = space.AddShape(NewSegment(lowerLeg, a, b, segRadius))
	shape.SetFilter(NewShapeFilter(1, ALL_CATEGORIES, ALL_CATEGORIES))

	shape = space.AddShape(NewCircle(lowerLeg, segRadius*2.0, b))
	shape.SetFilter(NewShapeFilter(1, ALL_CATEGORIES, ALL_CATEGORIES))
	shape.SetElasticity(0)
	shape.SetFriction(1)

	space.AddConstraint(NewPinJoint(chassis, lowerLeg, Vector{offset, 0}, VectorZero()))

	space.AddConstraint(NewGearJoint(upperLeg, lowerLeg, 0, 1))

	var constraint *Constraint
	diag := math.Sqrt(side*side + offset*offset)

	constraint = space.AddConstraint(NewPinJoint(crank, upperLeg, anchor, Vector{0, side}))
	constraint.Class.(*PinJoint).Dist = diag

	constraint = space.AddConstraint(NewPinJoint(crank, lowerLeg, anchor, VectorZero()))
	constraint.Class.(*PinJoint).Dist = diag
}

func update(space *Space, dt float64) {
	coef := (2.0 + examples.Keyboard.Y) / 3.0
	rate := examples.Keyboard.X * 10 * coef
	motor.Rate = rate
	if rate != 0 {
		motor.SetMaxForce(100000)
	} else {
		motor.SetMaxForce(0)
	}

	space.Step(dt)
}
