package main

import (
	"math"

	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

func main() {
	space := NewSpace()
	space.Iterations = 10
	space.SetGravity(Vector{0, -100})
	space.SleepTimeThreshold = 0.5

	walls := []Vector{
		{-320, 240}, {320, 240},
		{-320, 120}, {320, 120},
		{-320, 0}, {320, 0},
		{-320, -120}, {320, -120},
		{-320, -240}, {320, -240},
		{-320, -240}, {-320, 240},
		{-160, -240}, {-160, 240},
		{0, -240}, {0, 240},
		{160, -240}, {160, 240},
		{320, -240}, {320, 240},
	}

	for i := 0; i < len(walls)-1; i += 2 {
		shape := space.AddShape(NewSegment(space.StaticBody, walls[i], walls[i+1], 0))
		shape.SetElasticity(1)
		shape.SetFriction(1)
		shape.SetFilter(examples.NotGrabbableFilter)
	}

	var boxOffset Vector
	var body1, body2 *Body

	posA := Vector{50, 60}
	posB := Vector{110, 60}

	// Pin Joints - Link shapes with a solid bar or pin.
	// Keeps the anchor points the same distance apart from when the joint was created.
	boxOffset = Vector{-320, -240}
	body1 = addBall(space, posA, boxOffset)
	body2 = addBall(space, posB, boxOffset)
	space.AddConstraint(NewPinJoint(body1, body2, Vector{15, 0}, Vector{-15, 0}))

	// Slide Joints - Like pin joints but with a min/max distance.
	// Can be used for a cheap approximation of a rope.
	boxOffset = Vector{-160, -240}
	body1 = addBall(space, posA, boxOffset)
	body2 = addBall(space, posB, boxOffset)
	space.AddConstraint(NewSlideJoint(body1, body2, Vector{15, 0}, Vector{-15, 0}, 20, 40))

	// Pivot Joints - Holds the two anchor points together. Like a swivel.
	boxOffset = Vector{0, -240}
	body1 = addBall(space, posA, boxOffset)
	body2 = addBall(space, posB, boxOffset)
	space.AddConstraint(NewPivotJoint(body1, body2, boxOffset.Add(Vector{80, 60})))
	// cpPivotJointNew() takes it's anchor parameter in world coordinates. The anchors are calculated from that
	// cpPivotJointNew2() lets you specify the two anchor points explicitly

	// Groove Joints - Like a pivot joint, but one of the anchors is a line segment that the pivot can slide in
	boxOffset = Vector{160, -240}
	body1 = addBall(space, posA, boxOffset)
	body2 = addBall(space, posB, boxOffset)
	space.AddConstraint(NewGrooveJoint(body1, body2, Vector{30, 30}, Vector{30, -30}, Vector{-30, 0}))

	// Damped Springs
	boxOffset = Vector{-320, -120}
	body1 = addBall(space, posA, boxOffset)
	body2 = addBall(space, posB, boxOffset)
	space.AddConstraint(NewDampedSpring(body1, body2, Vector{15, 0}, Vector{-15, 0}, 20, 5, 0.3))

	// Damped Rotary Springs
	boxOffset = Vector{-160, -120}
	body1 = addBar(space, posA, boxOffset)
	body2 = addBar(space, posB, boxOffset)
	space.AddConstraint(NewPivotJoint(body1, space.StaticBody, boxOffset.Add(posA)))
	space.AddConstraint(NewPivotJoint(body2, space.StaticBody, boxOffset.Add(posB)))
	space.AddConstraint(NewDampedRotarySpring(body1, body2, 0, 3000, 60))

	// Rotary Limit Joint
	boxOffset = Vector{0, -120}
	body1 = addLever(space, posA, boxOffset)
	body2 = addLever(space, posB, boxOffset)
	space.AddConstraint(NewPivotJoint(body1, space.StaticBody, boxOffset.Add(posA)))
	space.AddConstraint(NewPivotJoint(body2, space.StaticBody, boxOffset.Add(posB)))
	// Hold their rotation within 90 degrees of each other.
	space.AddConstraint(NewRotaryLimitJoint(body1, body2, -math.Pi/2.0, math.Pi/2.0))

	// Ratchet Joint - A rotary ratchet, like a socket wrench
	boxOffset = Vector{160, -120}

	// Gear Joint - Maintain a specific angular velocity ratio
	boxOffset = Vector{-320, 0}
	body1 = addBar(space, posA, boxOffset)
	body2 = addBar(space, posB, boxOffset)
	space.AddConstraint(NewPivotJoint(body1, space.StaticBody, boxOffset.Add(posA)))
	space.AddConstraint(NewPivotJoint(body2, space.StaticBody, boxOffset.Add(posB)))
	// Force one to sping 2x as fast as the other
	space.AddConstraint(NewGearJoint(body1, body2, 0, 2))

	// Simple Motor - Maintain a specific angular relative velocity
	boxOffset = Vector{-160, 0}
	body1 = addBar(space, posA, boxOffset)
	body2 = addBar(space, posB, boxOffset)
	space.AddConstraint(NewPivotJoint(body1, space.StaticBody, boxOffset.Add(posA)))
	space.AddConstraint(NewPivotJoint(body2, space.StaticBody, boxOffset.Add(posB)))
	// Make them spin at 1/2 revolution per second in relation to each other.
	space.AddConstraint(NewSimpleMotor(body1, body2, math.Pi))

	// Make a car with some nice soft suspension
	boxOffset = VectorZero()
	wheel1 := addWheel(space, posA, boxOffset)
	wheel2 := addWheel(space, posB, boxOffset)
	chassis := addChassis(space, Vector{80, 100}, boxOffset)

	space.AddConstraint(NewGrooveJoint(chassis, wheel1, Vector{-30, -10}, Vector{-30, -40}, VectorZero()))
	space.AddConstraint(NewGrooveJoint(chassis, wheel2, Vector{30, -10}, Vector{30, -40}, VectorZero()))

	space.AddConstraint(NewDampedSpring(chassis, wheel1, Vector{-30, 0}, VectorZero(), 50, 20, 10))
	space.AddConstraint(NewDampedSpring(chassis, wheel2, Vector{30, 0}, VectorZero(), 50, 20, 10))

	examples.Main(space, 640, 480, 1.0/60.0, update)
}

func addBall(space *Space, pos, boxOffset Vector) *Body {
	radius := 15.0
	mass := 1.0
	body := space.AddBody(NewBody(mass, MomentForCircle(mass, 0, radius, VectorZero())))
	body.SetPosition(pos.Add(boxOffset))

	shape := space.AddShape(NewCircle(body, radius, VectorZero()))
	shape.SetElasticity(0)
	shape.SetFriction(0.7)
	return body
}

func addLever(space *Space, pos, boxOffset Vector) *Body {
	mass := 1.0
	a := Vector{0, 15}
	b := Vector{0, -15}

	body := space.AddBody(NewBody(mass, MomentForSegment(mass, a, b, 0)))
	body.SetPosition(pos.Add(boxOffset.Add(Vector{0, -15})))

	shape := space.AddShape(NewSegment(body, a, b, 5))
	shape.SetElasticity(0)
	shape.SetFriction(0.7)

	return body
}

func addBar(space *Space, pos, boxOffset Vector) *Body {
	mass := 2.0
	a := Vector{0, 30}
	b := Vector{0, -30}

	body := space.AddBody(NewBody(mass, MomentForSegment(mass, a, b, 0)))
	body.SetPosition(pos.Add(boxOffset))

	shape := space.AddShape(NewSegment(body, a, b, 5))
	shape.SetElasticity(0)
	shape.SetFriction(0.7)
	shape.SetFilter(NewShapeFilter(1, ALL_CATEGORIES, ALL_CATEGORIES))

	return body
}

func addWheel(space *Space, pos, boxOffset Vector) *Body {
	radius := 15.0
	mass := 1.0
	body := space.AddBody(NewBody(mass, MomentForCircle(mass, 0, radius, VectorZero())))
	body.SetPosition(pos.Add(boxOffset))

	shape := space.AddShape(NewCircle(body, radius, VectorZero()))
	shape.SetElasticity(0)
	shape.SetFriction(0.7)
	shape.SetFilter(NewShapeFilter(1, ALL_CATEGORIES, ALL_CATEGORIES))

	return body
}

func addChassis(space *Space, pos, boxOffset Vector) *Body {
	mass := 5.0
	width := 80.0
	height := 30.0

	body := space.AddBody(NewBody(mass, MomentForBox(mass, width, height)))
	body.SetPosition(pos.Add(boxOffset))

	shape := space.AddShape(NewBox(body, width, height, 0))
	shape.SetElasticity(0)
	shape.SetFriction(0.7)
	shape.SetFilter(NewShapeFilter(1, ALL_CATEGORIES, ALL_CATEGORIES))

	return body
}

func update(space *Space, dt float64) {
	examples.DrawString(Vector{-320, -240}, "Pin Joints")
	examples.DrawString(Vector{-160, -240}, "Slide Joints")
	examples.DrawString(Vector{0, -240}, "Pivot Joints")
	examples.DrawString(Vector{160, -240}, "Groove Joints")
	examples.DrawString(Vector{-320, -240}, "Damped Spring")
	examples.DrawString(Vector{-160, -120}, "Damped Rotary Spring")
	examples.DrawString(Vector{0, -120}, "Rotary Limit Joint")
	examples.DrawString(Vector{160, -120}, "Ratchet Joints")
	examples.DrawString(Vector{-320, 0}, "Gear Joint")
	examples.DrawString(Vector{-160, 0}, "Simple Motor")
	examples.DrawString(Vector{0, 0}, "Car")
	space.Step(dt)
}
