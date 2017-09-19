package main

import (
	"math"

	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

var dollyBody *Body

// Constraint used as a servo motor to move the dolly back and forth.
var dollyServo *PivotJoint

// Constraint used as a winch motor to lift the load.
var winchServo *SlideJoint

// Temporary joint used to hold the hook to the load.
var hookJoint *Constraint

func main() {
	space := NewSpace()
	space.Iterations = 30
	space.SetGravity(Vector{0, -100})
	space.SetDamping(0.8)

	shape := space.AddShape(NewSegment(space.StaticBody, Vector{-320, -240}, Vector{320, -240}, 0))
	shape.SetElasticity(1)
	shape.SetFriction(1)
	shape.SetFilter(examples.NotGrabbableFilter)

	dollyBody = space.AddBody(NewBody(10, INFINITY))
	dollyBody.SetPosition(Vector{0, 100})
	space.AddShape(NewBox(dollyBody, 30, 30, 0))
	// Add a groove joint for it to move back and forth on.
	space.AddConstraint(NewGrooveJoint(space.StaticBody, dollyBody, Vector{-250, 100}, Vector{250, 100}, Vector{}))

	// Add a pivot joint to act as a servo motor controlling it's position
	// By updating the anchor points of the pivot joint, you can move the dolly.
	dollyServo = space.AddConstraint(NewPivotJoint(space.StaticBody, dollyBody, dollyBody.Position())).Class.(*PivotJoint)
	// Max force the dolly servo can generate.
	dollyServo.SetMaxForce(10000)
	// Max speed of the dolly servo
	dollyServo.SetMaxBias(100)
	// You can also change the error bias to control how it slows down.
	//dollyServo.SetErrorBias(0.2)

	hookBody := space.AddBody(NewBody(1, INFINITY))
	hookBody.SetPosition(Vector{0, 50})

	// This will be used to figure out when the hook touches a box.
	shape = space.AddShape(NewCircle(hookBody, 10, Vector{}))
	shape.SetSensor(true)
	shape.SetCollisionType(COLLISION_HOOK)

	// By updating the max length of the joint you can make it pull up the load.
	winchServo = space.AddConstraint(NewSlideJoint(dollyBody, hookBody, Vector{}, Vector{}, 0, INFINITY)).Class.(*SlideJoint)
	winchServo.SetMaxForce(30000)
	winchServo.SetMaxBias(60)

	boxBody := space.AddBody(NewBody(30, MomentForBox(30, 50, 0)))
	boxBody.SetPosition(Vector{200, -200})
	shape = space.AddShape(NewBox(boxBody,50, 50, 0))
	shape.SetFriction(0.7)
	shape.SetCollisionType(COLLISION_CRATE)

	handler := space.NewCollisionHandler(COLLISION_HOOK, COLLISION_CRATE)
	handler.BeginFunc = hookCrate

	examples.Main(space, 1.0/60.0, update, examples.DefaultDraw)
}

func update(space *Space, dt float64) {
	// Set the first anchor point (the one attached to the static body) of the dolly servo to the mouse's x position.
	dollyServo.AnchorA = Vector{examples.Mouse.X, 100}

	// Set the max length of the winch servo to match the mouse's height.
	winchServo.Max = math.Max(100.0-examples.Mouse.Y, 50)

	if hookJoint != nil && examples.RightClick {
		space.RemoveConstraint(hookJoint)
		hookJoint = nil
	}

	examples.DrawString(Vector{-200, -200}, "Control the crane by moving the mouse. Right click to release.")
	space.Step(dt)
}

const (
	COLLISION_HOOK  = 1
	COLLISION_CRATE = 2
)

func attachHook(space *Space, b1, b2 interface{}) {
	hook := b1.(*Body)
	crate := b2.(*Body)
	hookJoint = space.AddConstraint(NewPivotJoint(hook, crate, hook.Position()))
}

func hookCrate(arb *Arbiter, space *Space, data interface{}) bool {
	if hookJoint == nil {
		// Get pointers to the two bodies in the collision pair and define local variables for them.
		// Their order matches the order of the collision types passed
		// to the collision handler this function was defined for
		hook, crate := arb.Bodies()

		// additions and removals can't be done in a normal callback.
		// Schedule a post step callback to do it.
		// Use the hook as the key and pass along the arbiter.
		space.AddPostStepCallback(attachHook, hook, crate)
	}

	return true
}
