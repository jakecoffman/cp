package main

import (
	"math"
	"math/rand"

	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

const (
	COLLISION_TYPE_STICKY = 1

	STICK_SENSOR_THICKNESS = 2.5
)

func PostStepAddJoint(space *Space, key, _ interface{}) {
	space.AddConstraint(key.(*Constraint))
}

func StickyPreSolve(arb *Arbiter, space *Space, data interface{}) bool {
	// We want to fudge the collisions a bit to allow shapes to overlap more.
	// This simulates their squishy sticky surface, and more importantly
	// keeps them from separating and destroying the joint.

	// Track the deepest collision point and use that to determine if a rigid collision should occur.
	deepest := INFINITY

	contacts := arb.ContactPointSet()
	for i := 0; i < contacts.Count; i++ {
		// Sink the contact points into the surface of each shape.
		contacts.Points[i].PointA = contacts.Points[i].PointA.Sub(contacts.Normal.Mult(STICK_SENSOR_THICKNESS))
		contacts.Points[i].PointB = contacts.Points[i].PointB.Sub(contacts.Normal.Mult(STICK_SENSOR_THICKNESS))
		deepest = math.Min(deepest, contacts.Points[i].Distance)
	}

	arb.SetContactPointSet(&contacts)

	// If the shapes are overlapping enough, then create a
	// joint that sticks them together at the first contact point.
	if arb.UserData == nil && deepest <= 0 {
		bodyA, bodyB := arb.Bodies()

		// Create a joint at the contact point to hold the body in place.
		anchorA := bodyA.WorldToLocal(contacts.Points[0].PointA)
		anchorB := bodyB.WorldToLocal(contacts.Points[0].PointB)
		joint := NewPivotJoint2(bodyA, bodyB, anchorA, anchorB)

		// Give it a finite force for the stickiness.
		joint.SetMaxForce(3e3)

		// Schedule a post-step() callback to add the joint.
		space.AddPostStepCallback(PostStepAddJoint, joint, nil)

		// Store the joint on the arbiter so we can remove it later.
		arb.UserData = joint
	}

	// Position correction and velocity are handled separately so changing
	// the overlap distance alone won't prevent the collision from occuring.
	// Explicitly the collision for this frame if the shapes don't overlap using the new distance.

	return deepest <= 0

	// Lots more that you could improve upon here as well:
	// * Modify the joint over time to make it plastic.
	// * Modify the joint in the post-step to make it conditionally plastic (like clay).
	// * Track a joint for the deepest contact point instead of the first.
	// * Track a joint for each contact point. (more complicated since you only get one data pointer).
}

func PostStepRemoveJoint(space *Space, key, _ interface{}) {
	space.RemoveConstraint(key.(*Constraint))
}

func StickySeparate(arb *Arbiter, space *Space, _ interface{}) {
	if arb.UserData != nil {
		joint := arb.UserData.(*Constraint)

		// The joint won't be removed until the step is done.
		// Need to disable it so that it won't apply itself.
		// Setting the force to 0 will do just that
		joint.SetMaxForce(0)

		// Perform the removal in a post-step() callback.
		space.AddPostStepCallback(PostStepRemoveJoint, joint, nil)

		// NULL out the reference to the joint.
		// Not required, but it's a good practice.
		arb.UserData = nil
	}
}

func main() {
	space := NewSpace()
	space.Iterations = 10
	space.SetGravity(Vector{0, -1000})
	space.SetCollisionSlop(2.0)

	walls := []Vector{
		{-340, -260}, {-340, 260},
		{340, -260}, {340, 260},
		{-340, -260}, {340, -260},
		{-340, 260}, {340, 260},
	}

	for i := 0; i < len(walls)-1; i += 2 {
		shape := space.AddShape(NewSegment(space.StaticBody, walls[i], walls[i+1], 20))
		shape.SetElasticity(1)
		shape.SetFriction(1)
		shape.SetFilter(examples.NotGrabbableFilter)
	}

	mass := 0.15
	radius := 10.0

	for i := 0; i < 200; i++ {
		body := space.AddBody(NewBody(mass, MomentForCircle(mass, 0, radius, Vector{})))
		body.SetPosition(Vector{Lerp(-150, 150, rand.Float64()), Lerp(-150, 150, rand.Float64())})

		shape := space.AddShape(NewCircle(body, radius+STICK_SENSOR_THICKNESS, Vector{}))
		shape.SetFriction(0.9)
		shape.SetCollisionType(COLLISION_TYPE_STICKY)
	}

	handler := space.NewWildcardCollisionHandler(COLLISION_TYPE_STICKY)
	handler.PreSolveFunc = StickyPreSolve
	handler.SeparateFunc = StickySeparate

	examples.Main(space, 640, 480, 1.0/60.0, update)
}

func update(space *Space, dt float64) {
	space.Step(dt)
}
