package main

import (
	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

const (
	CHAIN_COUNT = 8
	LINK_COUNT  = 10
)

func BreakableJointPostStepRemove(space *Space, joint interface{}, _ interface{}) {
	space.RemoveConstraint(joint.(*Constraint))
}

func BreakableJointPostSolve(joint *Constraint, space *Space) {
	dt := space.TimeStep()

	// Convert the impulse to a force by dividing it by the timestep.
	force := joint.Class.GetImpulse() / dt
	maxForce := joint.MaxForce()

	// If the force is almost as big as the joint's max force, break it.
	if force > 0.9*maxForce {
		space.AddPostStepCallback(BreakableJointPostStepRemove, joint, nil)
	}
}

func main() {
	space := NewSpace()
	space.Iterations = 30
	space.SetGravity(Vector{0, -100})
	space.SleepTimeThreshold = 0.5

	walls := []Vector{
		{-320, -240}, {-320, 240},
		{320, -240}, {320, 240},
		{-320, -240}, {320, -240},
		{-320, 240}, {320, 240},
	}
	for i := 0; i < len(walls)-1; i += 2 {
		shape := space.AddShape(NewSegment(space.StaticBody, walls[i], walls[i+1], 0))
		shape.SetElasticity(1)
		shape.SetFriction(1)
		shape.SetFilter(examples.NotGrabbableFilter)
	}

	mass := 1.0
	width := 20.0
	height := 30.0

	spacing := width * 0.3

	var i, j float64
	for i = 0; i < CHAIN_COUNT; i++ {
		var prev *Body

		for j = 0; j < LINK_COUNT; j++ {
			pos := Vector{40 * (i - (CHAIN_COUNT-1)/2.0), 240 - (j+0.5)*height - (j+1)*spacing}

			body := space.AddBody(NewBody(mass, MomentForBox(mass, width, height)))
			body.SetPosition(pos)

			shape := space.AddShape(NewSegment(body, Vector{0, (height - width) / 2}, Vector{0, (width - height) / 2}, width/2))
			shape.SetFriction(0.8)

			breakingForce := 80000.0

			var constraint *Constraint
			if prev == nil {
				constraint = space.AddConstraint(NewSlideJoint(body, space.StaticBody, Vector{0, height / 2}, Vector{pos.X, 240}, 0, spacing))
			} else {
				constraint = space.AddConstraint(NewSlideJoint(body, prev, Vector{0, height / 2}, Vector{0, -height / 2}, 0, spacing))
			}

			constraint.SetMaxForce(breakingForce)
			constraint.PostSolve = BreakableJointPostSolve
			constraint.SetCollideBodies(false)

			prev = body
		}
	}

	radius := 15.0
	body := space.AddBody(NewBody(10, MomentForCircle(10, 0, radius, VectorZero())))
	body.SetPosition(Vector{0, -240 + radius + 5})
	body.SetVelocity(0, 300)

	shape := space.AddShape(NewCircle(body, radius, VectorZero()))
	shape.SetElasticity(0)
	shape.SetFriction(0.9)

	examples.Main(space, 640, 480, 1.0/180.0, update)
}

func update(space *Space, dt float64) {
	space.Step(dt)
}
