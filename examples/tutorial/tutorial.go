package main

import (
	. "github.com/jakecoffman/physics"
	"fmt"
)

func main() {
	gravity := &Vector{0, -100}

	// Create an empty space.
	space := NewSpace()
	space.SetGravity(gravity)

	// Add a static line segment shape for the ground.
	// We'll make it slightly tilted so the ball will roll off.
	// We attach it to space->staticBody to tell Chipmunk it shouldn't be movable.
	ground := NewSegment(space.StaticBody(), &Vector{-20, 5}, &Vector{20, -5}, 0)
	ground.U = 1
	space.AddShape(ground)

	// Now let's make a ball that falls onto the line and rolls off.
	// First we need to make a cpBody to hold the physical properties of the object.
	// These include the mass, position, velocity, angle, etc. of the object.
	// Then we attach collision shapes to the cpBody to give it a size and shape.
	var radius float64 = 5
	var mass float64 = 1

	// The moment of inertia is like mass for rotation
	// Use the cpMomentFor*() functions to help you approximate it.
	moment := MomentForCircle(mass, 0, radius, VectorZero())

	// The cpSpaceAdd*() functions return the thing that you are adding.
	// It's convenient to create and add an object in one line.
	ballBody := space.AddBody(NewBody(mass, moment))
	ballBody.SetPosition(&Vector{0, 15})

	// Now we create the collision shape for the ball.
	// You can create multiple collision shapes that point to the same body.
	// They will all be attached to the body and move around to follow it.
	ballShape := space.AddShape(NewCircle(ballBody, radius, VectorZero()))
	ballShape.U = 0.7

	// Now that it's all set up, we simulate all the objects in the space by
	// stepping forward through time in small increments called steps.
	// It is *highly* recommended to use a fixed size time step.
	var timeStep float64 = 1.0/60.0
	var time float64
	var i uint
	for time = 0; time < 2; time += timeStep {
		pos := ballBody.Position()
		vel := ballBody.Velocity()
		fmt.Printf(
			"%d Time is %5.2f. ballBody is at (%5.2f, %5.2f). It's velocity is (%5.2f, %5.2f)\n",
			i, time, pos.X, pos.Y, vel.X, vel.Y)
		space.Step(timeStep)
		i++
	}
}
