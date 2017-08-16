package main

import (
	. "github.com/jakecoffman/physics"
	"fmt"
)

func main() {
	gravity := &Vector{0, -100}

	space := NewSpace()
	space.SetGravity(gravity)

	ground := NewSegment(space.StaticBody(), &Vector{-20, 5}, &Vector{20, -2}, 0)
	ground.U = 1
	space.AddShape(ground)

	var radius float64 = 5
	var mass float64 = 1

	moment := MomentForCircle(mass, 0, radius, VectorZero())

	ballBody := space.AddBody(NewBody(mass, moment))
	ballBody.SetPosition(&Vector{0, 15})

	ballShape := space.AddShape(NewCircle(ballBody, radius, VectorZero()))
	ballShape.U = 0.7

	var timeStep float64 = 1.0/60.0
	var time float64
	for time = 0; time < 2; time += timeStep {
		pos := ballBody.Position()
		vel := ballBody.Velocity()
		fmt.Printf(
			"Time is %5.2f. ballBody is at (%5.2f, %5.2f). It's velocity is (%5.2f, %5.2f)\n",
			time, pos.X, pos.Y, vel.X, vel.Y)
		space.Step(timeStep)
	}
}
