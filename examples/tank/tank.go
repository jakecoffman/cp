package main

import "github.com/jakecoffman/physics"

func main() {
	var space *physics.Space = physics.NewSpace()
	space.Iterations = 10
	space.SleepTimeThreshold = 0.5

	seg1 := physics.NewSegment(space.Body, &physics.Vector{-320, 420}, &physics.Vector{-320, 240}, 0)
	space.AddShape(seg1)
	seg1.Shape.Body().Activate()
	seg1.Shape.E = 1
	seg1.Shape.U = 1

}
