package main

import (
	"math/rand"

	. "github.com/jakecoffman/physics"
)

func main() {
	var space *Space = NewSpace()
	space.Iterations = 10
	space.SleepTimeThreshold = 0.5

	seg1 := NewSegment(space.Body, &Vector{-320, 420}, &Vector{-320, 240}, 0)
	space.AddShape(seg1)
	seg1.Shape.Body().Activate()
	seg1.Shape.E = 1
	seg1.Shape.U = 1

	for i := 0; i < 50; i++ {

	}
}

func addBox(space *Space, size, mass float64) *Body {
	radius := (&Vector{size, size}).Length()
	body := space.AddBody(NewBody(mass, MomentForBox(mass, size, size)))
	body.SetPosition(&Vector{rand.Float64()*(640-2*radius) - (320 - radius), rand.Float64()*(480-2*radius) - (240 - radius)})

	shape := NewBox(body, size, size, 0)
	space.AddShape(shape)
	shape.E = 0
	shape.U = 0.7
	return body
}
