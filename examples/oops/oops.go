package main

import (
	"log"

	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

func main() {
	log.SetFlags(log.LstdFlags | log.Lshortfile)
	space := NewSpace()
	space.SleepTimeThreshold = 0.5

	for i := 0; i < 2; i++ {
		addBox(space, 10, 1, float64(i), float64(i))
	}

	examples.Main(space, 200, 200, 0.01666, func(space *Space, dt float64) {
		space.Step(dt)
	})
}

func addBox(space *Space, size, mass, x, y float64) *Body {
	radius := (Vector{size, size}).Length()
	body := space.AddBody(NewBody(mass, MomentForBox(mass, size, size)))
	body.SetPosition(Vector{x, y})

	shape := NewBox(body, size, size, radius)
	space.AddShape(shape)
	shape.E = 0
	shape.U = 0.7
	return body
}
