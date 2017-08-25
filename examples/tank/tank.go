package main

import (
	"log"
	"math/rand"
	"time"

	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

func main() {
	log.SetFlags(log.LstdFlags | log.Lshortfile)
	rand.Seed(time.Now().Unix())

	space := NewSpace()
	space.Iterations = 10
	space.SleepTimeThreshold = 0.5

	var seg1 *Shape = NewSegment(space.Body, &Vector{-320, 420}, &Vector{-320, 240}, 0)
	space.AddShape(seg1)
	seg1.Body().Activate()
	seg1.E = 1
	seg1.U = 1

	for i := 0; i < 50; i++ {
		addBox(space, 10, 1)
		//pivot := NewPivotJoint2()
		//space.AddConstraint()
	}

	//tankControlBody := space.AddBody(NewKinematic())
	//tankBody := addBox(space, 30, 10)

	examples.Main(space, 600, 600, 0.1666)
}

func addBox(space *Space, size, mass float64) *Body {
	radius := (&Vector{size, size}).Length()
	body := space.AddBody(NewBody(mass, MomentForBox(mass, size, size)))
	body.SetPosition(&Vector{rand.Float64()*(600-2*radius) - (320 - radius), rand.Float64()*(600-2*radius) - (240 - radius)})

	shape := NewBox(body, size, size, 0)
	space.AddShape(shape)
	shape.E = 0
	shape.U = 0.7
	return body
}
