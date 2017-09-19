package main

import (
	. "github.com/jakecoffman/cp"
	"github.com/jakecoffman/cp/examples"
)

const (
	width  = 640
	height = 480

	hwidth  = width / 2
	hheight = height / 2
)

func main() {
	space := NewSpace()
	space.Iterations = 30
	space.SetGravity(Vector{0, -100})
	space.SleepTimeThreshold = 0.5
	space.SetCollisionSlop(0.5)

	sides := []Vector{
		{-hwidth, -hheight}, {-hwidth, hheight},
		{hwidth, -hheight}, {hwidth, hheight},
		{-hwidth, -hheight}, {hwidth, -hheight},
	}

	for i := 0; i < len(sides); i += 2 {
		var seg *Shape
		seg = space.AddShape(NewSegment(space.StaticBody, sides[i], sides[i+1], 0))
		seg.SetElasticity(1)
		seg.SetFriction(1)
		seg.SetFilter(examples.NotGrabbableFilter)
	}

	var body *Body
	var shape *Shape

	for i := 0; i < 14; i++ {
		for j := 0; j <= i; j++ {
			body = space.AddBody(NewBody(1.0, MomentForBox(1, 30, 30)))
			body.SetPosition(Vector{float64(j*32 - i*16), float64(300 - i*32)})

			shape = space.AddShape(NewBox(body, 30, 30, 0.5))
			shape.SetElasticity(0)
			shape.SetFriction(0.8)
		}
	}

	radius := 15.0
	body = space.AddBody(NewBody(10, MomentForCircle(10, 0, radius, Vector{})))
	body.SetPosition(Vector{0, -hheight + radius + 5})

	shape = space.AddShape(NewCircle(body, radius, Vector{}))
	shape.SetElasticity(0)
	shape.SetFriction(0.9)

	examples.Main(space,1.0/180.0, update, examples.DefaultDraw)
}

func update(space *Space, dt float64) {
	space.Step(dt)
}
