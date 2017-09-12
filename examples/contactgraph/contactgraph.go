package main

import (
	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

var scaleStaticBody, ballBody *Body

func main() {
	space := NewSpace()
	space.Iterations = 30
	space.SetGravity(Vector{0, -300})
	space.SetCollisionSlop(0.5)
	space.SleepTimeThreshold = 1

	var body *Body
	var shape *Shape

	walls := []Vector{
		{-320, -240}, {-320, 240},
		{320, -240}, {320, 240},
		{-320, -240}, {320, -240},
	}

	for i := 0; i < len(walls)-1; i += 2 {
		shape = space.AddShape(NewSegment(space.StaticBody, walls[i], walls[i+1], 0))
		shape.SetElasticity(1)
		shape.SetFriction(1)
		shape.SetFilter(examples.NotGrabbableFilter)
	}

	scaleStaticBody = NewStaticBody()
	shape = space.AddShape(NewSegment(scaleStaticBody, Vector{-240, -180}, Vector{-140, -180}, 4))
	shape.SetElasticity(1)
	shape.SetFriction(1)
	shape.SetFilter(examples.NotGrabbableFilter)

	for i := 0; i < 5; i++ {
		body = space.AddBody(NewBody(1, MomentForBox(1, 30, 30)))
		body.SetPosition(Vector{0, float64(i*32 - 220)})

		shape = space.AddShape(NewBox(body, 30, 30, 0))
		shape.SetElasticity(0)
		shape.SetFriction(0.8)
	}

	radius := 15.0
	ballBody = space.AddBody(NewBody(10, MomentForCircle(10, 0, radius, VectorZero())))
	ballBody.SetPosition(Vector{120, -240 + radius + 5})

	shape = space.AddShape(NewCircle(ballBody, radius, VectorZero()))
	shape.SetElasticity(0)
	shape.SetFriction(0.9)

	examples.Main(space, 640, 480, 1.0/60.0, update)
}

func update(space *Space, dt float64) {
	space.Step(dt)

	examples.DrawString(VectorZero(), "hi")
}
