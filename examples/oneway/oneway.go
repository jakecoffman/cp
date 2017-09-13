package main

import (
	"log"

	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

const (
	COLLISION_TYPE_ONE_WAY = 1
)

type oneWayPlatform struct {
	n Vector
}

var platformInstance oneWayPlatform

func PreSolve(arb *Arbiter, space *Space, _ interface{}) bool {
	log.Println("HERE")
	a, _ := arb.Shapes()
	platform := a.UserData.(*oneWayPlatform)

	if arb.Normal().Dot(platform.n) < 0 {
		return arb.Ignore()
	}

	return true
}

func main() {
	space := NewSpace()
	space.Iterations = 10
	space.SetGravity(Vector{0, -100})

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

	shape = space.AddShape(NewSegment(space.StaticBody, Vector{-160, -100}, Vector{160, -100}, 10))
	shape.SetElasticity(1)
	shape.SetFriction(1)
	shape.SetFilter(examples.NotGrabbableFilter)

	platformInstance.n = Vector{0, 1}
	shape.UserData = &platformInstance

	radius := 15.0
	body = space.AddBody(NewBody(1, MomentForCircle(10, 0, radius, VectorZero())))
	body.SetPosition(Vector{0, -200})
	body.SetVelocity(0, 170)

	shape = space.AddShape(NewCircle(body, radius, VectorZero()))
	shape.SetElasticity(0)
	shape.SetFriction(0.9)
	shape.SetCollisionType(2)

	handler := space.NewWildcardCollisionHandler(COLLISION_TYPE_ONE_WAY)
	handler.PreSolveFunc = PreSolve

	examples.Main(space, 640, 480, 1.0/60.0, update)
}

func update(space *Space, dt float64) {
	space.Step(dt)
}
