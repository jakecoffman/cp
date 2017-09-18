package main

import (
	"math"
	"math/rand"

	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

const gravityStrength = 5.0e6

var planetBody *Body

func main() {
	space := NewSpace()
	space.Iterations = 20

	planetBody = space.AddBody(NewKinematicBody())
	planetBody.SetAngularVelocity(0.2)

	for i := 0; i < 30; i++ {
		addBox(space)
	}

	shape := space.AddShape(NewCircle(planetBody, 70, VectorZero()))
	shape.SetElasticity(1)
	shape.SetFriction(1)
	shape.SetFilter(examples.NotGrabbableFilter)

	examples.Main(space, 640, 480, 1.0/60.0, update)
}

func planetGravityVelocity(body *Body, gravity Vector, damping, dt float64) {
	p := body.Position()
	sqdist := p.LengthSq()
	g := p.Mult(-gravityStrength / (sqdist * math.Sqrt(sqdist)))
	body.UpdateVelocity(g, damping, dt)
}

func randPos(radius float64) Vector {
	var v Vector
	for {
		v = Vector{rand.Float64()*(640-2*radius) - (320 - radius), rand.Float64()*(480-2*radius) - (240 - radius)}
		if v.Length() >= 85 {
			return v
		}
	}
}

func addBox(space *Space) {
	size := 10.0
	mass := 1.0

	verts := []Vector{
		{-size, -size},
		{-size, size},
		{size, size},
		{size, -size},
	}

	radius := Vector{size, size}.Length()
	pos := randPos(radius)

	body := space.AddBody(NewBody(mass, MomentForPoly(mass, len(verts), verts, VectorZero(), 0)))
	body.SetVelocityUpdateFunc(planetGravityVelocity)
	body.SetPosition(pos)

	r := pos.Length()
	v := math.Sqrt(gravityStrength/r) / r
	body.SetVelocityVector(pos.Perp().Mult(v))

	body.SetAngularVelocity(v)
	body.SetAngle(math.Atan2(pos.Y, pos.X))

	shape := space.AddShape(NewPolyShape(body, 4, verts, NewTransformIdentity(), 0))
	shape.SetElasticity(0)
	shape.SetFriction(0.7)
}

func update(space *Space, dt float64) {
	space.Step(dt)
}
