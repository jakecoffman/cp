package main

import (
	"math/rand"

	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

var KinematicBoxBox *Body

func main() {
	space := NewSpace()
	space.SetGravity(Vector{0, -600})

	// We create an infinite mass rogue body to attach the line segments to
	// This way we can control the rotation however we want.
	KinematicBoxBox = space.AddBody(NewKinematicBody())
	KinematicBoxBox.SetAngularVelocity(0.4)

	a := Vector{-200, -200}
	b := Vector{-200, 200}
	c := Vector{200, 200}
	d := Vector{200, -200}

	shape := space.AddShape(NewSegment(KinematicBoxBox, a, b, 0))
	shape.SetElasticity(1)
	shape.SetFriction(1)
	shape.SetFilter(examples.NotGrabbableFilter)

	shape = space.AddShape(NewSegment(KinematicBoxBox, b, c, 0))
	shape.SetElasticity(1)
	shape.SetFriction(1)
	shape.SetFilter(examples.NotGrabbableFilter)

	shape = space.AddShape(NewSegment(KinematicBoxBox, c, d, 0))
	shape.SetElasticity(1)
	shape.SetFriction(1)
	shape.SetFilter(examples.NotGrabbableFilter)

	shape = space.AddShape(NewSegment(KinematicBoxBox, d, a, 0))
	shape.SetElasticity(1)
	shape.SetFriction(1)
	shape.SetFilter(examples.NotGrabbableFilter)

	mass := 1.0
	width := 30.0
	height := width * 2

	for i := 0; i < 7; i++ {
		for j := 0; j < 3; j++ {
			pos := Vector{float64(i)*width - 150, float64(j)*height - 150}

			typ := rand.Intn(3)
			if typ == 0 {
				addBox(space, pos, mass, width, height)
			} else if typ == 1 {
				addSegment(space, pos, mass, width, height)
			} else {
				addCircle(space, pos.Add(Vector{0, (height - width) / 2}), mass, width/2)
				addCircle(space, pos.Add(Vector{0, (width - height) / 2}), mass, width/2)
			}
		}
	}

	examples.Main(space, 640, 480, 1.0/180.0, update)
}

func update(space *Space, dt float64) {
	space.Step(dt)
}

func addBox(space *Space, pos Vector, mass, width, height float64) {
	body := space.AddBody(NewBody(mass, MomentForBox(mass, width, height)))
	body.SetPosition(pos)

	shape := space.AddShape(NewBox(body, width, height, 0))
	shape.SetElasticity(0)
	shape.SetFriction(0.7)
}

func addSegment(space *Space, pos Vector, mass, width, height float64) {
	body := space.AddBody(NewBody(mass, MomentForBox(mass, width, height)))
	body.SetPosition(pos)

	shape := space.AddShape(NewSegment(body,
		Vector{0, (height - width) / 2.0},
		Vector{0, (width - height) / 2.0},
		width/2.0))
	shape.SetElasticity(0)
	shape.SetFriction(0.7)
}

func addCircle(space *Space, pos Vector, mass, radius float64) {
	body := space.AddBody(NewBody(mass, MomentForCircle(mass, 0, radius, VectorZero())))
	body.SetPosition(pos)

	shape := space.AddShape(NewCircle(body, radius, VectorZero()))
	shape.SetElasticity(0)
	shape.SetFriction(0.7)
}
