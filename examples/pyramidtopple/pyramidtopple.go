package main

import (
	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

const (
	width  = 4.0
	height = 30.0
)

func addDomino(space *Space, pos Vector, flipped bool) {
	mass := 1.0
	radius := 0.5
	moment := MomentForBox(mass, width, height)

	body := space.AddBody(NewBody(mass, moment))
	body.SetPosition(pos)

	var shape *Shape
	if flipped {
		shape = NewBox(body, height, width, 0)
	} else {
		shape = NewBox(body, width-radius*2, height, radius)
	}
	space.AddShape(shape)
	shape.SetElasticity(0)
	shape.SetFriction(0.6)
}

func main() {
	space := NewSpace()
	space.Iterations = 30
	space.SetGravity(Vector{0, -300})
	space.SleepTimeThreshold = 0.5
	space.SetCollisionSlop(0.5)

	shape := space.AddShape(NewSegment(space.StaticBody, Vector{-600, -240}, Vector{600, -240}, 0))
	shape.SetElasticity(1)
	shape.SetFriction(1)
	shape.SetFilter(examples.NotGrabbableFilter)

	n := 12.0
	var i, j float64
	for i = 0; i < n; i++ {
		for j = 0; j < (n - i); j++ {
			offset := Vector{(j - (n-1-i)*0.5) * 1.5 * height, (i+0.5)*(height+2*width) - width - 240}
			addDomino(space, offset, false)
			addDomino(space, offset.Add(Vector{0, (height + width) / 2}), true)

			if j == 0 {
				addDomino(space, offset.Add(Vector{0.5 * (width - height), height + width}), false)
			}

			if j != n-i-1 {
				addDomino(space, offset.Add(Vector{height * 0.75, (height + 3*width) / 2}), true)
			} else {
				addDomino(space, offset.Add(Vector{0.5 * (height - width), height + width}), false)
			}
		}
	}

	examples.Main(space, 640, 480, 1.0/180.0, update)
}

func update(space *Space, dt float64) {
	space.Step(dt)
}
