package main

import (
	. "github.com/jakecoffman/cp"
	"github.com/jakecoffman/cp/examples"
	"math/rand"
	"fmt"
)

var (
	queryStart Vector
	width      = 640.0
	height     = 480.0

	hwidth  = width / 2
	hheight = height / 2

	dragging bool
)

func main() {
	space := NewSpace()

	for i := 0; i < 50; i++ {
		addBox(space, 20, 1)
	}

	examples.Main(space, 1.0/60.0, update, func(space *Space) {
		examples.DefaultDraw(space)

		bb := NewBBForCircle(queryStart, examples.Mouse.Sub(queryStart).Length())
		examples.DrawBB(bb, FColor{0, 1, 0, 1})

		str := "Query: Center(%f, %f) Count(%d)"
		var count uint64 = 0
		space.BBQuery(bb, SHAPE_FILTER_ALL, func(shape *Shape, data interface{}) {
			count++
			examples.DrawBB(shape.BB(), FColor{1, 0, 0, 1})
		}, nil)

		examples.DrawString(Vector{-300, -150}, fmt.Sprintf(str, bb.Center().X, bb.Center().Y, count))
	})
}

func addBox(space *Space, size, mass float64) *Body {
	radius := (&Vector{size, size}).Length()
	body := space.AddBody(NewBody(mass, MomentForBox(mass, size, size)))
	body.SetPosition(Vector{rand.Float64()*(width-2*radius) - (hwidth - radius), rand.Float64()*(height-2*radius) - (hheight - radius)})

	space.AddShape(NewBox(body, size, size, 0))
	return body
}

func update(space *Space, dt float64) {
	space.Step(dt)

	if !examples.RightClick {
		dragging = false
		examples.DrawString(Vector{-300, -200}, "Right click and drag to select boxes")
		return
	}

	if !dragging {
		queryStart = examples.Mouse
	}

	dragging = true
}
