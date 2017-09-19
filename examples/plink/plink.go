package main

import (
	"math"

	"math/rand"

	. "github.com/jakecoffman/cp"
	"github.com/jakecoffman/cp/examples"
)

var (
	pentagonMass   = 0.0
	pentagonMoment = 0.0
)

const numVerts = 5

func main() {
	space := NewSpace()
	space.Iterations = 5
	space.SetGravity(Vector{0, -100})

	var body *Body
	var shape *Shape

	tris := []Vector{
		{-15, -15},
		{0, 10},
		{15, -15},
	}

	for i := 0; i < 9; i++ {
		for j := 0; j < 6; j++ {
			stagger := (j % 2) * 40
			offset := Vector{float64(i*80 - 320 + stagger), float64(j*70 - 240)}
			shape = space.AddShape(NewPolyShape(space.StaticBody, 3, tris, NewTransformTranslate(offset), 0))
			shape.SetElasticity(1)
			shape.SetFriction(1)
			shape.SetFilter(examples.NotGrabbableFilter)
		}
	}

	verts := []Vector{}
	for i := 0; i < numVerts; i++ {
		angle := -2.0 * math.Pi * float64(i) / numVerts
		verts = append(verts, Vector{10 * math.Cos(angle), 10 * math.Sin(angle)})
	}

	pentagonMass = 1.0
	pentagonMoment = MomentForPoly(1, numVerts, verts, Vector{}, 0)

	for i := 0; i < 300; i++ {
		body = space.AddBody(NewBody(pentagonMass, pentagonMoment))
		x := rand.Float64()*640 - 320
		body.SetPosition(Vector{x, 350})

		shape = space.AddShape(NewPolyShape(body, numVerts, verts, NewTransformIdentity(), 0))
		shape.SetElasticity(0)
		shape.SetFriction(0.4)
	}

	examples.Main(space, 1.0/60.0, update, examples.DefaultDraw)
}

func update(space *Space, dt float64) {
	if examples.RightDown {
		nearest := space.PointQueryNearest(examples.Mouse, 0, examples.GrabFilter)
		if nearest.Shape != nil {
			body := nearest.Shape.Body()
			if body.GetType() == BODY_STATIC {
				body.SetType(BODY_DYNAMIC)
				body.SetMass(pentagonMass)
				body.SetMoment(pentagonMoment)
			} else if body.GetType() == BODY_DYNAMIC {
				body.SetType(BODY_STATIC)
			}
		}
	}

	space.EachBody(func(body *Body) {
		pos := body.Position()
		if pos.Y < -260 || math.Abs(pos.X) > 340 {
			x := rand.Float64()*640 - 320
			body.SetPosition(Vector{x, 260})
		}
	})
	space.Step(dt)
}
