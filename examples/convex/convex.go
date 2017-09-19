package main

import (
	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

const DENSITY = 1.0 / 10000.0

var shape *PolyShape

func main() {
	space := NewSpace()
	space.Iterations = 30
	space.SetGravity(Vector{0, -500})
	space.SleepTimeThreshold = 0.5
	space.SetCollisionSlop(0.5)

	wall := space.AddShape(NewSegment(space.StaticBody, Vector{-320, -240}, Vector{320, -240}, 0))
	wall.SetElasticity(1)
	wall.SetFriction(1)
	wall.SetFilter(examples.NotGrabbableFilter)

	width := 50.0
	height := 70.0
	mass := width*height*DENSITY
	moment := MomentForBox(mass, width, height)

	body := space.AddBody(NewBody(mass, moment))
	shape = space.AddShape(NewBox(body, width, height, 0)).Class.(*PolyShape)
	shape.SetFriction(0.6)

	examples.Main(space, 1.0/60.0, update, examples.DefaultDraw)
}

func update(space *Space, dt float64) {
	tolerance := 2.0

	if examples.RightClick && shape.Shape.PointQuery(examples.Mouse).Distance > tolerance {
		body := shape.Body()
		count := shape.Count()

		verts := make([]Vector, count+1)

		for i := 0; i < count; i++ {
			verts[i] = shape.Vert(i)
		}

		verts[count] = body.WorldToLocal(examples.Mouse)

		hullCount := ConvexHull(count+1, verts, nil, tolerance)

		centroid := CentroidForPoly(hullCount, verts)

		mass := AreaForPoly(hullCount, verts, 0) * DENSITY
		body.SetMass(mass)
		body.SetMoment(MomentForPoly(mass, hullCount, verts, centroid.Neg(), 0))
		body.SetPosition(body.LocalToWorld(centroid))

		shape.SetVertsUnsafe(hullCount, verts, NewTransformTranslate(centroid.Neg()))
	}

	space.Step(dt)
}
