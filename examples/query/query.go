package main

import (
	"fmt"
	"math"

	. "github.com/jakecoffman/cp"
	"github.com/jakecoffman/cp/examples"
)

var queryStart Vector

func main() {
	space := NewSpace()

	queryStart = Vector{}
	space.Iterations = 5

	// fat segment
	{
		mass := 1.0
		length := 100.0
		a := Vector{-length / 2.0, 0}
		b := Vector{length / 2.0, 0}

		body := space.AddBody(NewBody(mass, MomentForSegment(mass, a, b, 0)))
		body.SetPosition(Vector{0, 100})

		space.AddShape(NewSegment(body, a, b, 20))
	}

	// static segment
	{
		space.AddShape(NewSegment(space.StaticBody, Vector{0, 300}, Vector{300, 0}, 0))
	}

	// pentagon
	{
		mass := 1.0
		const numVerts = 5

		verts := []Vector{}
		for i := 0; i < numVerts; i++ {
			angle := -2.0 * math.Pi * float64(i) / numVerts
			verts = append(verts, Vector{30 * math.Cos(angle), 30 * math.Sin(angle)})
		}

		body := space.AddBody(NewBody(mass, MomentForPoly(mass, len(verts), verts, Vector{}, 0)))
		body.SetPosition(Vector{50, 30})

		space.AddShape(NewPolyShape(body, numVerts, verts, NewTransformIdentity(), 10))
	}

	// circle
	{
		mass := 1.0
		r := 20.0

		body := space.AddBody(NewBody(mass, MomentForCircle(mass, 0, r, Vector{})))
		body.SetPosition(Vector{100, 100})

		space.AddShape(NewCircle(body, r, Vector{}))
	}

	examples.Main(space, 1.0/60.0, update, examples.DefaultDraw)
}

func update(space *Space, dt float64) {
	space.Step(dt)

	if examples.RightClick {
		queryStart = examples.Mouse
	}

	start := queryStart
	end := examples.Mouse
	radius := 10.0
	examples.DrawSegment(start, end, FColor{0, 1, 0, 0})

	str := `Query: Dist(%f) Point(%5.2f, %5.2f), `
	segInfo := space.SegmentQueryFirst(start, end, radius, SHAPE_FILTER_ALL)
	if segInfo.Shape != nil {
		point := segInfo.Point
		n := segInfo.Normal

		examples.DrawSegment(start.Lerp(end, segInfo.Alpha), end, FColor{0, 0, 1, 1})
		examples.DrawSegment(point, point.Add(n.Mult(16)), FColor{1, 0, 0, 1})
		examples.DrawDot(3, point, FColor{1, 0, 0, 1})
		str += fmt.Sprintf("Segment Query: Dist(%f) Normal(%5.2f, %5.2f)",
			segInfo.Alpha*start.Distance(end), n.X, n.Y)
	} else {
		str += "Segment Query (None)"
	}

	// Draw a fat green line over the unoccluded part of the query
	examples.DrawFatSegment(start, start.Lerp(end, segInfo.Alpha), radius, FColor{0, 1, 0, 1}, FColor{})
	nearestInfo := space.PointQueryNearest(examples.Mouse, 100, SHAPE_FILTER_ALL)
	if nearestInfo.Shape != nil {
		examples.DrawDot(3, examples.Mouse, FColor{0.5, 0.5, 0.5, 1})
		examples.DrawSegment(examples.Mouse, nearestInfo.Point, FColor{0.5, 0.5, 0.5, 1})
		if nearestInfo.Distance < 0 {
			examples.DrawBB(nearestInfo.Shape.BB(), FColor{1, 0, 0, 1})
		}
	}

	examples.DrawString(Vector{-300, -200}, fmt.Sprintf(str, start.Distance(end), end.X, end.Y))
}
