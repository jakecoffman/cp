package main

import (
	"math"

	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

const DENSITY = 1.0 / 10000.0

func ClipPoly(space *Space, shape *Shape, n Vector, dist float64) {
	body := shape.Body()

	poly := shape.Class.(*PolyShape)
	count := poly.Count()
	var clippedCount int

	clipped := make([]Vector, count+1)

	j := count - 1
	for i := 0; i < count; i++ {
		a := body.LocalToWorld(poly.Vert(j))
		aDist := a.Dot(n) - dist

		if aDist < 0 {
			clipped[clippedCount] = a
			clippedCount++
		}

		b := body.LocalToWorld(poly.Vert(i))
		bDist := b.Dot(n) - dist

		if aDist*bDist < 0 {
			t := math.Abs(aDist) / (math.Abs(aDist) + math.Abs(bDist))

			clipped[clippedCount] = a.Lerp(b, t)
			clippedCount++
		}
		j = i
	}

	centroid := CentroidForPoly(clippedCount, clipped)
	mass := AreaForPoly(clippedCount, clipped, 0) * DENSITY
	moment := MomentForPoly(mass, clippedCount, clipped, centroid.Neg(), 0)

	newBody := space.AddBody(NewBody(mass, moment))
	newBody.SetPosition(centroid)
	newBody.SetVelocityVector(body.VelocityAtWorldPoint(centroid))
	newBody.SetAngularVelocity(body.AngularVelocity())

	transform := NewTransformTranslate(centroid.Neg())
	newShape := space.AddShape(NewPolyShape(newBody, clippedCount, clipped, transform, 0))
	newShape.SetFriction(shape.Friction())
}

// Context structs are annoying, use blocks or closures instead if your compiler supports them.
type SliceContext struct {
	a, b  Vector
	space *Space
}

func SliceShapePostStep(space *Space, ptr, obj interface{}) {
	shape := ptr.(*Shape)
	context := obj.(*SliceContext)
	a := context.a
	b := context.b

	// Clipping plane normal and distance.
	n := b.Sub(a).Perp().Normalize()
	dist := a.Dot(n)

	ClipPoly(space, shape, n, dist)
	ClipPoly(space, shape, n.Neg(), -dist)

	body := shape.Body()
	space.RemoveShape(shape)
	space.RemoveBody(body)
}

func SliceQuery(shape *Shape, point, normal Vector, alpha float64, data interface{}) {
	context := data.(*SliceContext)
	a := context.a
	b := context.b

	// Check that the slice was complete by checking that the endpoints aren't in the sliced shape.
	if shape.PointQuery(a).Distance > 0 && shape.PointQuery(b).Distance > 0 {
		// Can't modify the space during a query.
		// Must make a post-step callback to do the actual slicing.
		context.space.AddPostStepCallback(SliceShapePostStep, shape, context)
	}
}

func main() {
	space := NewSpace()
	space.Iterations = 30
	space.SetGravity(Vector{0, -500})
	space.SleepTimeThreshold = 0.5
	space.SetCollisionSlop(0.5)

	shape := space.AddShape(NewSegment(space.StaticBody, Vector{-1000, -240}, Vector{1000, -240}, 0))
	shape.SetElasticity(1)
	shape.SetFriction(1)
	shape.SetFilter(examples.NotGrabbableFilter)

	width := 200.0
	height := 300.0
	mass := width * height * DENSITY
	moment := MomentForBox(mass, width, height)

	body := space.AddBody(NewBody(mass, moment))
	shape = space.AddShape(NewBox(body, width, height, 0))
	shape.SetFriction(0.6)

	examples.Main(space, 1.0/60.0, update, examples.DefaultDraw)
}

var lastClickState bool
var sliceStart Vector

func update(space *Space, dt float64) {
	space.Step(dt)

	// Annoying state tracking code that you wouldn't need
	// in a real event driven system.
	if examples.RightClick != lastClickState {
		if examples.RightClick {
			sliceStart = examples.Mouse
		} else {
			context := SliceContext{sliceStart, examples.Mouse, space}
			space.SegmentQuery(sliceStart, examples.Mouse, 0, examples.GrabFilter, SliceQuery, &context)
		}
		lastClickState = examples.RightClick
	}

	if examples.RightClick {
		examples.DrawSegment(sliceStart, examples.Mouse, FColor{1, 0, 0, 1})
	}
}
