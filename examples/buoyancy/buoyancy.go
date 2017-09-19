package main

import (
	"math"

	. "github.com/jakecoffman/cp"
	"github.com/jakecoffman/cp/examples"
)

const (
	FLUID_DENSITY = 0.00014
	FLUID_DRAG    = 2.0
)

func kScalarBody(body *Body, point, n Vector) float64 {
	rcn := point.Sub(body.Position()).Cross(n)
	return 1.0/body.Mass() + rcn*rcn/body.Moment()
}

func waterPreSolve(arb *Arbiter, space *Space, ptr interface{}) bool {
	water, polyShape := arb.Shapes()
	poly := polyShape.Class.(*PolyShape)
	body := poly.Body()

	// Get the top of the water sensor bounding box to use as the water level.
	level := water.BB().T

	// Clip the polygon against the water level
	count := poly.Count()
	var clippedCount int
	clipped := make([]Vector, count + 1)

	j := count - 1
	for i := 0; i < count; i++ {
		a := body.LocalToWorld(poly.Vert(j))
		b := body.LocalToWorld(poly.Vert(i))

		if a.Y < level {
			clipped[clippedCount] = a
			clippedCount++
		}

		aLevel := a.Y - level
		bLevel := b.Y - level

		if aLevel*bLevel < 0 {
			t := math.Abs(aLevel) / (math.Abs(aLevel) + math.Abs(bLevel))
			clipped[clippedCount] = a.Lerp(b, t)
			clippedCount++
		}
		j = i
	}

	// Calculate buoyancy from the clipped polygon area
	clippedArea := AreaForPoly(clippedCount, clipped, 0)
	displacedMass := clippedArea * FLUID_DENSITY
	centroid := CentroidForPoly(clippedCount, clipped)

	examples.DrawPolygon(clippedCount, clipped, 0, FColor{0, 0, 1, 1}, FColor{0, 0, 1, 0.1})
	examples.DrawDot(5, centroid, FColor{0, 0, 1, 1})

	dt := space.TimeStep()
	g := space.Gravity()

	// Apply the buoyancy force as an impulse.
	body.ApplyImpulseAtWorldPoint(g.Mult(-displacedMass*dt), centroid)

	// Apply linear damping for the fluid drag.
	vCentroid := body.VelocityAtWorldPoint(centroid)
	k := kScalarBody(body, centroid, vCentroid.Normalize())
	damping := clippedArea * FLUID_DRAG * FLUID_DENSITY
	vCoef := math.Exp(-damping * dt * k) // linear drag
	body.ApplyImpulseAtWorldPoint(vCentroid.Mult(vCoef).Sub(vCentroid).Mult(1.0/k), centroid)

	// Apply angular damping for the fluid drag.
	cog := body.LocalToWorld(body.CenterOfGravity())
	wDamping := MomentForPoly(FLUID_DENSITY*FLUID_DRAG*clippedArea, clippedCount, clipped, cog.Neg(), 0)
	body.SetAngularVelocity(body.AngularVelocity() * math.Exp(-wDamping*dt/body.Moment()))

	return true
}

func main() {
	space := NewSpace()
	space.Iterations = 30
	space.SetGravity(Vector{0, -500})
	space.SleepTimeThreshold = 0.5
	space.SetCollisionSlop(0.5)

	walls := []Vector{
		{-320, -240}, {-320, 240},
		{320, -240}, {320, 240},
		{-320, -240}, {320, -240},
		{-320, 240}, {320, 240},
	}

	for i := 0; i < len(walls)-1; i += 2 {
		shape := space.AddShape(NewSegment(space.StaticBody, walls[i], walls[i+1], 0))
		shape.SetElasticity(1)
		shape.SetFriction(1)
		shape.SetFilter(examples.NotGrabbableFilter)
	}

	// add the edges of the bucket
	{
		bb := BB{-300, -200, 100, 0}
		radius := 5.0

		shape := space.AddShape(NewSegment(space.StaticBody, Vector{bb.L, bb.B}, Vector{bb.L, bb.T}, radius))
		shape.SetElasticity(1)
		shape.SetFriction(1)
		shape.SetFilter(examples.NotGrabbableFilter)

		shape = space.AddShape(NewSegment(space.StaticBody, Vector{bb.R, bb.B}, Vector{bb.R, bb.T}, radius))
		shape.SetElasticity(1)
		shape.SetFriction(1)
		shape.SetFilter(examples.NotGrabbableFilter)

		shape = space.AddShape(NewSegment(space.StaticBody, Vector{bb.L, bb.B}, Vector{bb.R, bb.B}, radius))
		shape.SetElasticity(1)
		shape.SetFriction(1)
		shape.SetFilter(examples.NotGrabbableFilter)

		// Add the sensor for the water.
		shape = space.AddShape(NewBox2(space.StaticBody, bb, 0))
		shape.SetSensor(true)
		shape.SetCollisionType(1)
	}

	{
		width := 200.0
		height := 50.0
		mass := 0.3 * FLUID_DENSITY * width * height
		moment := MomentForBox(mass, width, height)

		body := space.AddBody(NewBody(mass, moment))
		body.SetPosition(Vector{-50, -100})
		body.SetVelocity(0, -100)
		body.SetAngularVelocity(1)

		shape := space.AddShape(NewBox(body, width, height, 0))
		shape.SetFriction(0.8)
	}

	{
		width := 40.0
		height := width * 2
		mass := 0.3 * FLUID_DENSITY * width * height
		moment := MomentForBox(mass, width, height)

		body := space.AddBody(NewBody(mass, moment))
		body.SetPosition(Vector{-200, -50})
		body.SetVelocity(0, -100)
		body.SetAngularVelocity(1)

		shape := space.AddShape(NewBox(body, width, height, 0))
		shape.SetFriction(0.8)
	}

	handler := space.NewCollisionHandler(1, 0)
	handler.PreSolveFunc = waterPreSolve

	examples.Main(space, 1.0/180.0, update, examples.DefaultDraw)
}

func update(space *Space, dt float64) {
	space.Step(dt)
}
