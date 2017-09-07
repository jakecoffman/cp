package main

import (
	"math/rand"

	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

const (
	bevel = 1
)

func randUnitCircle() Vector {
	v := Vector{rand.Float64()*2.0 - 1.0, rand.Float64()*2.0 - 1.0}
	if v.LengthSq() < 1.0 {
		return v
	}
	return randUnitCircle()
}

var simpleTerrainVerts = []Vector{
	{350.00, 425.07}, {336.00, 436.55}, {272.00, 435.39}, {258.00, 427.63}, {225.28, 420.00}, {202.82, 396.00},
	{191.81, 388.00}, {189.00, 381.89}, {173.00, 380.39}, {162.59, 368.00}, {150.47, 319.00}, {128.00, 311.55},
	{119.14, 286.00}, {126.84, 263.00}, {120.56, 227.00}, {141.14, 178.00}, {137.52, 162.00}, {146.51, 142.00},
	{156.23, 136.00}, {158.00, 118.27}, {170.00, 100.77}, {208.43, 84.00}, {224.00, 69.65}, {249.30, 68.00},
	{257.00, 54.77}, {363.00, 45.94}, {374.15, 54.00}, {386.00, 69.60}, {413.00, 70.73}, {456.00, 84.89},
	{468.09, 99.00}, {467.09, 123.00}, {464.92, 135.00}, {469.00, 141.03}, {497.00, 148.67}, {513.85, 180.00},
	{509.56, 223.00}, {523.51, 247.00}, {523.00, 277.00}, {497.79, 311.00}, {478.67, 348.00}, {467.90, 360.00},
	{456.76, 382.00}, {432.95, 389.00}, {417.00, 411.32}, {373.00, 433.19}, {361.00, 430.02}, {350.00, 425.07},
}

func addCircle(space *Space, index int, radius float64) {
	mass := radius * radius / 25.0
	body := space.AddBody(NewBody(mass, MomentForCircle(mass, 0, radius, VectorZero())))
	body.SetPosition(randUnitCircle().Mult(180))

	shape := space.AddShape(NewCircle(body, radius, VectorZero()))
	shape.SetElasticity(0)
	shape.SetFriction(0.9)
}

func update(space *Space, dt float64) {
	space.Step(dt)
}

func simpleTerrain() *Space {
	space := NewSpace()
	space.Iterations = 10
	space.SetGravity(Vector{0, -100})
	space.SetCollisionSlop(0.5)

	offset := Vector{-320, -240}
	for i := 0; i < len(simpleTerrainVerts)-1; i++ {
		a := simpleTerrainVerts[i]
		b := simpleTerrainVerts[i+1]
		space.AddShape(NewSegment(space.StaticBody, a.Add(offset), b.Add(offset), 0))
	}

	return space
}

func simpleTerrainCircles_1000() *Space {
	space := simpleTerrain()
	for i := 0; i < 1000; i++ {
		addCircle(space, i, 5)
	}
	return space
}

func main() {
	examples.Main(simpleTerrainCircles_1000(), 620, 480, 1.0/60.0, update)
}
