package main

import (
	"math"

	"math/rand"

	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

const (
	DENSITY                 = 1.0 / 10000.0
	MAX_VERTEXES_PER_VORNOI = 16
)

type WorleyContex struct {
	seed          uint32
	cellSize      float64
	width, height int
	bb            BB
	focus         Vector
}

func HashVect(x, y, seed uint32) Vector {
	border := 0.05
	var h uint32 = (x*1640531513 ^ y*2654435789) + seed

	return Vector{
		Lerp(border, 1.0-border, float64(h&0xFFFF)/0xFFFF),
		Lerp(border, 1.0-border, float64((h>>16)&0xFFFF)/0xFFFF),
	}
}

func WorleyPoint(i, j int, context *WorleyContex) Vector {
	size := context.cellSize
	width := context.width
	height := context.height
	bb := context.bb

	fv := HashVect(uint32(i), uint32(j), context.seed)

	return Vector{
		Lerp(bb.L, bb.R, 0.5) + size*(float64(i)+fv.X-float64(width)*0.5),
		Lerp(bb.B, bb.T, 0.5) + size*(float64(j)+fv.Y-float64(height)*0.5),
	}
}

func ClipCell(shape *Shape, center Vector, i, j int, context *WorleyContex, verts []Vector, clipped []Vector, count int) int {
	other := WorleyPoint(i, j, context)
	if shape.PointQuery(other).Distance > 0 {
		copy(clipped[:count], verts[:count])
		return count
	}

	n := other.Sub(center)
	dist := n.Dot(center.Lerp(other, 0.5))

	var clippedCount int
	i = count - 1
	for j = 0; j < count; j++ {
		a := verts[i]
		aDist := a.Dot(n) - dist

		if aDist <= 0 {
			clipped[clippedCount] = a
			clippedCount++
		}

		b := verts[j]
		bDist := b.Dot(n) - dist

		if aDist*bDist < 0 {
			t := math.Abs(aDist) / (math.Abs(aDist) + math.Abs(bDist))

			clipped[clippedCount] = a.Lerp(b, t)
			clippedCount++
		}

		i = j
	}

	return clippedCount
}

func ShatterCell(space *Space, shape *Shape, cell Vector, cellI, cellJ int, context *WorleyContex) {
	body := shape.Body()

	ping := make([]Vector, MAX_VERTEXES_PER_VORNOI)
	pong := make([]Vector, MAX_VERTEXES_PER_VORNOI)

	poly := shape.Class.(*PolyShape)
	count := poly.Count()
	if count > MAX_VERTEXES_PER_VORNOI {
		count = MAX_VERTEXES_PER_VORNOI
	}

	for i := 0; i < count; i++ {
		ping[i] = body.LocalToWorld(poly.Vert(i))
	}

	for i := 0; i < context.width; i++ {
		for j := 0; j < context.height; j++ {
			if !(i == cellI && j == cellJ) && shape.PointQuery(cell).Distance < 0 {
				count = ClipCell(shape, cell, i, j, context, ping, pong, count)
				copy(ping, pong)
			}
		}
	}

	centroid := CentroidForPoly(count, ping)
	mass := AreaForPoly(count, ping, 0) * DENSITY
	moment := MomentForPoly(mass, count, ping, centroid.Neg(), 0)

	newBody := space.AddBody(NewBody(mass, moment))
	newBody.SetPosition(centroid)
	newBody.SetVelocityVector(body.VelocityAtWorldPoint(centroid))
	newBody.SetAngularVelocity(body.AngularVelocity())

	transform := NewTransformTranslate(centroid.Neg())
	newShape := space.AddShape(NewPolyShape(newBody, count, ping, transform, 0))
	newShape.SetFriction(shape.Friction())
}

func ShatterShape(space *Space, shape *Shape, cellSize float64, focus Vector) {
	space.RemoveShape(shape)
	space.RemoveBody(shape.Body())

	bb := shape.BB()
	width := int((bb.R-bb.L)/cellSize) + 1
	height := int((bb.T-bb.B)/cellSize) + 1
	context := WorleyContex{rand.Uint32(), cellSize, width, height, bb, focus}

	for i := 0; i < context.width; i++ {
		for j := 0; j < context.height; j++ {
			cell := WorleyPoint(i, j, &context)
			if shape.PointQuery(cell).Distance < 0 {
				ShatterCell(space, shape, cell, i, j, &context)
			}
		}
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
	height := 200.0
	mass := width * height * DENSITY
	moment := MomentForBox(mass, width, height)

	body := space.AddBody(NewBody(mass, moment))

	shape = space.AddShape(NewBox(body, width, height, 0))
	shape.SetFriction(0.6)

	examples.Main(space, 1.0/60.0, update, examples.DefaultDraw)
}

func update(space *Space, dt float64) {
	space.Step(dt)

	if examples.RightDown {
		info := space.PointQueryNearest(examples.Mouse, 0, examples.GrabFilter)
		if info.Shape != nil {
			bb := info.Shape.BB()
			cellSize := math.Max(bb.R-bb.L, bb.T-bb.B) / 5.0
			if cellSize > 5.0 {
				ShatterShape(space, info.Shape, cellSize, examples.Mouse)
			}
		}
	}
}
