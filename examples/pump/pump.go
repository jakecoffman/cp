package main

import (
	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
	"math"
)

const numBalls = 5

var motor *SimpleMotor
var balls [numBalls]*Body

func main() {
	space := NewSpace()
	space.SetGravity(Vector{0, -600})

	walls := []Vector{
		{-256, 16}, {-256, 300},
		{-256, 16}, {-192, 0},
		{-192, 0}, {-192, -64},
		{-128, -64}, {-122, 144},
		{-192, 80}, {-192, 176},
		{-192, 176}, {-128, 240},
		{-128, 144}, {192, 64},
	}

	var shape *Shape
	for i:=0; i<len(walls)-1; i+=2 {
		shape = space.AddShape(NewSegment(space.StaticBody, walls[i], walls[i+1], 2))
		shape.SetElasticity(0)
		shape.SetFriction(0.5)
		shape.SetFilter(examples.NotGrabbableFilter)
	}

	verts := []Vector{
		{-30, -80},
		{-30, 80},
		{30, 64},
		{30, -80},
	}

	plunger := space.AddBody(NewBody(1, INFINITY))
	plunger.SetPosition(Vector{-160, -80})

	shape = space.AddShape(NewPolyShape(plunger, verts, NewTransformIdentity(), 0))
	shape.SetElasticity(1)
	shape.SetFriction(0.5)
	shape.SetFilter(NewShapeFilter(NO_GROUP, 1, 1))

	for i:=0; i<numBalls; i++ {
		balls[i] = addBall(space, Vector{-224 + float64(i), 80 + 64*float64(i)})
	}

	smallGear := space.AddBody(NewBody(10, MomentForCircle(10, 80, 0, VectorZero())))
	smallGear.SetPosition(Vector{-160, -160})
	smallGear.SetAngle(-math.Pi/2)

	shape = space.AddShape(NewCircle(smallGear, 80, VectorZero()))
	shape.SetFilter(SHAPE_FILTER_NONE)

	space.AddConstraint(NewPivotJoint2(space.StaticBody, smallGear, Vector{-160, -160}, VectorZero()))

	bigGear := space.AddBody(NewBody(40, MomentForCircle(40, 160, 0, VectorZero())))
	bigGear.SetPosition(Vector{80, -160})
	bigGear.SetAngle(math.Pi/2)

	shape = space.AddShape(NewCircle(bigGear, 160, VectorZero()))
	shape.SetFilter(SHAPE_FILTER_NONE)

	space.AddConstraint(NewPivotJoint2(space.StaticBody, bigGear, Vector{80, -160}, VectorZero()))

	space.AddConstraint(NewPinJoint(smallGear, plunger, Vector{80, 0}, VectorZero()))
	space.AddConstraint(NewGearJoint(smallGear, bigGear, -math.Pi/2, -2))

	bottom := -300.0
	top := 32.0
	feeder := space.AddBody(NewBody(1, MomentForSegment(1, Vector{-224, bottom}, Vector{-224, top}, 0)))
	feeder.SetPosition(Vector{-224, (bottom+top)/2})

	length := top-bottom
	shape = space.AddShape(NewSegment(feeder, Vector{0, length/2}, Vector{0, -length/2}, 20))
	shape.SetFilter(examples.GrabFilter)

	space.AddConstraint(NewPivotJoint2(space.StaticBody, feeder, Vector{-224, bottom}, Vector{0, -length/2}))
	anchr := feeder.WorldToLocal(Vector{-224, -160})
	space.AddConstraint(NewPinJoint(feeder, smallGear, anchr, Vector{0, 80}))

	motor = space.AddConstraint(NewSimpleMotor(space.StaticBody, bigGear, 3)).Class.(*SimpleMotor)

	examples.Main(space, 640, 480, 1.0/60.0, update)
}

func update(space *Space, dt float64) {
	coef := (2.0+examples.Keyboard.Y)/3
	rate := examples.Keyboard.X*30*coef

	motor.Rate = rate
	if rate != 0 {
		motor.SetMaxForce(1000000)
	} else {
		motor.SetMaxForce(0)
	}

	space.Step(dt)

	for i:=0; i<numBalls; i++ {
		ball := balls[i]
		pos := ball.Position()

		if pos.X > 320 {
			ball.SetVelocityVector(VectorZero())
			ball.SetPosition(Vector{-224, 200})
		}
	}
}


func addBall(space *Space, pos Vector) *Body {
	body := space.AddBody(NewBody(1, MomentForCircle(1, 30, 0, VectorZero())))
	body.SetPosition(pos)

	shape := space.AddShape(NewCircle(body, 30, VectorZero()))
	shape.SetElasticity(0)
	shape.SetFriction(0.5)
	return body
}