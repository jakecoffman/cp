package main

import (
	"log"
	"math/rand"
	"time"

	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

var (
	tankBody, tankControlBody *Body
)

const (
	width  = 640
	height = 480

	hwidth  = width / 2
	hheight = height / 2
)

func main() {
	log.SetFlags(log.LstdFlags | log.Lshortfile)
	rand.Seed(time.Now().Unix())

	space := NewSpace()
	//space.Iterations = 10
	space.SleepTimeThreshold = 0.5

	sides := []Vector{
		{-hwidth, -hheight}, {-hwidth, hheight},
		{hwidth, -hheight}, {hwidth, hheight},
		{-hwidth, -hheight}, {hwidth, -hheight},
		{-hwidth, hheight}, {hwidth, hheight},
	}

	for i := 0; i < len(sides); i += 2 {
		var seg *Shape
		seg = space.AddShape(NewSegment(space.StaticBody, sides[i], sides[i+1], 0))
		seg.SetElasticity(1)
		seg.SetFriction(1)
		seg.SetFilter(examples.NotGrabbableFilter)
	}

	for i := 0; i < 50; i++ {
		body := addBox(space, 20, 1)
		pivot := space.AddConstraint(NewPivotJoint2(space.StaticBody, body, Vector{}, Vector{}))
		pivot.SetMaxBias(0)       // disable joint correction
		pivot.SetMaxForce(1000.0) // emulate linear friction

		gear := space.AddConstraint(NewGearJoint(space.StaticBody, body, 0.0, 1.0))
		gear.SetMaxBias(0)
		gear.SetMaxForce(5000.0) // emulate angular friction
	}

	// We joint the tank to the control body and control the tank indirectly by modifying the control body.
	tankControlBody = space.AddBody(NewKinematicBody())
	tankBody = addBox(space, 30, 10)

	pivot := space.AddConstraint(NewPivotJoint2(tankControlBody, tankBody, Vector{}, Vector{}))
	pivot.SetMaxBias(0)
	pivot.SetMaxForce(10000)

	gear := space.AddConstraint(NewGearJoint(tankControlBody, tankBody, 0.0, 1.0))
	gear.SetErrorBias(0) // attempt to fully correct the joint each step
	gear.SetMaxBias(1.2)
	gear.SetMaxForce(50000)

	examples.Main(space, 1.0/60.0, update, examples.DefaultDraw)
}

func addBox(space *Space, size, mass float64) *Body {
	radius := (&Vector{size, size}).Length()
	body := space.AddBody(NewBody(mass, MomentForBox(mass, size, size)))
	body.SetPosition(Vector{rand.Float64()*(width-2*radius) - (hwidth - radius), rand.Float64()*(height-2*radius) - (hheight - radius)})

	shape := space.AddShape(NewBox(body, size, size, 0))
	shape.SetElasticity(0)
	shape.SetFriction(0.7)
	return body
}

func update(space *Space, dt float64) {
	// turn the control body based on the angle relative to the actual body
	mouseDelta := examples.Mouse.Sub(tankBody.Position())
	turn := tankBody.Rotation().Unrotate(mouseDelta).ToAngle()
	tankControlBody.SetAngle(tankBody.Angle() - turn)

	// drive the tank towards the mouse
	if examples.Mouse.Near(tankBody.Position(), 30.0) {
		tankControlBody.SetVelocityVector(Vector{}) // stop
	} else {
		var direction float64
		if mouseDelta.Dot(tankBody.Rotation()) > 0.0 {
			direction = 1.0
		} else {
			direction = -1.0
		}
		tankControlBody.SetVelocityVector(tankBody.Rotation().Rotate(Vector{30.0 * direction, 0.0}))
	}

	space.Step(dt)
}
