package main

import (
	"math"

	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

const (
	PLAYER_VELOCITY = 500.0

	PLAYER_GROUND_ACCEL_TIME = 0.1
	PLAYER_GROUND_ACCEL      = PLAYER_VELOCITY / PLAYER_GROUND_ACCEL_TIME

	PLAYER_AIR_ACCEL_TIME = 0.25
	PLAYER_AIR_ACCEL      = PLAYER_VELOCITY / PLAYER_AIR_ACCEL_TIME

	JUMP_HEIGHT       = 50.0
	JUMP_BOOST_HEIGHT = 55.0
	FALL_VELOCITY     = 900.0
	GRAVITY           = 2000.0
)

var playerBody *Body
var playerShape *Shape

var remainingBoost float64
var grounded, lastJumpState bool

func playerUpdateVelocity(body *Body, gravity Vector, damping, dt float64) {
	jumpState := examples.Keyboard.Y > 0

	// Grab the grounding normal from last frame
	groundNormal := Vector{}
	playerBody.EachArbiter(func(arb *Arbiter) {
		n := arb.Normal().Neg()

		if n.Y > groundNormal.Y {
			groundNormal = n
		}
	})

	grounded = groundNormal.Y > 0
	if groundNormal.Y < 0 {
		remainingBoost = 0
	}

	// Do a normal-ish update
	boost := jumpState && remainingBoost > 0
	var g Vector
	if !boost {
		g = gravity
	}
	body.UpdateVelocity(g, damping, dt)

	// Target horizontal speed for air/ground control
	targetVx := PLAYER_VELOCITY * examples.Keyboard.X

	// Update the surface velocity and friction
	// Note that the "feet" move in the opposite direction of the player.
	surfaceV := Vector{-targetVx, 0}
	playerShape.SetSurfaceV(surfaceV)
	if grounded {
		playerShape.SetFriction(PLAYER_GROUND_ACCEL / GRAVITY)
	} else {
		playerShape.SetFriction(0)
	}

	// Apply air control if not grounded
	if !grounded {
		v := playerBody.Velocity()
		playerBody.SetVelocity(LerpConst(v.X, targetVx, PLAYER_AIR_ACCEL*dt), v.Y)
	}

	v := body.Velocity()
	body.SetVelocity(v.X, Clamp(v.Y, -FALL_VELOCITY, INFINITY))
}

func main() {
	space := NewSpace()
	space.Iterations = 10
	space.SetGravity(Vector{0, -GRAVITY})

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

	// player
	playerBody = space.AddBody(NewBody(1, INFINITY))
	playerBody.SetPosition(Vector{0, -200})
	playerBody.SetVelocityUpdateFunc(playerUpdateVelocity)

	playerShape = space.AddShape(NewBox2(playerBody, BB{-15, -27.5, 15, 27.5}, 10))
	playerShape.SetElasticity(0)
	playerShape.SetFriction(0)
	playerShape.SetCollisionType(1)

	for i := 0; i < 6; i++ {
		for j := 0; j < 3; j++ {
			body := space.AddBody(NewBody(4, INFINITY))
			body.SetPosition(Vector{float64(100 + j*60), float64(-200 + i*60)})

			shape := space.AddShape(NewBox(body, 50, 50, 0))
			shape.SetElasticity(0)
			shape.SetFriction(0.7)
		}
	}

	examples.Main(space, 1.0/180.0, update, examples.DefaultDraw)
}

func update(space *Space, dt float64) {
	jumpState := examples.Keyboard.Y > 0

	// If the jump key was just pressed this frame, jump!
	if jumpState && !lastJumpState && grounded {
		jumpV := math.Sqrt(2.0 * JUMP_HEIGHT * GRAVITY)
		playerBody.SetVelocityVector(playerBody.Velocity().Add(Vector{0, jumpV}))

		remainingBoost = JUMP_BOOST_HEIGHT / jumpV
	}

	space.Step(dt)

	remainingBoost -= dt
	lastJumpState = jumpState
}
