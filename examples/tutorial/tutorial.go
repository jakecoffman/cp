package main

import (
	"io/ioutil"
	"log"

	"os"
	"runtime/pprof"

	"time"

	. "github.com/jakecoffman/physics"
	"github.com/jakecoffman/physics/examples"
)

func main() {
	f, err := os.Create("cpuprofile")
	if err != nil {
		log.Fatal(err)
	}
	pprof.StartCPUProfile(f)
	defer pprof.StopCPUProfile()

	gravity := &Vector{0, -100}

	// Create an empty space.
	space := NewSpace()
	space.SetGravity(gravity)

	// Add a static line segment shape for the ground.
	// We'll make it slightly tilted so the ball will roll off.
	// We attach it to space->staticBody to tell Chipmunk it shouldn't be movable.
	ground := NewSegment(space.StaticBody(), &Vector{-20, 5}, &Vector{20, -5}, 0)
	ground.U = 1
	space.AddShape(ground)

	// Now let's make a ball that falls onto the line and rolls off.
	// First we need to make a cpBody to hold the physical properties of the object.
	// These include the mass, position, velocity, angle, etc. of the object.
	// Then we attach collision shapes to the cpBody to give it a size and shape.
	//var radius float64 = 5
	var mass float64 = 1

	// The moment of inertia is like mass for rotation
	// Use the cpMomentFor*() functions to help you approximate it.
	moment := MomentForBox(mass, 5, 5)

	// The cpSpaceAdd*() functions return the thing that you are adding.
	// It's convenient to create and add an object in one line.
	ballBody := space.AddBody(NewBody(mass, moment))
	ballBody.SetPosition(&Vector{0, 100})

	// Now we create the collision shape for the ball.
	// You can create multiple collision shapes that point to the same body.
	// They will all be attached to the body and move around to follow it.
	radius := (&Vector{5, 5}).Length()
	ballShape := space.AddShape(NewBox(ballBody, 5, 5, radius))
	ballShape.U = 0.7

	// Now that it's all set up, we simulate all the objects in the space by
	// stepping forward through time in small increments called steps.
	// It is *highly* recommended to use a fixed size time step.
	log.SetFlags(0)
	log.SetOutput(ioutil.Discard)

	go func() {
		time.Sleep(10 * time.Second)
		os.Exit(1)
	}()

	examples.Main(space, 200, 200, 0.01666)
}
