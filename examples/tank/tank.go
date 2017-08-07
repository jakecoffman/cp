package main

import (
	"math/rand"

	"log"

	"github.com/go-gl/gl/v2.1/gl"
	"github.com/go-gl/glfw/v3.2/glfw"
	. "github.com/jakecoffman/physics"
)

var space *Space

func main() {
	space = NewSpace()
	space.Iterations = 10
	space.SleepTimeThreshold = 0.5

	seg1 := NewSegment(space.Body, &Vector{-320, 420}, &Vector{-320, 240}, 0)
	space.AddShape(seg1)
	seg1.Shape.Body().Activate()
	seg1.Shape.E = 1
	seg1.Shape.U = 1

	for i := 0; i < 50; i++ {
		addBox(space, 20, 1)
		//pivot := NewPivotJoint2()
		//space.AddConstraint()
	}

	//tankControlBody := space.AddBody(NewKinematic())
	//tankBody := addBox(space, 30, 10)

	if err := glfw.Init(); err != nil {
		log.Fatal(err)
	}
	defer glfw.Terminate()

	glfw.WindowHint(glfw.Resizable, glfw.False)
	glfw.WindowHint(glfw.ContextVersionMajor, 2)
	glfw.WindowHint(glfw.ContextVersionMinor, 1)
	window, err := glfw.CreateWindow(800, 600, "Cube", nil, nil)
	if err != nil {
		panic(err)
	}
	window.MakeContextCurrent()

	if err := gl.Init(); err != nil {
		panic(err)
	}

	window.SetCharCallback(func(w *glfw.Window, char rune){
		if char == 'q' {
			window.SetShouldClose(true)
			return
		}
	})

	setupScene()
	for !window.ShouldClose() {
		drawScene()
		window.SwapBuffers()
		glfw.PollEvents()
	}
}

func addBox(space *Space, size, mass float64) *Body {
	radius := (&Vector{size, size}).Length()
	body := space.AddBody(NewBody(mass, MomentForBox(mass, size, size)))
	body.SetPosition(&Vector{rand.Float64()*(640-2*radius) - (320 - radius), rand.Float64()*(480-2*radius) - (240 - radius)})

	shape := NewBox(body, size, size, 0)
	space.AddShape(shape)
	shape.E = 0
	shape.U = 0.7
	return body
}

func setupScene() {
	gl.Enable(gl.DEPTH_TEST)
	gl.Enable(gl.LIGHTING)

	gl.ClearColor(0.5, 0.5, 0.5, 0.0)
	gl.ClearDepth(1)
	gl.DepthFunc(gl.LEQUAL)

	ambient := []float32{0.5, 0.5, 0.5, 1}
	diffuse := []float32{1, 1, 1, 1}
	lightPosition := []float32{-5, 5, 10, 0}
	gl.Lightfv(gl.LIGHT0, gl.AMBIENT, &ambient[0])
	gl.Lightfv(gl.LIGHT0, gl.DIFFUSE, &diffuse[0])
	gl.Lightfv(gl.LIGHT0, gl.POSITION, &lightPosition[0])
	gl.Enable(gl.LIGHT0)

	gl.MatrixMode(gl.PROJECTION)
	gl.LoadIdentity()
	gl.Frustum(-1, 1, -1, 1, 1.0, 10.0)
	gl.MatrixMode(gl.MODELVIEW)
	gl.LoadIdentity()
}

var (
	texture   uint32
	rotationX float32
	rotationY float32
)

var accumulator float64
var lastTime float64

func drawScene() {
	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

	gl.MatrixMode(gl.MODELVIEW)
	gl.LoadIdentity()
	gl.Translatef(0, 0, 0.0)
	gl.Scalef(1, 1, 1)

	// Update
	time := glfw.GetTime()
	dt := time - lastTime
	if dt > 0.2 {
		dt = 0.2
	}

	var fixed_dt float64 = 1/60

	for accumulator += dt; accumulator > fixed_dt; accumulator -= fixed_dt {
		Tick(fixed_dt)
	}

	lastTime = time

	gl.End()
}

func Tick(dt float64) {
	space.Step(dt)
}