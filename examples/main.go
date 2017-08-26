package examples

import (
	"runtime"

	"math"

	"log"

	"github.com/go-gl/gl/v2.1/gl"
	"github.com/go-gl/glfw/v3.2/glfw"
	"github.com/go-gl/mathgl/mgl32"
	. "github.com/jakecoffman/physics"
)

var Mouse *Vector
var mouseBody *Body

var accumulator float64
var lastTime float64

func init() {
	runtime.LockOSThread()
}

func Display(space *Space, tick float64, update UpdateFunc) {
	gl.MatrixMode(gl.MODELVIEW)
	gl.LoadIdentity()
	gl.Translatef(0.0, 0.0, 0.0)
	gl.Scalef(1, 1, 1)

	Update(space, tick, update)

	ClearRenderer()

	// builds triangles buffer
	DrawSpace(space, NewDrawOptions(
		DRAW_SHAPES|DRAW_CONSTRAINTS|DRAW_COLLISION_POINTS,
		FColor{R: 1, A: 1},
		FColor{G: 1, A: 1},
		FColor{B: 1, A: 1},
		nil,
	))

	// gives buffer to open-gl to draw
	FlushRenderer()
}

type UpdateFunc func(*Space, float64)

func Update(space *Space, tick float64, update UpdateFunc) {
	t := glfw.GetTime()
	dt := t - lastTime
	if dt > 0.2 {
		dt = 0.2
	}

	for accumulator += dt; accumulator > tick; accumulator -= tick {
		// Tick
		if mouseBody == nil {
			panic("WAT")
		}
		newPoint := mouseBody.Position().Lerp(Mouse, 0.25)
		mouseBody.SetVelocityVector(newPoint.Sub(mouseBody.Position()).Mult(60.0))
		mouseBody.SetPosition(newPoint)
		update(space, tick)
	}

	lastTime = t
}

func Main(space *Space, width, height int, tick float64, update UpdateFunc) {
	if err := glfw.Init(); err != nil {
		panic(err)
	}
	defer glfw.Terminate()

	glfw.WindowHint(glfw.Resizable, glfw.False)
	glfw.WindowHint(glfw.ContextVersionMajor, 2)
	glfw.WindowHint(glfw.ContextVersionMinor, 1)
	window, err := glfw.CreateWindow(width, height, "Cube", nil, nil)
	if err != nil {
		panic(err)
	}
	defer window.Destroy()
	window.MakeContextCurrent()
	glfw.SwapInterval(1)

	if err := gl.Init(); err != nil {
		panic(err)
	}

	SetupGL()

	wi, hi := window.GetFramebufferSize()
	w := float64(wi)
	h := float64(hi)
	gl.Viewport(0, 0, int32(wi), int32(hi))

	scale := math.Min(w/float64(width), h/float64(height))
	hw := w * (0.5 / scale)
	hh := h * (0.5 / scale)

	gl.MatrixMode(gl.PROJECTION)
	gl.LoadIdentity()
	gl.Ortho(-hw, hw, -hh, hh, -1, 1)

	window.SetCharCallback(func(w *glfw.Window, char rune) {
		if char == 'q' {
			window.SetShouldClose(true)
			return
		}
	})

	window.SetCursorPosCallback(func(w *glfw.Window, xpos float64, ypos float64) {
		ww, wh := w.GetSize()
		Mouse = MouseToSpace(xpos, ypos, ww, wh)
	})
	Mouse = VectorZero()

	mouseBody = NewKinematicBody()

	for !window.ShouldClose() {
		Display(space, tick, update)
		window.SwapBuffers()
		glfw.PollEvents()
		gl.Clear(gl.COLOR_BUFFER_BIT)
	}
}

func MouseToSpace(x, y float64, ww, wh int) *Vector {
	var model [16]float64
	gl.GetDoublev(gl.MODELVIEW_MATRIX, &model[0])
	modelMat := mgl32.Mat4{}
	for i, m := range model {
		modelMat[i] = float32(m)
	}

	var proj [16]float64
	gl.GetDoublev(gl.PROJECTION_MATRIX, &proj[0])
	projMat := mgl32.Mat4{}
	for i, p := range proj {
		projMat[i] = float32(p)
	}

	var view [4]int32
	gl.GetIntegerv(gl.VIEWPORT, &view[0])

	//var mx, my, mz float64
	obj, err := mgl32.UnProject(
		mgl32.Vec3{float32(x), float32(float64(ww) - y), 0},
		modelMat,
		projMat,
		0, 0,
		ww, wh,
	)

	if err != nil {
		panic(err)
	}

	loc := &Vector{float64(obj.X()), float64(obj.Y())}
	log.Println("Mouse:", loc)
	return loc
}
