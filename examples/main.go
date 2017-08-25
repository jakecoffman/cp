package examples

import (
	"runtime"

	"github.com/go-gl/gl/v2.1/gl"
	"github.com/go-gl/glfw/v3.2/glfw"
	. "github.com/jakecoffman/physics"
	"math"
)

var accumulator float64
var lastTime float64

func init() {
	runtime.LockOSThread()
}

func Display(space *Space, tick float64) {
	gl.MatrixMode(gl.MODELVIEW)
	gl.LoadIdentity()
	gl.Translatef(0.0, 0.0, 0.0)
	gl.Scalef(1, 1, 1)

	Update(space, tick)

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

func Update(space *Space, tick float64) {
	t := glfw.GetTime()
	dt := t - lastTime
	if dt > 0.2 {
		dt = 0.2
	}

	for accumulator += dt; accumulator > tick; accumulator -= tick {
		space.Step(tick)
	}

	lastTime = t
}

func Main(space *Space, width, height int, tick float64) {
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

	for !window.ShouldClose() {
		Display(space, tick)
		window.SwapBuffers()
		glfw.PollEvents()
		gl.Clear(gl.COLOR_BUFFER_BIT)
	}
}
