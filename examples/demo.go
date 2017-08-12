package examples

import (
	"math"

	"github.com/go-gl/gl/v2.1/gl"
	"github.com/jakecoffman/physics"
)

func SetupGL() {
	DrawInit()
	//TextInit()

	gl.ClearColor(52/255, 62/255, 72/255, 1)
	gl.Clear(gl.COLOR_BUFFER_BIT)

	gl.Enable(gl.LINE_SMOOTH)
	gl.Enable(gl.POINT_SMOOTH)

	gl.Hint(gl.LINE_SMOOTH_HINT, gl.DONT_CARE)
	gl.Hint(gl.POINT_SMOOTH_HINT, gl.DONT_CARE)

	gl.Enable(gl.BLEND)
	gl.BlendFunc(gl.ONE, gl.ONE_MINUS_SRC_ALPHA)
}

func ColorForShape(shape *physics.Shape, data interface{}) physics.FColor {
	if shape.GetSensor() {
		return physics.FColor{R: 1, G: 1, B: 1, A: .1}
	}

	body := shape.Body()

	if body.IsSleeping() {
		return physics.FColor{R: .2, G: .2, B: .2, A: 1}
	}

	if body.IdleTime() > shape.Space().SleepTimeThreshold {
		return physics.FColor{R: .66, G: .66, B: .66, A: 1}
	}

	val := shape.HashId()

	// scramble the bits up using Robert Jenkins' 32 bit integer hash function
	val = (val + 0x7ed55d16) + (val << 12)
	val = (val ^ 0xc761c23c) ^ (val >> 19)
	val = (val + 0x165667b1) + (val << 5)
	val = (val + 0xd3a2646c) ^ (val << 9)
	val = (val + 0xfd7046c5) + (val << 3)
	val = (val ^ 0xb55a4f09) ^ (val >> 16)

	r := float32((val >> 0) & 0xFF)
	g := float32((val >> 8) & 0xFF)
	b := float32((val >> 16) & 0xFF)

	max := float32(math.Max(math.Max(float64(r), float64(g)), float64(b)))
	min := float32(math.Min(math.Min(float64(r), float64(g)), float64(b)))
	var intensity float32
	if body.GetType() == physics.BODY_STATIC {
		intensity = 0.15
	} else {
		intensity = 0.75
	}

	if min == max {
		return physics.FColor{R: intensity, A: 1}
	}

	var coef float32 = intensity / (max - min)
	return physics.FColor{
		R: (r - min) * coef,
		G: (g - min) * coef,
		B: (b - min) * coef,
		A: 1,
	}
}
