package examples

import (
	"image/color"
	"math"

	"github.com/jakecoffman/physics"
	"github.com/go-gl/gl/v2.1/gl"
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

func ColorForShape(shape *physics.Shape, data interface{}) color.Color {
	if shape.GetSensor() {
		return color.RGBA{R: 255, G: 255, B: 255, A: 255 * 01}
	}

	body := shape.Body()

	if body.IsSleeping() {
		return color.RGBA{R: 51, G: 51, B: 51, A: 255}
	}

	if body.IdleTime() > shape.Space().SleepTimeThreshold {
		return color.RGBA{R: 153, G: 153, B: 153, A: 255}
	}

	val := shape.HashId()

	// scramble the bits up using Robert Jenkins' 32 bit integer hash function
	val = (val + 0x7ed55d16) + (val << 12)
	val = (val ^ 0xc761c23c) ^ (val >> 19)
	val = (val + 0x165667b1) + (val << 5)
	val = (val + 0xd3a2646c) ^ (val << 9)
	val = (val + 0xfd7046c5) + (val << 3)
	val = (val ^ 0xb55a4f09) ^ (val >> 16)

	r := float64((val >> 0) & 0xFF)
	g := float64((val >> 8) & 0xFF)
	b := float64((val >> 16) & 0xFF)

	max := math.Max(math.Max(r, g), b)
	min := math.Min(math.Min(r, g), b)
	var intensity float64
	if body.GetType() == physics.BODY_STATIC {
		intensity = 0.15
	} else {
		intensity = 0.75
	}

	if min == max {
		return color.RGBA{R: uint8(intensity * 255), A: 1}
	}

	coef := intensity / (max - min)
	return color.RGBA{
		R: uint8((r - min) * coef * 255),
		G: uint8((g - min) * coef * 255),
		B: uint8((b - min) * coef * 255),
		A: 1,
	}
}
