package examples

import (
	"image/color"
	"math"

	"github.com/jakecoffman/physics"
)

func ColorForShape(shape *physics.Shape, data interface{}) color.Color {
	if shape.GetSensor() {
		return color.RGBA{255, 255, 255, 255 * 01}
	}

	body := shape.Body()

	if body.IsSleeping() {
		return color.RGBA{51, 51, 51, 255}
	}

	if body.IdleTime() > shape.Space().SleepTimeThreshold {
		return color.RGBA{153, 153, 153, 255}
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
		return color.RGBA{uint8(intensity * 255), 0, 0, 1}
	}

	coef := intensity / (max - min)
	return color.RGBA{
		uint8((r - min) * coef * 255),
		uint8((g - min) * coef * 255),
		uint8((b - min) * coef * 255),
		1,
	}
}
