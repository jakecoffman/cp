package examples

import (
	"math"

	"log"

	"github.com/go-gl/gl/v2.1/gl"
	"github.com/jakecoffman/physics"
)

func SetupGL() {
	version := gl.GoStr(gl.GetString(gl.VERSION))
	log.Println("OpenGL version", version)

	vshader := CompileShader(gl.VERTEX_SHADER, vertexShader)
	fshader := CompileShader(gl.FRAGMENT_SHADER, fragmentShader)

	program = LinkProgram(vshader, fshader)

	// TODO implement OS specific stuff in demo_darwin, etc
	gl.GenVertexArraysAPPLE(1, &vao)
	gl.BindVertexArrayAPPLE(vao)

	CheckGLErrors()

	gl.GenBuffers(1, &vbo)
	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)

	CheckGLErrors()

	SetAttribute(program, "vertex", 2, gl.FLOAT, 48, 0)
	SetAttribute(program, "aa_coord", 2, gl.FLOAT, 48, 8)
	SetAttribute(program, "fill_color", 4, gl.FLOAT, 48, 16)
	SetAttribute(program, "outline_color", 4, gl.FLOAT, 48, 32)

	// TODO non-apple
	gl.BindVertexArrayAPPLE(0)

	CheckGLErrors()

	gl.ClearColor(0, 1, 1, 1)
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

const vertexShader = `
		attribute vec2 vertex;
		attribute vec2 aa_coord;
		attribute vec4 fill_color;
		attribute vec4 outline_color;

		varying vec2 v_aa_coord;
		varying vec4 v_fill_color;
		varying vec4 v_outline_color;

		void main(void){
			// TODO: get rid of the GL 2.x matrix bit eventually?
			gl_Position = gl_ModelViewProjectionMatrix*vec4(vertex, 0.0, 1.0);

			v_fill_color = fill_color;
			v_outline_color = outline_color;
			v_aa_coord = aa_coord;
		}
	`

const fragmentShader = `
		uniform float u_outline_coef;

		varying vec2 v_aa_coord;
		varying vec4 v_fill_color;
		//const vec4 v_fill_color = vec4(0.0, 0.0, 0.0, 1.0);
		varying vec4 v_outline_color;

		float aa_step(float t1, float t2, float f)
		{
			//return step(t2, f);
			return smoothstep(t1, t2, f);
		}

		void main(void)
		{
			float l = length(v_aa_coord);

			// Different pixel size estimations are handy.
			//float fw = fwidth(l);
			//float fw = length(vec2(dFdx(l), dFdy(l)));
			float fw = length(fwidth(v_aa_coord));

			// Outline width threshold.
			float ow = 1.0 - fw;//*u_outline_coef;

			// Fill/outline color.
			float fo_step = aa_step(max(ow - fw, 0.0), ow, l);
			vec4 fo_color = mix(v_fill_color, v_outline_color, fo_step);

			// Use pre-multiplied alpha.
			float alpha = 1.0 - aa_step(1.0 - fw, 1.0, l);
			gl_FragColor = fo_color*(fo_color.a*alpha);
			//gl_FragColor = vec4(vec3(l), 1);
		}
	`
