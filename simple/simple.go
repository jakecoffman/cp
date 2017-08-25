package main

import (
	"fmt"
	"log"
	"os"
	"strings"

	"github.com/go-gl/gl/v2.1/gl"
	"github.com/go-gl/glfw/v3.2/glfw"
)

var (
	program, vao, vbo uint32
)

// 8 bytes
type v2f struct {
	x, y float32
}

// 16 bytes
type FColor struct {
	R, G, B, A float32
}

// 8*2 + 16*2 bytes = 48 bytes
type Vertex struct {
	vertex, aa_coord          v2f
	fill_color, outline_color FColor
}

type Triangle struct {
	a, b, c Vertex
}

var triangleStack []Triangle = []Triangle{{
	Vertex{v2f{-.5, -.5}, v2f{0, 0}, FColor{R: 1, A: 1}, FColor{R: 1, A: 1}},
	Vertex{v2f{0, .5}, v2f{0, 0}, FColor{R: 1, A: 1}, FColor{R: 1, A: 1}},
	Vertex{v2f{.5, .5}, v2f{0, 0}, FColor{R: 1, A: 1}, FColor{R: 1, A: 1}},
}}

//var triangleStack []Triangle = []Triangle{{
//	Vertex{v2f{100.0, 187.54}, v2f{0, 0}, FColor{R: 1, A: 1}, FColor{R: 1, A: 1}},
//	Vertex{v2f{100.0, 216}, v2f{0, 0}, FColor{R: 1, A: 1}, FColor{R: 1, A: 1}},
//	Vertex{v2f{-100.0, 216.5}, v2f{0, 0}, FColor{R: 1, A: 1}, FColor{R: 1, A: 1}},
//}}

func main() {
	if err := glfw.Init(); err != nil {
		panic(err)
	}
	defer glfw.Terminate()

	//glfw.WindowHint(glfw.Resizable, glfw.False)
	glfw.WindowHint(glfw.ContextVersionMajor, 2)
	glfw.WindowHint(glfw.ContextVersionMinor, 1)
	window, err := glfw.CreateWindow(600, 600, "Cube", nil, nil)
	if err != nil {
		panic(err)
	}
	window.MakeContextCurrent()
	glfw.SwapInterval(1)

	if err := gl.Init(); err != nil {
		panic(err)
	}

	version := gl.GoStr(gl.GetString(gl.VERSION))
	log.Println("OpenGL version", version)

	vshader := CompileShader(gl.VERTEX_SHADER, vertexShader)
	fshader := CompileShader(gl.FRAGMENT_SHADER, fragmentShader)

	program = LinkProgram(vshader, fshader)

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

	gl.BindVertexArrayAPPLE(0)

	CheckGLErrors()

	gl.ClearColor(0, 0, 0, 1)

	gl.Enable(gl.LINE_SMOOTH)
	gl.Enable(gl.POINT_SMOOTH)

	gl.Hint(gl.LINE_SMOOTH_HINT, gl.DONT_CARE)
	gl.Hint(gl.POINT_SMOOTH_HINT, gl.DONT_CARE)

	gl.Enable(gl.BLEND)
	gl.BlendFunc(gl.ONE, gl.ONE_MINUS_SRC_ALPHA)

	window.SetCharCallback(func(w *glfw.Window, char rune) {
		if char == 'q' {
			window.SetShouldClose(true)
			return
		}
	})

	window.SetFramebufferSizeCallback(func(w *glfw.Window, width int, height int) {
		log.Println("Framebuffer size callback", width, height)
		gl.Viewport(0, 0, int32(width), int32(height))
	})

	gl.Viewport(0, 0, 600, 600)

	for !window.ShouldClose() {
		gl.Clear(gl.COLOR_BUFFER_BIT)
		Display()
		window.SwapBuffers()
		glfw.PollEvents()
	}
}

func Display() {
	gl.MatrixMode(gl.MODELVIEW)
	gl.LoadIdentity()
	gl.Translatef(0.0, 0.0, 0.0)
	gl.Scalef(1, 1, 1)

	CheckGLErrors()
	//ClearRenderer()

	// FLUSH
	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
	gl.BufferData(gl.ARRAY_BUFFER, len(triangleStack)*(48*3), gl.Ptr(triangleStack), gl.STREAM_DRAW)

	gl.UseProgram(program)
	gl.Uniform1f(gl.GetUniformLocation(program, gl.Str("u_outline_coef\x00")), 1)

	// TODO
	gl.BindVertexArrayAPPLE(vao)

	gl.DrawArrays(gl.TRIANGLES, 0, int32(len(triangleStack)*3))

	CheckGLErrors()
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

func CheckGLErrors() {
	for err := gl.GetError(); err != 0; err = gl.GetError() {
		switch err {
		case gl.NO_ERROR:
			// ok
		case gl.INVALID_ENUM:
			panic("Invalid enum")
		case gl.INVALID_VALUE:
			panic("Invalid value")
		case gl.INVALID_OPERATION:
			panic("Invalid operation")
		case gl.INVALID_FRAMEBUFFER_OPERATION:
			panic("Invalid Framebuffer Operation")
		case gl.OUT_OF_MEMORY:
			panic("Out of memory")
		}
	}
}

func CheckError(obj uint32, status uint32, getiv func(uint32, uint32, *int32), getInfoLog func(uint32, int32, *int32, *uint8)) bool {
	var success int32
	getiv(obj, status, &success)

	if success == gl.FALSE {
		var length int32
		getiv(obj, gl.INFO_LOG_LENGTH, &length)

		log := strings.Repeat("\x00", int(length+1))
		getInfoLog(obj, length, nil, gl.Str(log))

		fmt.Fprintln(os.Stderr, "GL Error:", log)
		return true
	}

	return false
}

func CompileShader(typ uint32, source string) uint32 {
	shader := gl.CreateShader(typ)

	sources, free := gl.Strs(source + "\x00")
	defer free()
	gl.ShaderSource(shader, 1, sources, nil)
	gl.CompileShader(shader)

	if CheckError(shader, gl.COMPILE_STATUS, gl.GetShaderiv, gl.GetShaderInfoLog) {
		panic("Error compiling shader")
	}

	return shader
}

func LinkProgram(vshader, fshader uint32) uint32 {
	program := gl.CreateProgram()

	gl.AttachShader(program, vshader)
	gl.AttachShader(program, fshader)

	gl.LinkProgram(program)

	if CheckError(program, gl.LINK_STATUS, gl.GetProgramiv, gl.GetProgramInfoLog) {
		panic("Error linking shader program")
	}

	return program
}

func SetAttribute(program uint32, name string, size int32, gltype uint32, stride int32, offset int) {
	var index uint32 = uint32(gl.GetAttribLocation(program, gl.Str(name+"\x00")))
	gl.EnableVertexAttribArray(index)
	gl.VertexAttribPointer(index, size, gltype, false, stride, gl.PtrOffset(offset))
	CheckGLErrors()
}
