package examples

import (
	"fmt"
	"os"
	"strings"

	"github.com/go-gl/gl/v2.1/gl"
)

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
	p := gl.CreateProgram()

	gl.AttachShader(p, vshader)
	gl.AttachShader(p, fshader)

	gl.LinkProgram(p)

	if CheckError(p, gl.LINK_STATUS, gl.GetProgramiv, gl.GetProgramInfoLog) {
		panic("Error linking shader program")
	}

	return p
}

func SetAttribute(program uint32, name string, size int32, gltype uint32, stride int32, offset int) {
	var index uint32 = uint32(gl.GetAttribLocation(program, gl.Str(name+"\x00")))
	gl.EnableVertexAttribArray(index)
	gl.VertexAttribPointer(index, size, gltype, false, stride, gl.PtrOffset(offset))
	CheckGLErrors()
}
