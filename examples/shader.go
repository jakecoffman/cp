package examples

import (
	"github.com/go-gl/gl/v2.1/gl"
	"fmt"
	"strings"
	"os"
	"unsafe"
)

func CheckGLErrors() {
	for err := gl.GetError(); err != 0; err = gl.GetError() {
		if err != 0 {
			panic(fmt.Sprint("GL Error ", err))
		}
	}
}

func CheckError(obj uint32, status uint32, getiv func(uint32, uint32, *int32), getInfoLog func(uint32, int32, *int32, *uint8)) bool {
	var success int32
	getiv(obj, status, &success)

	if success != 0 {
		var length int32
		getiv(obj, gl.INFO_LOG_LENGTH, &length)

		log := strings.Repeat("\x00", int(length+1))
		getInfoLog(obj, length, nil, gl.Str(log))

		fmt.Fprintln(os.Stderr, "Shader compile error for ", status, log)
		return false
	}

	return true
}

func CompileShader(typ uint32, source string) uint32 {
	shader := gl.CreateShader(typ)

	gl.ShaderSource(shader, 1, &gl.Str(source), nil)
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

	if CheckError(program, gl.LINK_STATUS, gl.GetProgramiv, gl.GetProgramInfoLog) {
		panic("Error linking shader program")
	}

	return program
}

func SetAttribute(program uint32, name string, size int32, gltype uint32, stride int32, offset unsafe.Pointer) {
	var index uint32 = uint32(gl.GetAttribLocation(program, gl.Str(name)))
	gl.EnableVertexAttribArray(index)
	gl.VertexAttribPointer(index, size, gltype, false, stride, offset)
}
