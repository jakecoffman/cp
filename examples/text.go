package examples

import (
	"runtime"

	"github.com/go-gl/gl/v2.1/gl"
	. "github.com/jakecoffman/physics"
)

const (
	scale      = 0.7
	lineHeight = 18.0 * scale
)

var textprogram uint32
var texture uint32

// 8*2 + 16 = 32
type TextVertex struct {
	vertex, text_coord v2f
	color              FColor
}

type TextTriangle struct {
	a, b, c TextVertex
}

var textvao uint32
var textvbo uint32

var glyph_indexes = [256]int{}

func TextInit() {
	vshader := CompileShader(gl.VERTEX_SHADER, `
		attribute vec2 vertex;
		attribute vec2 tex_coord;
		attribute vec4 color;

		varying vec2 v_tex_coord;
		varying vec4 v_color;

		void main(void){
			// TODO: get rid of the GL 2.x matrix bit eventually?
			gl_Position = gl_ModelViewProjectionMatrix*vec4(vertex, 0.0, 1.0);

			v_color = color;
			v_tex_coord = tex_coord;
		}
	`)
	fshader := CompileShader(gl.FRAGMENT_SHADER, `
		uniform sampler2D u_texture;

		varying vec2 v_tex_coord;
		varying vec4 v_color;

		float aa_step(float t1, float t2, float f)
		{
			//return step(t2, f);
			return smoothstep(t1, t2, f);
		}

		void main(void)
		{
			float sdf = texture2D(u_texture, v_tex_coord).a;

			//float fw = fwidth(sdf)*0.5;
			float fw = length(vec2(dFdx(sdf), dFdy(sdf)))*0.5;

			float alpha = aa_step(0.5 - fw, 0.5 + fw, sdf);
			gl_FragColor = v_color*(v_color.a*alpha);
//			gl_FragColor = vec4(1, 0, 0, 1);
		}
		`)

	textprogram = LinkProgram(vshader, fshader)
	CheckGLErrors()

	if runtime.GOOS == "darwin" {
		gl.GenVertexArraysAPPLE(1, &textvao)
		gl.BindVertexArrayAPPLE(textvao)
	} else {
		gl.GenVertexArrays(1, &textvao)
		gl.BindVertexArray(textvao)
	}

	gl.GenBuffers(1, &textvbo)
	gl.BindBuffer(gl.ARRAY_BUFFER, textvbo)

	SetAttribute(textprogram, "vertex", 2, gl.FLOAT, 32, 0)
	SetAttribute(textprogram, "tex_coord", 2, gl.FLOAT, 32, 8)
	SetAttribute(textprogram, "color", 4, gl.FLOAT, 32, 16)

	gl.BindBuffer(gl.ARRAY_BUFFER, 0)

	if runtime.GOOS == "darwin" {
		gl.BindVertexArrayAPPLE(0)
	} else {
		gl.BindVertexArray(0)
	}
	CheckGLErrors()

	// Load the SDF font texture.
	gl.GenTextures(1, &texture)
	gl.BindTexture(gl.TEXTURE_2D, texture)
	gl.TexImage2D(gl.TEXTURE_2D, 0, gl.ALPHA, sdf_tex_width, sdf_tex_height, 0, gl.ALPHA, gl.UNSIGNED_BYTE, gl.Ptr(sdf_data))
	gl.GenerateMipmap(gl.TEXTURE_2D)
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR_MIPMAP_LINEAR)
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR)
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP)
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP)
	CheckGLErrors()

	for i := 0; i < sdf_num_chars; i++ {
		idx := sdf_spacing[i*8]
		glyph_indexes[idx] = i
	}
}

var textStack []TextTriangle = []TextTriangle{}

func PushChar(character int32, x, y float64, color FColor) float64 {
	i := glyph_indexes[character]

	gw := float32(sdf_spacing[i*8+3])
	gh := float32(sdf_spacing[i*8+4])

	txmin := float32(sdf_spacing[i*8+1]) / sdf_tex_width
	tymin := float32(sdf_spacing[i*8+2]) / sdf_tex_height
	txmax := txmin + gw/sdf_tex_width
	tymax := tymin + gh/sdf_tex_height

	s := scale / scale_factor
	xmin := float32(x) + float32(sdf_spacing[i*8+5])/scale_factor*scale
	ymin := float32(y) + (float32(sdf_spacing[i*8+6])/scale_factor-gh)*scale
	xmax := xmin + gw*scale
	ymax := ymin + gh*scale

	a := TextVertex{v2f{xmin, ymin}, v2f{txmin, tymax}, color}
	b := TextVertex{v2f{xmin, ymax}, v2f{txmin, tymin}, color}
	c := TextVertex{v2f{xmax, ymax}, v2f{txmax, tymin}, color}
	d := TextVertex{v2f{xmax, ymin}, v2f{txmax, tymax}, color}

	t0 := TextTriangle{a, b, c}
	t1 := TextTriangle{a, c, d}

	textStack = append(textStack, t0)
	textStack = append(textStack, t1)

	return float64(sdf_spacing[i*8+7]) * s
}

func DrawString(pos Vector, str string) {
	c := FColor{1, 1, 1, 1}
	x := pos.X
	y := pos.Y

	for _, character := range str {
		if character == '\n' {
			y -= lineHeight
			x = pos.X
		} else {
			x += PushChar(character, x, y, c)
		}
	}
}

func FlushTextRenderer() {
	gl.BindBuffer(gl.ARRAY_BUFFER, textvbo)
	gl.BufferData(gl.ARRAY_BUFFER, len(textStack)*3*32, gl.Ptr(textStack), gl.STREAM_DRAW)

	gl.UseProgram(textprogram)

	if runtime.GOOS == "darwin" {
		gl.BindVertexArrayAPPLE(textvao)
	} else {
		gl.BindVertexArray(textvao)
	}
	gl.DrawArrays(gl.TRIANGLES, 0, int32(len(textStack)*3))
	CheckGLErrors()
}

func ClearTextRenderer() {
	textStack = textStack[0:0]
}
