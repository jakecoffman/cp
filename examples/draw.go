package examples

import (
	"image/color"

	"unsafe"

	"github.com/go-gl/gl/v2.1/gl"
	. "github.com/jakecoffman/physics"
)

const (
	DrawPointLineScale = 1
	DrawOutlineWidth   = 1
)

var program uint32

type v2f struct {
	x, y float32
}

func V2f(v *Vector) v2f {
	return v2f{float32(v.X), float32(v.Y)}
}
func v2f0() v2f {
	return v2f{0, 0}
}

type Vertex struct {
	vertex, aa_coord v2f
	fill, outline    color.Color
}

type Triangle struct {
	a, b, c Vertex
}

var vao uint32 = 0
var vbo uint32 = 0

func DrawInit() {
	vshader := CompileShader(gl.VERTEX_SHADER, `
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
		}`)

	fshader := CompileShader(gl.FRAGMENT_SHADER, `
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
		}`)

	program = LinkProgram(vshader, fshader)

	// TODO implement OS specific stuff in demo_darwin, etc
	gl.GenVertexArraysAPPLE(1, &vao)
	gl.BindVertexArrayAPPLE(vao)

	gl.GenBuffers(1, &vbo)
	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)

	v := Vertex{}
	size := int32(unsafe.Sizeof(v) / unsafe.Sizeof(gl.FLOAT))
	stride := int32(unsafe.Sizeof(v))

	SetAttribute(program, "vertex", size, gl.FLOAT, stride, unsafe.Pointer(unsafe.Offsetof(v.vertex)))
	SetAttribute(program, "aa_coord", size, gl.FLOAT, stride, unsafe.Pointer(unsafe.Offsetof(v.aa_coord)))
	SetAttribute(program, "fill_color", size, gl.FLOAT, stride, unsafe.Pointer(unsafe.Offsetof(v.fill)))
	SetAttribute(program, "outline_color", size, gl.FLOAT, stride, unsafe.Pointer(unsafe.Offsetof(v.outline)))

	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)

	// TODO non-apple
	gl.BindVertexArrayAPPLE(0)

	CheckGLErrors()
}

var triangleStack []*Triangle = []*Triangle{}

func max(a, b int32) int32 {
	if a > b {
		return a
	}
	return b
}

func PushTriangles(count int32) []*Triangle {
	var i int32
	for i = 0; i < count; i++ {
		triangleStack = append(triangleStack, &Triangle{})
	}
	return triangleStack[-count:]
}

func DrawCircle(pos *Vector, angle, radius float64, outline, fill color.Color) {
	triangles := PushTriangles(2)

	r := radius + 1/DrawPointLineScale
	a := Vertex{
		v2f{float32(pos.X - r), float32(pos.Y - r)},
		v2f{-1, 1},
		fill,
		outline,
	}
	b := Vertex{
		v2f{float32(pos.X-r), float32(pos.Y+r)},
		v2f{-1, 1},
		fill,
		outline,
	}
	c := Vertex{
		v2f{float32(pos.X+r), float32(pos.Y+r)},
		v2f{1, 1},
		fill,
		outline,
	}
	d := Vertex{
		v2f{float32(pos.X+r), float32(pos.Y-r)},
		v2f{1, -1},
		fill,
		outline,
	}

	t0 := Triangle{a, b, c}
	t1 := Triangle{a, c, d}

	triangles[0] = &t0
	triangles[1] = &t1

	DrawSegment(pos, pos.Add(ForAngle(angle).Mult(radius - DrawPointLineScale*0.5)), 0, outline, fill)
}
func DrawSegment(a, b *Vector, radius float64, outline, fill color.Color) {
	triangles := PushTriangles(6)

	n := b.Sub(a).Perp().Normalize()
	t := n.Perp()

	var half float64 = 1/DrawPointLineScale
	r := radius+half

	if r <= half {
		r = half
		fill = outline
	}

	nw := n.Mult(r)
	tw := t.Mult(r)
	v0 := V2f(b.Sub(nw.Add(tw)))
	v1 := V2f(b.Add(nw.Add(tw)))
	v2 := V2f(b.Sub(nw))
	v3 := V2f(b.Add(nw))
	v4 := V2f(a.Sub(nw))
	v5 := V2f(a.Add(nw))
	v6 := V2f(a.Sub(nw.Add(tw)))
	v7 := V2f(a.Add(nw.Add(tw)))

	t0 := &Triangle{Vertex{v0, v2f{1, -1}, fill, outline}, Vertex{v1, v2f{1, 1}, fill, outline}, Vertex{v2, v2f{0, -1}, fill, outline}}
	t1 := &Triangle{Vertex{v3, v2f{0, 1}, fill, outline}, Vertex{v1, v2f{1, 1}, fill, outline}, Vertex{v2, v2f{0, -1}, fill, outline}}
	t2 := &Triangle{Vertex{v3, v2f{0, 1}, fill, outline}, Vertex{v4, v2f{0, -1}, fill, outline}, Vertex{v2, v2f{0, -1}, fill, outline}}
	t3 := &Triangle{Vertex{v3, v2f{0, 1}, fill, outline}, Vertex{v4, v2f{0, -1}, fill, outline}, Vertex{v5, v2f{0, 1}, fill, outline}}
	t4 := &Triangle{Vertex{v6, v2f{-1, -1}, fill, outline}, Vertex{v4, v2f{0, -1}, fill, outline}, Vertex{v5, v2f{0, 1}, fill, outline}}
	t5 := &Triangle{Vertex{v6, v2f{-1, -1}, fill, outline}, Vertex{v7, v2f{-1, 1}, fill, outline}, Vertex{v5, v2f{0, 1}, fill, outline}}

	triangles[0] = t0
	triangles[1] = t1
	triangles[2] = t2
	triangles[3] = t3
	triangles[4] = t4
	triangles[5] = t5
}
