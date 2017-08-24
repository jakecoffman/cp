package examples

import (
	"math"

	"github.com/go-gl/gl/v2.1/gl"
	. "github.com/jakecoffman/physics"
)

const (
	DrawPointLineScale = 1
	DrawOutlineWidth   = 1
)

var program uint32

// 8 bytes
type v2f struct {
	x, y float32
}

func V2f(v *Vector) v2f {
	return v2f{float32(v.X), float32(v.Y)}
}
func v2f0() v2f {
	return v2f{0, 0}
}

// 8*2 + 16*2 bytes = 48 bytes
type Vertex struct {
	vertex, aa_coord          v2f
	fill_color, outline_color FColor
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
		}
	`)

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
		}
	`)

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
}

var triangleStack []Triangle = []Triangle{}

func max(a, b int32) int32 {
	if a > b {
		return a
	}
	return b
}

func DrawCircle(pos *Vector, angle, radius float64, outline, fill FColor) {
	r := radius + 1/DrawPointLineScale
	a := Vertex{
		v2f{float32(pos.X - r), float32(pos.Y - r)},
		v2f{-1, 1},
		fill,
		outline,
	}
	b := Vertex{
		v2f{float32(pos.X - r), float32(pos.Y + r)},
		v2f{-1, 1},
		fill,
		outline,
	}
	c := Vertex{
		v2f{float32(pos.X + r), float32(pos.Y + r)},
		v2f{1, 1},
		fill,
		outline,
	}
	d := Vertex{
		v2f{float32(pos.X + r), float32(pos.Y - r)},
		v2f{1, -1},
		fill,
		outline,
	}

	t0 := Triangle{a, b, c}
	t1 := Triangle{a, c, d}

	triangleStack = append(triangleStack, t0)
	triangleStack = append(triangleStack, t1)

	DrawFatSegment(pos, pos.Add(ForAngle(angle).Mult(radius-DrawPointLineScale*0.5)), 0, outline, fill)
}

func DrawSegment(a, b *Vector, fill FColor) {
	DrawFatSegment(a, b, 0, fill, fill)
}

func DrawFatSegment(a, b *Vector, radius float64, outline, fill FColor) {
	n := b.Sub(a).ReversePerp().Normalize()
	t := n.ReversePerp()

	var half float64 = 1 / DrawPointLineScale
	r := radius + half

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

	t0 := Triangle{Vertex{v0, v2f{1, -1}, fill, outline}, Vertex{v1, v2f{1, 1}, fill, outline}, Vertex{v2, v2f{0, -1}, fill, outline}}
	t1 := Triangle{Vertex{v3, v2f{0, 1}, fill, outline}, Vertex{v1, v2f{1, 1}, fill, outline}, Vertex{v2, v2f{0, -1}, fill, outline}}
	t2 := Triangle{Vertex{v3, v2f{0, 1}, fill, outline}, Vertex{v4, v2f{0, -1}, fill, outline}, Vertex{v2, v2f{0, -1}, fill, outline}}
	t3 := Triangle{Vertex{v3, v2f{0, 1}, fill, outline}, Vertex{v4, v2f{0, -1}, fill, outline}, Vertex{v5, v2f{0, 1}, fill, outline}}
	t4 := Triangle{Vertex{v6, v2f{-1, -1}, fill, outline}, Vertex{v4, v2f{0, -1}, fill, outline}, Vertex{v5, v2f{0, 1}, fill, outline}}
	t5 := Triangle{Vertex{v6, v2f{-1, -1}, fill, outline}, Vertex{v7, v2f{-1, 1}, fill, outline}, Vertex{v5, v2f{0, 1}, fill, outline}}

	triangleStack = append(triangleStack, t0)
	triangleStack = append(triangleStack, t1)
	triangleStack = append(triangleStack, t2)
	triangleStack = append(triangleStack, t3)
	triangleStack = append(triangleStack, t4)
	triangleStack = append(triangleStack, t5)
}

func DrawPolygon(count uint, verts []*Vector, radius float64, outline, fill FColor) {
	type ExtrudeVerts struct {
		offset, n Vector
	}
	extrude := make([]ExtrudeVerts, count)

	var i uint
	for i = 0; i < count; i++ {
		v0 := verts[(i-1+count)%count]
		v1 := verts[i]
		v2 := verts[(i+1)%count]

		n1 := v1.Sub(v0).ReversePerp().Normalize()
		n2 := v2.Sub(v1).ReversePerp().Normalize()

		offset := n1.Add(n2).Mult(1 / (n1.Dot(n2) + 1))
		v := ExtrudeVerts{*offset, *n2}
		extrude[i] = v
	}

	cursor := 0

	inset := math.Max(0, 1/DrawPointLineScale-radius)
	for i = 0; i < count-2; i++ {
		v0 := V2f(verts[0].Add(extrude[0].offset.Mult(inset)))
		v1 := V2f(verts[i+1].Add(extrude[i+1].offset.Mult(inset)))
		v2 := V2f(verts[i+2].Add(extrude[i+2].offset.Mult(inset)))

		triangleStack = append(triangleStack, Triangle{
			Vertex{v0, v2f0(), fill, fill},
			Vertex{v1, v2f0(), fill, fill},
			Vertex{v2, v2f0(), fill, fill},
		})
		cursor++
	}

	outset := 1/DrawPointLineScale + radius - inset
	j := count - 1
	for i = 0; i < count; i++ {
		vA := verts[i]
		vB := verts[j]

		nA := extrude[i].n
		nB := extrude[j].n

		offsetA := extrude[i].offset
		offsetB := extrude[j].offset

		innerA := vA.Add(offsetA.Mult(inset))
		innerB := vB.Add(offsetB.Mult(inset))

		inner0 := V2f(innerA)
		inner1 := V2f(innerB)
		outer0 := V2f(innerA.Add(nB.Mult(outset)))
		outer1 := V2f(innerB.Add(nB.Mult(outset)))
		outer2 := V2f(innerA.Add(offsetA.Mult(outset)))
		outer3 := V2f(innerA.Add(nA.Mult(outset)))

		n0 := V2f(&nA)
		n1 := V2f(&nB)
		offset0 := V2f(&offsetA)

		triangleStack = append(triangleStack, Triangle{Vertex{inner0, v2f0(), fill, outline}, Vertex{inner1, v2f0(), fill, outline}, Vertex{outer1, n1, fill, outline}})
		triangleStack = append(triangleStack, Triangle{Vertex{inner0, v2f0(), fill, outline}, Vertex{outer0, n1, fill, outline}, Vertex{outer1, n1, fill, outline}})
		triangleStack = append(triangleStack, Triangle{Vertex{inner0, v2f0(), fill, outline}, Vertex{outer0, n1, fill, outline}, Vertex{outer2, offset0, fill, outline}})
		triangleStack = append(triangleStack, Triangle{Vertex{inner0, v2f0(), fill, outline}, Vertex{outer2, offset0, fill, outline}, Vertex{outer3, n0, fill, outline}})
		cursor += 4

		j = i
	}
}

func DrawDot(size float64, pos *Vector, fill FColor) {
	r := size * 0.5 / DrawPointLineScale
	a := Vertex{v2f{float32(pos.X - r), float32(pos.Y - r)}, v2f{-1, -1}, fill, fill}
	b := Vertex{v2f{float32(pos.X - r), float32(pos.Y + r)}, v2f{-1, 1}, fill, fill}
	c := Vertex{v2f{float32(pos.X + r), float32(pos.Y + r)}, v2f{1, 1}, fill, fill}
	d := Vertex{v2f{float32(pos.X + r), float32(pos.Y - r)}, v2f{1, -1}, fill, fill}

	triangleStack = append(triangleStack, Triangle{a, b, c})
	triangleStack = append(triangleStack, Triangle{a, c, d})
}

func DrawBB(bb *BB, outline FColor) {
	verts := []*Vector{
		{bb.R, bb.B},
		{bb.R, bb.T},
		{bb.L, bb.T},
		{bb.L, bb.B},
	}
	DrawPolygon(4, verts, 0, outline, FColor{A: 1})
}

func FlushRenderer() {
	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
	gl.BufferData(gl.ARRAY_BUFFER, len(triangleStack)*(48*3), gl.Ptr(triangleStack), gl.STREAM_DRAW_ARB)

	gl.UseProgram(program)
	gl.Uniform1f(gl.GetUniformLocation(program, gl.Str("u_outline_coef\x00")), DrawPointLineScale)

	// TODO
	gl.BindVertexArrayAPPLE(vao)

	gl.DrawArrays(gl.TRIANGLES, 0, int32(len(triangleStack)*3))

	CheckGLErrors()
}

func ClearRenderer() {
	triangleStack = triangleStack[0:0]
}
