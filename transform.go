package cp

import "math"

type Transform struct {
	a, b, c, d, tx, ty float64
}

func NewTransformIdentity() Transform {
	return Transform{1, 0, 0, 1, 0, 0}
}

func NewTransform(a, c, tx, b, d, ty float64) Transform {
	return Transform{a, b, c, d, tx, ty}
}

func NewTransformTranspose(a, c, tx, b, d, ty float64) Transform {
	return Transform{a, b, c, d, tx, ty}
}

func NewTransformTranslate(translate Vector) Transform {
	return NewTransformTranspose(
		1, 0, translate.X,
		0, 1, translate.Y,
	)
}

func NewTransformScale(scaleX, scaleY float64) Transform {
	return NewTransformTranspose(
		scaleX, 0, 0,
		0, scaleY, 0,
	)
}

func NewTransformRotate(radians float64) Transform {
	rot := ForAngle(radians)
	return NewTransformTranspose(
		rot.X, -rot.Y, 0,
		rot.Y, rot.X, 0,
	)
}

func NewTransformRigid(translate Vector, radians float64) Transform {
	rot := ForAngle(radians)
	return NewTransformTranspose(
		rot.X, -rot.Y, translate.X,
		rot.Y, rot.X, translate.Y,
	)
}

func NewTransformRigidInverse(t Transform) Transform {
	return NewTransformTranspose(
		t.d, -t.c, t.c*t.ty-t.tx*t.d,
		-t.b, t.a, t.tx*t.b-t.a*t.ty,
	)
}

func (t Transform) Inverse() Transform {
	inv_det := 1.0 / (t.a*t.d - t.c*t.b)
	return NewTransformTranspose(
		t.d*inv_det, -t.c*inv_det, (t.c*t.ty-t.tx*t.d)*inv_det,
		-t.b*inv_det, t.a*inv_det, (t.tx*t.b-t.a*t.ty)*inv_det,
	)
}

func (t Transform) Mult(t2 Transform) Transform {
	return NewTransformTranspose(
		t.a*t2.a+t.c*t2.b, t.a*t2.c+t.c*t2.d, t.a*t2.tx+t.c*t2.ty+t.tx,
		t.b*t2.a+t.d*t2.b, t.b*t2.c+t.d*t2.d, t.b*t2.tx+t.d*t2.ty+t.ty,
	)
}

func (t Transform) Point(p Vector) Vector {
	return Vector{X: t.a*p.X + t.c*p.Y + t.tx, Y: t.b*p.X + t.d*p.Y + t.ty}
}

func (t Transform) Vect(v Vector) Vector {
	return Vector{t.a*v.X + t.c*v.Y, t.b*v.X + t.d*v.Y}
}

func (t Transform) BB(bb BB) BB {
	hw := (bb.R - bb.L) * 0.5
	hh := (bb.T - bb.B) * 0.5

	a := t.a * hw
	b := t.c * hh
	d := t.b * hw
	e := t.d * hh
	hw_max := math.Max(math.Abs(a+b), math.Abs(a-b))
	hh_max := math.Max(math.Abs(d+e), math.Abs(d-e))
	return NewBBForExtents(t.Point(bb.Center()), hw_max, hh_max)
}

// miscellaneous transform matrices

func (outer Transform) Wrap(inner Transform) Transform {
	return outer.Inverse().Mult(inner.Mult(outer))
}

func (outer Transform) Ortho(bb BB) Transform {
	return NewTransformTranspose(
		2.0/(bb.R-bb.L), 0.0, -(bb.R +bb.L)/(bb.R-bb.L),
		0.0, 2.0/(bb.T-bb.B), -(bb.T +bb.B)/(bb.T-bb.B),
	)
}

func (outer Transform) BoneScale(v0, v1 Vector) Transform {
	d := v1.Sub(v0)
	return NewTransformTranspose(
		d.X, -d.Y, v0.X,
		d.Y, d.X, v0.Y,
	)
}

func (outer Transform) AxialScale(axis, pivot Vector, scale float64) Transform {
	A := axis.X * axis.Y * (scale - 1.0)
	B := axis.Dot(pivot) * (1.0 - scale)

	return NewTransformTranspose(
		scale*axis.X*axis.X+axis.Y*axis.Y, A, axis.X*B,
		A, axis.X*axis.X+scale*axis.Y*axis.Y, axis.Y*B,
	)
}
