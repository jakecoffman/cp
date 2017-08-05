package physics

type Transform struct {
	a, b, c, d, tx, ty float64
}

func TransformNewTranspose(a, c, tx, b, d, ty float64) *Transform {
	return &Transform{a, b, c, d, tx, ty}
}

func TransformPoint(t *Transform, p *Vector) *Vector {
	return &Vector{X: t.a*p.X + t.c*p.Y + t.tx, Y: t.b*p.X + t.d*p.Y + t.ty}
}

func (t *Transform) Vect(v *Vector) *Vector {
	return &Vector{t.a*v.X + t.c*v.Y, t.b*v.X + t.d*v.Y}
}
