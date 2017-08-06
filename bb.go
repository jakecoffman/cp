package physics

type BB struct{
	l, b, r, t float64
}

func NewBBForExtents(c *Vector, hw, hh float64) *BB {
	return &BB{
		l: c.X - hw,
		b: c.Y - hh,
		r: c.X + hw,
		t: c.Y + hh,
	}
}

func (bb *BB) Center() *Vector {
	return Vector{bb.l, bb.b}.Lerp(&Vector{bb.r, bb.t}, 0.5)
}
