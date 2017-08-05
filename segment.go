package physics

type Segment struct {
	Shape

	a, b, n    Vector
	ta, tb, tn Vector
	r          float64

	a_tangent, b_tangent float64
}

func NewSegment() *Segment {

}