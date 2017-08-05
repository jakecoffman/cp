package physics

type Shaper interface {
	CacheData(*Shape, Transform) BB
	Destroy(*Shape)
	PointQuery(*Shape, Vector, *PointQueryInfo)
	SegmentQuery(*Shape, Vector, Vector, float64, *SegmentQueryInfo)
}

type Shape struct {
	space    *Space
	body     *Body
	massInfo ShapeMassInfo
	bb       BB

	sensor   bool
	e, u     float64
	surfaceV Vector

	userData interface{}

	collisionType uint
	filter        ShapeFilter

	next, prev *Shape

	hashid int
}

func NewShape(body *body) *Shape {
	return &Shape {

	}
}
