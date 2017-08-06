package physics

type Shaper interface {
	Body() *Body
	MassInfo() *ShapeMassInfo
	HashId() int
	SetHashId(int)
	SetSpace(*Space)
	//CacheData(*Shape, Transform) BB
	//Destroy(*Shape)
	//PointQuery(*Shape, Vector, *PointQueryInfo)
	//SegmentQuery(*Shape, Vector, Vector, float64, *SegmentQueryInfo)
}

type Shape struct {
	space    *Space
	body     *Body
	massInfo *ShapeMassInfo
	bb       *BB

	sensor   bool
	E, U     float64
	surfaceV *Vector

	userData interface{}

	collisionType uint
	Filter        *ShapeFilter

	next, prev *Shape

	hashid int
}

func (s *Shape) Body() *Body {
	return s.body
}

func (s *Shape) MassInfo() *ShapeMassInfo {
	return s.massInfo
}

func (s *Shape) HashId() int {
	return s.hashid
}

func (s *Shape) SetHashId(hashid int) {
	s.hashid = hashid
}

func (s *Shape) SetSpace(space *Space) {
	s.space = space
}

func NewShape(body *Body, massInfo *ShapeMassInfo) *Shape {
	return &Shape {
		body: body,
		massInfo: massInfo,

		surfaceV: VectorZero(),
		Filter:   &ShapeFilter{},
	}
}
