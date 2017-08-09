package physics

type Shaper interface {
	Body() *Body
	MassInfo() *ShapeMassInfo
	HashId() int
	SetHashId(int)
	SetSpace(*Space)
	SetBB(*BB)
	//CacheData(*Shape, Transform) BB
	//Destroy(*Shape)
	//PointQuery(*Shape, Vector, *PointQueryInfo)
	//SegmentQuery(*Shape, Vector, Vector, float64, *SegmentQueryInfo)
}

type ShapeClass interface {
	CacheData(transform *Transform) *BB
	Destroy()
	PointQuery(p Vector, info *PointQueryInfo)
	SegmentQuery(a, b Vector, radius float64, info *SegmentQueryInfo)
}

type Shape struct {
	class    ShapeClass
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

func (s *Shape) SetBB(bb *BB) {
	s.bb = bb
}

func (s *Shape) CacheBB() *BB {
	return s.Update(s.body.transform)
}

func (s *Shape) Update(transform *Transform) *BB {
	s.bb = s.class.CacheData(transform)
}

func NewShape(class ShapeClass, body *Body, massInfo *ShapeMassInfo) *Shape {
	return &Shape{
		class:    class,
		body:     body,
		massInfo: massInfo,

		surfaceV: VectorZero(),
		Filter:   &ShapeFilter{},
	}
}
