package physics

type Shaper interface {
	Body() *Body
	MassInfo() *ShapeMassInfo
	HashId() uint
	SetHashId(uint)
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

// Shape type enum
const (
	SHAPE_TYPE_CIRCLE = iota
	SHAPE_TYPE_SEGMENT
	SHAPE_TYPE_POLY
	SHAPE_TYPE_NUM // total number of shapes
)

type Shape struct {
	class     ShapeClass
	shapeType int // SHAPE_TYPE used for sorting in Collision
	space     *Space
	body      *Body
	massInfo  *ShapeMassInfo
	bb        *BB

	sensor   bool
	E, U     float64
	surfaceV *Vector

	userData interface{}

	collisionType uint
	Filter        *ShapeFilter

	next, prev *Shape

	hashid uint
}

func (s *Shape) GetSensor() bool {
	return s.sensor
}

func (s *Shape) SetSensor(sensor bool) {
	s.body.Activate()
	s.sensor = sensor
}

func (s *Shape) Space() *Space {
	return s.space
}

func (s *Shape) Body() *Body {
	return s.body
}

func (s *Shape) MassInfo() *ShapeMassInfo {
	return s.massInfo
}

func (s *Shape) HashId() uint {
	return s.hashid
}

func (s *Shape) SetHashId(hashid uint) {
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
	return s.bb
}

func (s *Shape) Point(i uint) *SupportPoint {
	switch s.shapeType {
	case SHAPE_TYPE_CIRCLE:
		return NewSupportPoint(s.class.(*Circle).tc, 0)
	case SHAPE_TYPE_SEGMENT:
		seg := s.class.(*Segment)
		if i == 0 {
			return NewSupportPoint(seg.ta, i)
		}
		return NewSupportPoint(seg.tb, i)
	case SHAPE_TYPE_POLY:
		poly := s.class.(*PolyShape)
		// Poly shapes may change vertex count.
		var index uint
		if i < poly.count {
			index = i
		} else {
			index = 0
		}
		return NewSupportPoint(poly.planes[index].v0, index)
	default:
		return NewSupportPoint(VectorZero(), 0)
	}
}

func NewShape(class ShapeClass, shapeType int, body *Body, massInfo *ShapeMassInfo) *Shape {
	return &Shape{
		class:    class,
		body:     body,
		massInfo: massInfo,

		surfaceV: VectorZero(),
		Filter:   &ShapeFilter{},
	}
}
