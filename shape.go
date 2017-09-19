package cp

import "fmt"

type Shaper interface {
	Body() *Body
	MassInfo() *ShapeMassInfo
	HashId() HashValue
	SetHashId(HashValue)
	SetSpace(*Space)
	BB() BB
	SetBB(BB)
}

type ShapeClass interface {
	CacheData(transform Transform) BB
	Destroy()
	PointQuery(p Vector, info *PointQueryInfo)
	SegmentQuery(a, b Vector, radius float64, info *SegmentQueryInfo)
}

const (
	SHAPE_TYPE_NUM = 3
)

type Shape struct {
	Class    ShapeClass
	space    *Space
	body     *Body
	massInfo *ShapeMassInfo
	bb       BB

	sensor   bool
	e, u     float64
	surfaceV Vector

	UserData interface{}

	collisionType CollisionType
	Filter        ShapeFilter

	hashid HashValue
}

func (s Shape) String() string {
	return fmt.Sprintf("%T", s.Class)
}

func (s *Shape) Order() int {
	switch s.Class.(type) {
	case *Circle:
		return 0
	case *Segment:
		return 1
	case *PolyShape:
		return 2
	default:
		return 3
	}
}

func (s *Shape) Sensor() bool {
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

func (s *Shape) HashId() HashValue {
	return s.hashid
}

func (s *Shape) SetHashId(hashid HashValue) {
	s.hashid = hashid
}

func (s *Shape) SetSpace(space *Space) {
	s.space = space
}

func (s *Shape) BB() BB {
	return s.bb
}

func (s *Shape) SetBB(bb BB) {
	s.bb = bb
}

func (s *Shape) SetCollisionType(collisionType CollisionType) {
	s.body.Activate()
	s.collisionType = collisionType
}

func (s *Shape) Friction() float64 {
	return s.u
}

func (s *Shape) SetFriction(u float64) {
	assert(s.u >= 0, "Must be positive")
	s.body.Activate()
	s.u = u
}

func (s *Shape) SetSurfaceV(surfaceV Vector) {
	s.surfaceV = surfaceV
}

func (s *Shape) Elasticity() float64 {
	return s.e
}

func (s *Shape) SetElasticity(e float64) {
	assert(s.e >= 0, "Must be positive")
	s.body.Activate()
	s.e = e
}

func (s *Shape) SetFilter(filter ShapeFilter) {
	s.body.Activate()
	s.Filter = filter
}

func (s *Shape) CacheBB() BB {
	return s.Update(s.body.transform)
}

func (s *Shape) Update(transform Transform) BB {
	s.bb = s.Class.CacheData(transform)
	return s.bb
}

func (s *Shape) Point(i uint32) *SupportPoint {
	switch s.Class.(type) {
	case *Circle:
		return NewSupportPoint(s.Class.(*Circle).tc, 0)
	case *Segment:
		seg := s.Class.(*Segment)
		if i == 0 {
			return NewSupportPoint(seg.ta, i)
		}
		return NewSupportPoint(seg.tb, i)
	case *PolyShape:
		poly := s.Class.(*PolyShape)
		// Poly shapes may change vertex count.
		var index int
		if i < uint32(poly.count) {
			index = int(i)
		}
		return NewSupportPoint(poly.planes[index].v0, uint32(index))
	default:
		return NewSupportPoint(Vector{}, 0)
	}
}

func (s *Shape) PointQuery(p Vector) PointQueryInfo {
	info := PointQueryInfo{nil, Vector{}, INFINITY, Vector{}}
	s.Class.PointQuery(p, &info)
	return info
}

func (shape *Shape) SegmentQuery(a, b Vector, radius float64, info *SegmentQueryInfo) bool {
	blank := SegmentQueryInfo{nil, b, Vector{}, 1}
	if info != nil {
		*info = blank
	} else {
		info = &blank
	}

	var nearest PointQueryInfo
	shape.Class.PointQuery(a, &nearest)
	if nearest.Distance <= radius {
		info.Shape = shape
		info.Alpha = 0
		info.Normal = a.Sub(nearest.Point).Normalize()
	} else {
		shape.Class.SegmentQuery(a, b, radius, info)
	}

	return info.Shape != nil
}

func NewShape(class ShapeClass, body *Body, massInfo *ShapeMassInfo) *Shape {
	return &Shape{
		Class:    class,
		body:     body,
		massInfo: massInfo,

		surfaceV: Vector{},
		Filter: ShapeFilter{
			Group:      NO_GROUP,
			Categories: ALL_CATEGORIES,
			Mask:       ALL_CATEGORIES,
		},
	}
}
