package physics

type Circle struct {
	shape *Shape
	c, tc *Vector
	r     float64
}

func (*Circle) CacheData(transform *Transform) *BB {
	panic("implement me")
}

func (*Circle) Destroy() {
	panic("implement me")
}

func (*Circle) PointQuery(p Vector, info *PointQueryInfo) {
	panic("implement me")
}

func (*Circle) SegmentQuery(a, b Vector, radius float64, info *SegmentQueryInfo) {
	panic("implement me")
}

