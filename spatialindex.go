package cp

type SpatialIndexBB func(obj *Shape) BB
type SpatialIndexIterator func(obj *Shape)
type SpatialIndexQuery func(obj1 any, obj2 *Shape, collisionId uint32, data any) uint32
type SpatialIndexSegmentQuery func(obj1 any, obj2 *Shape, data any) float64

// SpatialIndexer implemented by BBTree
type SpatialIndexer interface {
	Count() int
	Each(f SpatialIndexIterator)
	Contains(obj *Shape, hashId HashValue) bool
	Insert(obj *Shape, hashId HashValue)
	Remove(obj *Shape, hashId HashValue)
	Reindex()
	ReindexObject(obj *Shape, hashId HashValue)
	ReindexQuery(f SpatialIndexQuery, data any)
	Query(obj any, bb BB, f SpatialIndexQuery, data any)
	SegmentQuery(obj any, a, b Vector, t_exit float64, f SpatialIndexSegmentQuery, data any)
}

func ShapeGetBB(obj *Shape) BB {
	return obj.bb
}

type SpatialIndex struct {
	class                     SpatialIndexer
	bbfunc                    SpatialIndexBB
	staticIndex, dynamicIndex *SpatialIndex
}

func NewSpatialIndex(klass SpatialIndexer, bbfunc SpatialIndexBB, staticIndex *SpatialIndex) *SpatialIndex {
	index := &SpatialIndex{
		class:       klass,
		bbfunc:      bbfunc,
		staticIndex: staticIndex,
	}

	if staticIndex != nil {
		staticIndex.dynamicIndex = index
	}

	return index
}

func (index *SpatialIndex) GetTree() *BBTree {
	if index == nil {
		return nil
	}
	return index.class.(*BBTree)
}

func (index *SpatialIndex) GetRootIfTree() *Node {
	if index == nil {
		return nil
	}
	return index.class.(*BBTree).root
}

func (dynamicIndex *SpatialIndex) CollideStatic(staticIndex *SpatialIndex, f SpatialIndexQuery, data any) {
	if staticIndex != nil && staticIndex.class.Count() > 0 {
		dynamicIndex.class.Each(func(obj *Shape) {
			staticIndex.class.Query(obj, dynamicIndex.bbfunc(obj), f, data)
		})
	}
}
