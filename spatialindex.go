package physics

type SpatialIndexBB func(obj *Shape) BB
type SpatialIndexIterator func(obj *Shape, data interface{})
type SpatialIndexQuery func(obj1 interface{}, obj2 *Shape, collisionId uint32, data interface{}) uint32
type SpatialIndexSegmentQuery func(obj1 interface{}, obj2 *Shape, data interface{}) float64

// implemented by BBTree
type SpatialIndexer interface {
	Destroy()
	Count() int
	Each(f SpatialIndexIterator, data interface{})
	Contains(obj *Shape, hashId HashValue) bool
	Insert(obj *Shape, hashId HashValue)
	Remove(obj *Shape, hashId HashValue)
	Reindex()
	ReindexObject(obj *Shape, hashId HashValue)
	ReindexQuery(f SpatialIndexQuery, data interface{})
	Query(obj interface{}, bb BB, f SpatialIndexQuery, data interface{})
	SegmentQuery(obj interface{}, a, b Vector, t_exit float64, f SpatialIndexSegmentQuery, data interface{})
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

type DynamicToStaticContext struct {
	bbfunc      SpatialIndexBB
	staticIndex *SpatialIndex
	queryFunc   SpatialIndexQuery
	data        interface{}
}

var DyanamicToStaticIter = func(obj *Shape, context interface{}) {
	dtsc := context.(*DynamicToStaticContext)
	dtsc.staticIndex.class.Query(obj, dtsc.bbfunc(obj), dtsc.queryFunc, dtsc.data)
}

func (dynamicIndex *SpatialIndex) CollideStatic(staticIndex *SpatialIndex, f SpatialIndexQuery, data interface{}) {
	if staticIndex != nil && staticIndex.class.Count() > 0 {
		context := &DynamicToStaticContext{dynamicIndex.bbfunc, staticIndex, f, data}
		dynamicIndex.class.Each(DyanamicToStaticIter, context)
	}
}
