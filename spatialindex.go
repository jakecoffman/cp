package physics

type SpatialIndexBB func(obj interface{}) *BB
type SpatialIndexIterator func(obj interface{}, data interface{})
type SpatialIndexQuery func(obj1, obj2 interface{}, collisionId uint, data interface{}) uint
type SpatialIndexSegmentQuery func(obj1, obj2, data interface{}) float64

// implemented by BBTree
type SpatialIndexer interface {
	Destroy()
	Count() int
	Each(f SpatialIndexIterator, data interface{})
	Contains(obj interface{}, hashId HashValue) bool
	Insert(obj interface{}, hashId HashValue)
	Remove(obj interface{}, hashId HashValue)
	Reindex()
	ReindexObject(obj interface{}, hashId HashValue)
	ReindexQuery(f SpatialIndexQuery, data interface{})
	Query(obj interface{}, bb *BB, f SpatialIndexQuery, data interface{})
	SegmentQuery(obj interface{}, a, b *Vector, t_exit float64, f SpatialIndexSegmentQuery, data interface{})
}

func ShapeGetBB(obj interface{}) *BB {
	return obj.(*Shape).bb
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

var DyanamicToStaticIter = func(obj interface{}, context interface{}) {
	dtsc := context.(*DynamicToStaticContext)
	dtsc.staticIndex.class.Query(obj, dtsc.bbfunc(obj), dtsc.queryFunc, dtsc.data)
}

func (dynamicIndex *SpatialIndex) CollideStatic(staticIndex *SpatialIndex, f SpatialIndexQuery, data interface{}) {
	if staticIndex != nil && staticIndex.class.Count() > 0 {
		context := &DynamicToStaticContext{dynamicIndex.bbfunc, staticIndex, f, data}
		dynamicIndex.class.Each(DyanamicToStaticIter, context)
	}
}
