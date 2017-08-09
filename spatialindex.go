package physics

type SpatialIndexBB func(obj interface{}) *BB
type SpatialIndexIterator func(obj interface{}, data interface{})
type SpatialIndexQuery func(obj1, obj2 interface{}, collisionId int, data interface{})
type SpatialIndexSegmentQuery func()

type SpatialIndexer interface {
	Destroy()
	Count() int
	Each(f SpatialIndexIterator, data interface{})
	Contains(obj interface{}, hashId int)
	Insert(obj interface{}, hashId int)
	Remove(obj interface{}, hashId int)
	Reindex()
	ReindexObject(obj interface{}, hashId int)
	ReindexQuery(f SpatialIndexIterator, data interface{})
	Query(obj interface{}, bb *BB, f SpatialIndexQuery, data interface{})
	SegmentQuery(obj interface{}, a, b *Vector, t_exit float64, f SpatialIndexSegmentQuery, data interface{})
}

type SpatialIndex struct {
	klass                     SpatialIndexer
	bbfunc                    SpatialIndexBB
	staticIndex, dynamicIndex *SpatialIndex
}

func NewSpatialIndex(klass SpatialIndexer, bbfunc SpatialIndexBB, staticIndex *SpatialIndex) *SpatialIndex {
	index := &SpatialIndex{
		klass:       klass,
		bbfunc:      bbfunc,
		staticIndex: staticIndex,
	}

	if staticIndex {
		staticIndex.dynamicIndex = index
	}

	return index
}

func (index *SpatialIndex) GetTree() *BBTree {
	return index.klass.(*BBTree)
}

func (index *SpatialIndex) GetRootIfTree() *Node {
	return index.klass.(*BBTree).root
}

type DynamicToStaticContext struct {
	bbfunc      SpatialIndexBB
	staticIndex *SpatialIndex
	queryFunc   SpatialIndexQuery
	data        interface{}
}

var DyanamicToStaticIter = func(obj interface{}, context interface{}) {
	dtsc := context.(*DynamicToStaticContext)
	dtsc.staticIndex.klass.Query(obj, dtsc.bbfunc(obj), dtsc.queryFunc, dtsc.data)
}

func CollideStatic(dynamicIndex, staticIndex *SpatialIndex, f SpatialIndexQuery, data interface{}) {
	if staticIndex != nil && staticIndex.klass.Count() > 0 {
		context := &DynamicToStaticContext{dynamicIndex.bbfunc, staticIndex, f, data}
		dynamicIndex.klass.Each(DyanamicToStaticIter, context)
	}
}
