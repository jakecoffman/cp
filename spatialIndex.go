package physics

type SpatialIndexBB func(obj interface{}) *BB
type SpatialIndexIterator func (obj, data interface{})
type SpatialIndexQuery func (obj1, obj2 interface{}, collisionId int, data interface{}) int
type SpatialIndexSegmentQuery func (obj1, obj2, data interface{}) float64

type SpatialIndexer interface {
	Destroy()
	Count()
	Each(iterator SpatialIndexIterator, data interface{})
	Contains(obj interface{}, hashid int)
	Insert(obj interface{}, hashid int)
	Remove(obj interface{}, hashid int)
	Reindex()
	ReindexObject(obj interface{}, hashid int)
	ReindexQuery(query SpatialIndexQuery, data interface{})
	Query(obj interface{}, bb *BB, query SpatialIndexQuery, data interface{})
	SegmentQuery(obj interface{}, a, b *Vector, t_exit float64, query SpatialIndexSegmentQuery, data interface{})
}

type BBFunc func(obj interface{}) *BB

type SpatialIndex struct {
	bbfunc BBFunc
	staticIndex, dynamicIndex *SpatialIndex
}
