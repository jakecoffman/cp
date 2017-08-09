package physics

import "math"

type BBTreeVelocityFunc func(obj interface{}) *Vector

type Node struct {
	obj    interface{}
	bb     *BB
	parent *Node

	children *Children
	leaf     *Leaf
}

type Children struct {
	a, b *Node
}

type Leaf struct {
	stamp uint
	pairs *Pair
}

type Pair struct {
	a, b        *Thread
	collisionId int
}

type Thread struct {
	prev, next *Pair
	leaf       *Node
}

type BBTree struct {
	spatialIndex *SpatialIndex
	velocityFunc BBTreeVelocityFunc

	leaves map[int]*Node
	root   *Node

	pooledNodes      *Node
	pooledPairs      *Pair
	allocatedBuffers *ContactBuffer

	stamp uint
}

func (tree *BBTree) Destroy() {
	panic("implement me")
}

func (tree *BBTree) Count() int {
	panic("implement me")
}

func (tree *BBTree) Each(f SpatialIndexIterator, data interface{}) {
	panic("implement me")
}

func (tree *BBTree) Contains(obj interface{}, hashId int) {
	panic("implement me")
}

func (tree *BBTree) Insert(obj interface{}, hashId int) {
	panic("IMP")
}

func (tree *BBTree) Remove(obj interface{}, hashId int) {
	panic("implement me")
}

func (tree *BBTree) Reindex() {
	panic("implement me")
}

func (tree *BBTree) ReindexObject(obj interface{}, hashId int) {
	panic("implement me")
}

func (tree *BBTree) ReindexQuery(f SpatialIndexIterator, data interface{}) {
	panic("implement me")
}

func (tree *BBTree) Query(obj interface{}, bb *BB, f SpatialIndexQuery, data interface{}) {
	panic("implement me")
}

func (tree *BBTree) SegmentQuery(obj interface{}, a, b *Vector, t_exit float64, f SpatialIndexSegmentQuery, data interface{}) {
	panic("implement me")
}

func (tree *BBTree) GetBB(obj interface{}) *BB {
	bb := tree.spatialIndex.bbfunc(obj)
	if tree.velocityFunc {
		coef := 0.1
		x := (bb.r - bb.l) * coef
		y := (bb.t - bb.b) * coef

		v := tree.velocityFunc(obj).Mult(0.1)
		return &BB{
			bb.l + math.Min(-x, v.X),
			bb.b + math.Min(-y, v.Y),
			bb.r + math.Max(x, v.X),
			bb.t + cpfmax(y, v.Y),
		}
	}

	return bb
}

func cpfmax(x, y float64) float64 {
	return math.Max(x, y)
}

func cpfmin(x, y float64) float64 {
	return math.Min(x, y)
}

func (tree *BBTree) GetMasterTree() *BBTree {
	dynamicTree := tree.spatialIndex.dynamicIndex.GetTree()
	if dynamicTree {
		return dynamicTree
	}
	return tree
}
