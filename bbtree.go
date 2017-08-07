package physics

type Node struct {
	obj interface{}
	bb *BB
	parent *Node
	NodeValue
}

type NodeValue struct {
	// internal node
	a, b *Node
	// or children
	stamp uint
	pairs *Pair
}

type Pair struct {
	a, b Thread
	id int
}

type Thread struct {
	next, prev *Pair
	lead *Node
}

type BBTreeVelocity func(obj interface{})

type BBTree struct {
	*SpatialIndex
	velocityFunc BBTreeVelocity

	leaves map[int]int
	root *Node
	pooledNodes *Node

}