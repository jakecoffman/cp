package cp

type HashSetEqualCollisionHandler func(ptr, elt interface{}) bool
type HashSetTransCollisionHandler func(ptr, data interface{}) interface{}
type HashSetIteratorCollisionHandler func(elt interface{})
type HashSetFilterCollisionHandler func(elt, data interface{}) bool

type HashSetBinCollisionHandler struct {
	elt  *CollisionHandler
	hash HashValue
	next *HashSetBinCollisionHandler
}

type HashSetCollisionHandler struct {
	// number of bins in the table, not just table size
	entries      uint
	defaultValue *CollisionHandler

	size       uint
	table      []*HashSetBinCollisionHandler
	pooledBins *HashSetBinCollisionHandler
}

func NewHashSetCollisionHandler() *HashSetCollisionHandler {
	size := nextPrime(0)
	return &HashSetCollisionHandler{
		size:  size,
		table: make([]*HashSetBinCollisionHandler, size),
	}
}

func (set *HashSetCollisionHandler) Resize() {
	newSize := nextPrime(set.size + 1)
	newTable := make([]*HashSetBinCollisionHandler, newSize)

	var i uint
	for i = 0; i < set.size; i++ {
		bin := set.table[i]
		for bin != nil {
			next := bin.next
			idx := uint(bin.hash) % newSize
			bin.next = newTable[idx]
			newTable[idx] = bin
			bin = next
		}
	}

	set.table = newTable
	set.size = newSize
}

func (set *HashSetCollisionHandler) Count() uint {
	return set.entries
}

func handlerSetEql(check, pair *CollisionHandler) bool {
	if check.TypeA == pair.TypeA && check.TypeB == pair.TypeB {
		return true
	}
	if check.TypeB == pair.TypeA && check.TypeA == pair.TypeB {
		return true
	}
	return false
}

func (set *HashSetCollisionHandler) Insert(hash HashValue, ptr *CollisionHandler) *CollisionHandler {
	idx := uint(hash) % set.size

	// Find the bin with the matching element.
	bin := set.table[idx]
	for bin != nil && !handlerSetEql(ptr, bin.elt) {
		bin = bin.next
	}

	// Create it if necessary.
	if bin == nil {
		bin = &HashSetBinCollisionHandler{}
		bin.hash = hash
		bin.elt = ptr

		bin.next = set.table[idx]
		set.table[idx] = bin

		set.entries++
		if set.entries >= set.size {
			set.Resize()
		}
	}

	return bin.elt
}

func (set *HashSetCollisionHandler) Find(hash HashValue, ptr *CollisionHandler) *CollisionHandler {
	idx := uint(hash) % set.size
	bin := set.table[idx]
	for bin != nil && !handlerSetEql(ptr, bin.elt) {
		bin = bin.next
	}

	if bin != nil {
		return bin.elt
	} else {
		return set.defaultValue
	}
}

func (set *HashSetCollisionHandler) Each(f HashSetIterator) {
	for _, bin := range set.table {
		for bin != nil {
			next := bin.next
			f(bin.elt)
			bin = next
		}
	}
}
