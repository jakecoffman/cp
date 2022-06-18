package cp

type HashValue uintptr

// HashSetBin implements a linked list
type HashSetBin struct {
	elt  *Node
	hash HashValue
	next *HashSetBin
}

// HashSet implements a hash set
type HashSet struct {
	// number of bins in the table, not just table size
	entries uint
	isEqual func(ptr *Shape, elt *Node) bool

	size       uint
	table      []*HashSetBin
	pooledBins *HashSetBin
}

// NewHashSet is a HashSet constructor
func NewHashSet(isEqual func(ptr *Shape, elt *Node) bool) *HashSet {
	size := nextPrime(0)
	return &HashSet{
		isEqual: isEqual,
		size:    size,
		table:   make([]*HashSetBin, size),
	}
}

func (set *HashSet) resize() {
	newSize := nextPrime(set.size + 1)
	newTable := make([]*HashSetBin, newSize)

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

// Count returns the number of entries
func (set *HashSet) Count() uint {
	return set.entries
}

// Insert returns the Node the Shape is in, or inserts a new Node and returns it.
func (set *HashSet) Insert(hash HashValue, shape *Shape, transform func(obj *Shape, tree *BBTree) *Node, tree *BBTree) *Node {
	idx := uint(hash) % set.size

	// Find the bin with the matching element.
	bin := set.table[idx]
	for bin != nil && !set.isEqual(shape, bin.elt) {
		bin = bin.next
	}

	if bin != nil {
		return bin.elt
	}

	// Create it if necessary.
	bin = set.getUnusedBin()
	bin.hash = hash
	bin.elt = transform(shape, tree)

	bin.next = set.table[idx]
	set.table[idx] = bin

	set.entries++
	if set.entries >= set.size {
		set.resize()
	}

	return bin.elt
}

// Remove removes the Shape from the HashSet, returning the Node it was in.
func (set *HashSet) Remove(hash HashValue, ptr *Shape) *Node {
	idx := uint(hash) % set.size

	bin := set.table[idx]
	prevPtr := &set.table[idx]

	// Find the bin
	for bin != nil && !set.isEqual(ptr, bin.elt) {
		prevPtr = &bin.next
		bin = bin.next
	}

	// Remove the bin if it exists
	if bin != nil {
		// Update the previous linked list pointer
		*prevPtr = bin.next
		set.entries--

		elt := bin.elt
		set.recycle(bin)

		return elt
	}

	return nil
}

// Find returns the Node the Shape is in, or nil.
func (set *HashSet) Find(hash HashValue, ptr *Shape) *Node {
	idx := uint(hash) % set.size
	bin := set.table[idx]
	for bin != nil && !set.isEqual(ptr, bin.elt) {
		bin = bin.next
	}

	if bin != nil {
		return bin.elt
	}
	return nil
}

// Each calls f for every Node in the HashSet.
func (set *HashSet) Each(f func(elt *Node)) {
	for _, bin := range set.table {
		for bin != nil {
			next := bin.next
			f(bin.elt)
			bin = next
		}
	}
}

// Filter removes elements if f returns false
func (set *HashSet) Filter(f func(*Node) bool) {
	var i uint
	for i = 0; i < set.size; i++ {
		prevPtr := &set.table[i]
		bin := set.table[i]
		for bin != nil {
			next := bin.next

			if f(bin.elt) {
				prevPtr = &bin.next
			} else {
				*prevPtr = next

				set.entries--
				set.recycle(bin)
			}

			bin = next
		}
	}
}

func (set *HashSet) recycle(bin *HashSetBin) {
	bin.next = set.pooledBins
	set.pooledBins = bin
	bin.elt = nil
}

func (set *HashSet) getUnusedBin() *HashSetBin {
	bin := set.pooledBins

	if bin != nil {
		set.pooledBins = bin.next
		return bin
	}

	for i := 0; i < POOLED_BUFFER_SIZE; i++ {
		set.recycle(&HashSetBin{})
	}

	return &HashSetBin{}
}

var primes = []uint{
	5,
	13,
	23,
	47,
	97,
	193,
	389,
	769,
	1543,
	3079,
	6151,
	12289,
	24593,
	49157,
	98317,
	196613,
	393241,
	786433,
	1572869,
	3145739,
	6291469,
	12582917,
	25165843,
	50331653,
	100663319,
	201326611,
	402653189,
	805306457,
	1610612741,
	0,
}

func nextPrime(n uint) uint {
	var i uint = 0
	for n > primes[i] {
		i++
	}
	return primes[i]
}
