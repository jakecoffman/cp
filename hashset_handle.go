package cp

type HashSetEqualHandle func(ptr *Shape, elt *Handle) bool
type HashSetTransHandle func(ptr *Shape, hash *SpaceHash) *Handle
type HashSetIteratorHandle func(elt *Handle)
type HashSetFilterHandle func(arb *Handle, space *Space) bool

type HashSetBinHandle struct {
	elt  *Handle
	hash HashValue
	next *HashSetBinHandle
}

type HashSetHandle struct {
	// number of bins in the table, not just table size
	entries      uint
	eql          HashSetEqualHandle
	defaultValue Handle

	size       uint
	table      []*HashSetBinHandle
	pooledBins *HashSetBinHandle
}

func NewHashSetHandle(eql HashSetEqualHandle) *HashSetHandle {
	size := nextPrime(0)
	return &HashSetHandle{
		eql:   eql,
		size:  size,
		table: make([]*HashSetBinHandle, size),
	}
}

func (set *HashSetHandle) Resize() {
	newSize := nextPrime(set.size + 1)
	newTable := make([]*HashSetBinHandle, newSize)

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

func (set *HashSetHandle) Free() {
	if set != nil {
		set.table = []*HashSetBinHandle{}
	}
}

func (set *HashSetHandle) Count() uint {
	return set.entries
}

func (set *HashSetHandle) Insert(hash HashValue, ptr *Shape, trans HashSetTransHandle, spaceHash *SpaceHash) *Handle {
	idx := uint(hash) % set.size

	// Find the bin with the matching element.
	bin := set.table[idx]
	for bin != nil && !set.eql(ptr, bin.elt) {
		bin = bin.next
	}

	// Create it if necessary.
	if bin == nil {
		bin = set.GetUnusedBin()
		bin.hash = hash
		bin.elt = trans(ptr, spaceHash)

		bin.next = set.table[idx]
		set.table[idx] = bin

		set.entries++
		if set.entries >= set.size {
			set.Resize()
		}
	}

	return bin.elt
}

func (set *HashSetHandle) InsertArb(hash HashValue, ptr *Shape, arb *Handle) interface{} {
	idx := uint(hash) % set.size

	// Find the bin with the matching element.
	bin := set.table[idx]
	for bin != nil && !set.eql(ptr, bin.elt) {
		bin = bin.next
	}

	// Create it if necessary.
	if bin == nil {
		bin = &HashSetBinHandle{}
		bin.hash = hash
		bin.elt = arb

		bin.next = set.table[idx]
		set.table[idx] = bin

		set.entries++
	}

	return bin.elt
}

func (set *HashSetHandle) Recycle(bin *HashSetBinHandle) {
	bin.next = set.pooledBins
	set.pooledBins = bin
	bin.elt = nil
}

func (set *HashSetHandle) GetUnusedBin() *HashSetBinHandle {
	bin := set.pooledBins

	if bin != nil {
		set.pooledBins = bin.next
		return bin
	}

	for i := 0; i < POOLED_BUFFER_SIZE; i++ {
		set.Recycle(&HashSetBinHandle{})
	}

	return &HashSetBinHandle{}
}

func (set *HashSetHandle) Remove(hash HashValue, ptr *Shape) *Handle {
	idx := uint(hash) % set.size
	bin := set.table[idx]
	prevPtr := &set.table[idx]

	// Find the bin
	for bin != nil && !set.eql(ptr, bin.elt) {
		prevPtr = &bin.next
		bin = bin.next
	}

	// Remove the bin if it exists
	if bin != nil {
		// Update the previous linked list pointer
		*prevPtr = bin.next
		set.entries--

		elt := bin.elt
		set.Recycle(bin)

		return elt
	}

	return nil
}

func (set *HashSetHandle) Find(hash HashValue, ptr *Shape) interface{} {
	idx := uint(hash) % set.size
	bin := set.table[idx]
	for bin != nil && !set.eql(ptr, bin.elt) {
		bin = bin.next
	}

	if bin != nil {
		return bin.elt
	} else {
		return set.defaultValue
	}
}

func (set *HashSetHandle) Each(f HashSetIteratorHandle) {
	var next *HashSetBinHandle
	for _, bin := range set.table {
		for bin != nil {
			next = bin.next
			f(bin.elt)
			bin = next
		}
	}
}
