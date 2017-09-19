package cp

type HashValue uintptr
type HashSetEqual func(ptr, elt interface{}) bool
type HashSetTrans func(ptr, data interface{}) interface{}
type HashSetIterator func(elt interface{})
type HashSetFilter func(elt, data interface{}) bool

type HashSetBin struct {
	elt  interface{}
	hash HashValue
	next *HashSetBin
}

type HashSet struct {
	// number of bins in the table, not just table size
	entries      uint
	eql          HashSetEqual
	defaultValue interface{}

	size       uint
	table      []*HashSetBin
	pooledBins *HashSetBin
}

func NewHashSet(eql HashSetEqual) *HashSet {
	size := nextPrime(0)
	return &HashSet{
		eql:   eql,
		size:  size,
		table: make([]*HashSetBin, size),
	}
}

func (set *HashSet) Resize() {
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

func (set *HashSet) Free() {
	if set != nil {
		set.table = []*HashSetBin{}
	}
}

func (set *HashSet) Count() uint {
	return set.entries
}

func (set *HashSet) Insert(hash HashValue, ptr interface{}, trans HashSetTrans, data interface{}) interface{} {
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
		if trans != nil {
			bin.elt = trans(ptr, data)
		} else {
			bin.elt = data
		}

		bin.next = set.table[idx]
		set.table[idx] = bin

		set.entries++
		if set.entries >= set.size {
			set.Resize()
		}
	}

	return bin.elt
}

func (set *HashSet) Remove(hash HashValue, ptr interface{}) interface{} {
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

func (set *HashSet) Find(hash HashValue, ptr interface{}) interface{} {
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

func (set *HashSet) Each(f HashSetIterator) {
	for _, bin := range set.table {
		for bin != nil {
			next := bin.next
			f(bin.elt)
			bin = next
		}
	}
}

func (set *HashSet) Filter(f HashSetFilter, data interface{}) {
	var i uint
	for i = 0; i < set.size; i++ {
		prevPtr := &set.table[i]
		bin := set.table[i]
		for bin != nil {
			next := bin.next

			if f(bin.elt, data) {
				prevPtr = &bin.next
			} else {
				*prevPtr = next

				set.entries--
				set.Recycle(bin)
			}

			bin = next
		}
	}
}

func (set *HashSet) Recycle(bin *HashSetBin) {
	bin.next = set.pooledBins
	set.pooledBins = bin
	bin.elt = nil
}

func (set *HashSet) GetUnusedBin() *HashSetBin {
	bin := set.pooledBins

	if bin != nil {
		set.pooledBins = bin.next
		return bin
	}

	for i := 0; i < POOLED_BUFFER_SIZE; i++ {
		set.Recycle(&HashSetBin{})
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
