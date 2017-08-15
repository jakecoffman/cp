package physics

type HashValue uintptr
type HashSetEqual func(ptr, elt interface{}) bool
type HashSetTrans func(ptr, data interface{}) interface{}
type HashSetIterator func(elt, data interface{})
type HashSetFilter func(elt, data interface{}) bool

type HashSetBin struct {
	elt  interface{} // number of bins in the table, not just table size
	hash HashValue
	next *HashSetBin
}

type HashSet struct {
	entries      uint
	eql          HashSetEqual
	defaultValue interface{}

	table map[HashValue]*HashSetBin
}

func NewHashSet(eql HashSetEqual) *HashSet {
	return &HashSet{
		eql:   eql,
		table: map[HashValue]*HashSetBin{},
	}
}

func (set *HashSet) Free() {
	if set != nil {
		set.table = map[HashValue]*HashSetBin{}
	}
}

func (set *HashSet) Count() uint {
	return set.entries
}

func (set *HashSet) Insert(hash HashValue, ptr interface{}, trans HashSetTrans, data interface{}) interface{} {
	// Find the bin with the matching element.
	bin := set.table[hash]
	for bin != nil && !set.eql(ptr, bin.elt) {
		bin = bin.next
	}

	// Create it if necessary.
	if bin == nil {
		bin = &HashSetBin{}
		bin.hash = hash
		if trans != nil {
			bin.elt = trans(ptr, data)
		} else {
			bin.elt = data
		}

		bin.next = set.table[hash]
		set.table[hash] = bin

		set.entries++
	}

	return bin.elt
}

func (set *HashSet) Remove(hash HashValue, ptr interface{}) interface{} {
	bin := set.table[hash]
	prevPtr := &bin

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
		return bin.elt
	}

	return nil
}

func (set *HashSet) Find(hash HashValue, ptr interface{}) interface{} {
	bin := set.table[hash]
	for bin != nil && !set.eql(ptr, bin.elt) {
		bin = bin.next
	}

	if bin != nil {
		return bin.elt
	} else {
		return set.defaultValue
	}
}

func (set *HashSet) Each(f HashSetIterator, data interface{}) {
	for _, bin := range set.table {
		for bin != nil {
			next := bin.next
			f(bin.elt, data)
			bin = next
		}
	}
}

func (set *HashSet) Filter(f HashSetFilter, data interface{}) {
	for _, bin := range set.table {
		prevPtr := &bin
		for bin != nil {
			next := bin.next

			if f(bin.elt, data) {
				prevPtr = &bin.next
			} else {
				*prevPtr = next

				set.entries--
			}

			bin = next
		}
	}
}
