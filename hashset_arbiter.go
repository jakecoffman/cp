package physics

type HashSetEqualArbiter func(ptr []*Shape, elt *Arbiter) bool
type HashSetTransArbiter func(ptr []*Shape, space *Space) *Arbiter
type HashSetIteratorArbiter func(elt *Arbiter)
type HashSetFilterArbiter func(arb *Arbiter, space *Space) bool

type HashSetBinArbiter struct {
	elt  *Arbiter
	hash HashValue
	next *HashSetBinArbiter
}

type HashSetArbiter struct {
	// number of bins in the table, not just table size
	entries      uint
	eql          HashSetEqualArbiter
	defaultValue Arbiter

	table      map[HashValue]*HashSetBinArbiter
	pooledBins *HashSetBinArbiter
}

func NewHashSetArbiter(eql HashSetEqualArbiter) *HashSetArbiter {
	return &HashSetArbiter{
		eql:   eql,
		table: map[HashValue]*HashSetBinArbiter{},
	}
}

func (set *HashSetArbiter) Free() {
	if set != nil {
		set.table = map[HashValue]*HashSetBinArbiter{}
	}
}

func (set *HashSetArbiter) Count() uint {
	return set.entries
}

func (set *HashSetArbiter) Insert(hash HashValue, ptr []*Shape, trans HashSetTransArbiter, space *Space) interface{} {
	// Find the bin with the matching element.
	bin := set.table[hash]
	for bin != nil && !set.eql(ptr, bin.elt) {
		bin = bin.next
	}

	// Create it if necessary.
	if bin == nil {
		bin = set.GetUnusedBin()
		bin.hash = hash
		bin.elt = trans(ptr, space)

		bin.next = set.table[hash]
		set.table[hash] = bin

		set.entries++
	}

	return bin.elt
}

func (set *HashSetArbiter) InsertArb(hash HashValue, ptr []*Shape, arb *Arbiter) interface{} {
	// Find the bin with the matching element.
	bin := set.table[hash]
	for bin != nil && !set.eql(ptr, bin.elt) {
		bin = bin.next
	}

	// Create it if necessary.
	if bin == nil {
		bin = &HashSetBinArbiter{}
		bin.hash = hash
		bin.elt = arb

		bin.next = set.table[hash]
		set.table[hash] = bin

		set.entries++
	}

	return bin.elt
}

func (set *HashSetArbiter) Recycle(bin *HashSetBinArbiter) {
	bin.next = set.pooledBins
	set.pooledBins = bin
	bin.elt = nil
}

func (set *HashSetArbiter) GetUnusedBin() *HashSetBinArbiter {
	bin := set.pooledBins

	if bin != nil {
		set.pooledBins = bin.next
		return bin
	}

	for i := 0; i < POOLED_BUFFER_SIZE; i++ {
		set.Recycle(&HashSetBinArbiter{})
	}

	return &HashSetBinArbiter{}
}

func (set *HashSetArbiter) Remove(hash HashValue, ptr []*Shape) *Arbiter {
	bin := set.table[hash]
	// In Go we can't take the address of a map entry, so this differs a bit.
	var prevPtr **HashSetBinArbiter

	// Find the bin
	for bin != nil && !set.eql(ptr, bin.elt) {
		prevPtr = &bin.next
		bin = bin.next
	}

	// Remove the bin if it exists
	if bin != nil {
		// Update the previous linked list pointer
		if prevPtr != nil {
			*prevPtr = bin.next
		} else {
			delete(set.table, hash)
		}

		set.entries--

		elt := bin.elt
		set.Recycle(bin)

		return elt
	}

	return nil
}

func (set *HashSetArbiter) Find(hash HashValue, ptr []*Shape) interface{} {
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

func (set *HashSetArbiter) Each(f HashSetIteratorArbiter) {
	for _, bin := range set.table {
		for bin != nil {
			next := bin.next
			f(bin.elt)
			bin = next
		}
	}
}

func (set *HashSetArbiter) Filter(space *Space) {
	var prevPtr **HashSetBinArbiter
	var next *HashSetBinArbiter

	for i, first := range set.table {
		prevPtr = &first
		bin := first
		for bin != nil {
			next = bin.next

			if SpaceArbiterSetFilter(bin.elt, space) {
				prevPtr = &bin.next
			} else {
				if first == *prevPtr {
					set.table[i] = nil
				} else {
					*prevPtr = bin.next
				}

				set.entries--

				set.Recycle(bin)
			}

			bin = next
		}
	}
}

// Hashset filter func to throw away old arbiters.
func SpaceArbiterSetFilter(arb *Arbiter, space *Space) bool {
	// TODO: should make an arbiter state for this so it doesn't require filtering arbiters for dangling body pointers on body removal.
	// Preserve arbiters on sensors and rejected arbiters for sleeping objects.
	// This prevents errant separate callbacks from happening.

	if (arb.body_a.sleepingRoot != nil || arb.body_a.sleepingIdleTime == INFINITY) &&
		(arb.body_b.sleepingRoot != nil || arb.body_b.sleepingIdleTime == INFINITY) {
		return true
	}

	ticks := space.stamp - arb.stamp

	if ticks >= 1 && arb.state != CP_ARBITER_STATE_CACHED {
		arb.state = CP_ARBITER_STATE_CACHED
		handler := arb.handler
		handler.separateFunc(arb, space, handler.userData)
	}

	if ticks >= space.collisionPersistence {
		arb.contacts = nil
		arb.count = 0
		select {
		case space.pooledArbiters <- arb:
		default:
		}
		return false
	}

	return true
}
