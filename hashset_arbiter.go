package cp

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

	size       uint
	table      []*HashSetBinArbiter
	pooledBins *HashSetBinArbiter
}

func NewHashSetArbiter(eql HashSetEqualArbiter) *HashSetArbiter {
	size := nextPrime(0)
	return &HashSetArbiter{
		eql:   eql,
		size:  size,
		table: make([]*HashSetBinArbiter, size),
	}
}

func (set *HashSetArbiter) Resize() {
	newSize := nextPrime(set.size + 1)
	newTable := make([]*HashSetBinArbiter, newSize)

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

func (set *HashSetArbiter) Free() {
	if set != nil {
		set.table = []*HashSetBinArbiter{}
	}
}

func (set *HashSetArbiter) Count() uint {
	return set.entries
}

func (set *HashSetArbiter) Insert(hash HashValue, ptr []*Shape, trans HashSetTransArbiter, space *Space) *Arbiter {
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
		bin.elt = trans(ptr, space)

		bin.next = set.table[idx]
		set.table[idx] = bin

		set.entries++
		if set.entries >= set.size {
			set.Resize()
		}
	}

	return bin.elt
}

func (set *HashSetArbiter) InsertArb(hash HashValue, ptr []*Shape, arb *Arbiter) interface{} {
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
		bin.elt = arb

		bin.next = set.table[idx]
		set.table[idx] = bin

		set.entries++
		if set.entries >= set.size {
			set.Resize()
		}
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
	idx := uint(hash) % set.size
	prevPtr := &set.table[idx]
	bin := set.table[idx]

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

func (set *HashSetArbiter) Find(hash HashValue, ptr []*Shape) interface{} {
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

func (set *HashSetArbiter) Each(f HashSetIteratorArbiter) {
	for _, bin := range set.table {
		for bin != nil {
			next := bin.next
			f(bin.elt)
			bin = next
		}
	}
}

func (set *HashSetArbiter) Filter(filter func(arb *Arbiter) bool) {
	var i uint
	for i = 0; i < set.size; i++ {
		prevPtr := &set.table[i]
		bin := set.table[i]
		for bin != nil {
			next := bin.next

			if filter(bin.elt) {
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

// Hashset filter func to throw away old arbiters.
func SpaceArbiterSetFilter(arb *Arbiter, space *Space) bool {
	// TODO: should make an arbiter state for this so it doesn't require filtering arbiters for dangling body pointers on body removal.
	// Preserve arbiters on sensors and rejected arbiters for sleeping objects.
	// This prevents errant separate callbacks from happening.

	a := arb.body_a
	b := arb.body_b

	if (a.GetType() == BODY_STATIC || a.IsSleeping()) && (b.GetType() == BODY_STATIC || b.IsSleeping()) {
		return true
	}

	ticks := space.stamp - arb.stamp

	if ticks >= 1 && arb.state != CP_ARBITER_STATE_CACHED {
		arb.state = CP_ARBITER_STATE_CACHED
		handler := arb.handler
		handler.SeparateFunc(arb, space, handler.UserData)
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

func CachedArbitersFilter(arb *Arbiter, space *Space, shape *Shape, body *Body) bool {
	// Match on the filter shape, or if it's NULL the filter body
	if (body == arb.body_a && (shape == arb.a || shape == nil)) ||
		(body == arb.body_b && (shape == arb.b || shape == nil)) {
		// Call separate when removing shapes.
		if shape != nil && arb.state != CP_ARBITER_STATE_CACHED {
			// Invalidate the arbiter since one of the shapes was removed
			arb.state = CP_ARBITER_STATE_INVALIDATED

			handler := arb.handler
			handler.SeparateFunc(arb, space, handler.UserData)
		}

		arb.Unthread()
		for i, arbiter := range space.arbiters {
			if arb == arbiter {
				space.arbiters = append(space.arbiters[:i], space.arbiters[i+1:]...)
				break
			}
		}
		select {
		case space.pooledArbiters <- arb:
		default:
		}
		return false
	}

	return true
}
