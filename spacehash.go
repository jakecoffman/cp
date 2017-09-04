package physics

import "math"

type SpaceHash struct {
	*SpatialIndex

	numCells int
	celldim  float64

	table     []*SpaceHashBin
	handleSet *HashSet

	pooledBins *SpaceHashBin
	pooledHandles []*Handle

	stamp uint
}

func NewSpaceHash(celldim float64, num int, bbfunc SpatialIndexBB, staticIndex *SpatialIndex) *SpatialIndex {
	spaceHash := &SpaceHash{
		celldim:  celldim,
		numCells: num,
		table:    make([]*SpaceHashBin, num),
		handleSet: NewHashSet(func(obj, elt interface{}) bool {
			return obj == elt.(*Handle).obj
		}),
		stamp: 1,
	}
	spatialIndex := NewSpatialIndex(spaceHash, bbfunc, staticIndex)
	spaceHash.SpatialIndex = spatialIndex
	return spatialIndex
}

func (hash *SpaceHash) hashHandle(hand *Handle, bb *BB) {
	dim := hash.celldim

	// TODO: chipmunk said floor is slow, use custom floor
	l := math.Floor(bb.L / dim)
	r := math.Floor(bb.R / dim)
	b := math.Floor(bb.B / dim)
	t := math.Floor(bb.T / dim)

	n := hash.numCells
	for i := l; i <= r; i++ {
		for j := b; j <= t; j++ {
			idx := hashFunc(HashValue(i), HashValue(j), HashValue(n))
			bin := hash.table[idx]

			if bin.containsHandle(hand) {
				continue
			}

			hand.retain()
			newBin := hash.getEmptyBin()
			newBin.handle = hand
			newBin.next = bin
			hash.table[idx] = newBin
		}
	}
}

func (hash *SpaceHash) Destroy() {
	panic("implement me")
}

func (hash *SpaceHash) Count() int {
	return int(hash.handleSet.Count())
}

func (hash *SpaceHash) Each(f SpatialIndexIterator, data interface{}) {
	hash.handleSet.Each(func(elt interface{}) {
		f(elt.(*Handle).obj, data)
	})
}

func (hash *SpaceHash) Contains(obj interface{}, hashId HashValue) bool {
	return hash.handleSet.Find(hashId, obj) != nil
}

func (hash *SpaceHash) Insert(obj interface{}, hashId HashValue) {
	hand := hash.handleSet.Insert(hashId, obj, handleSetTrans, hash)
	hash.hashHandle(hand.(*Handle), hash.bbfunc(obj))
}

func (hash *SpaceHash) Remove(obj interface{}, hashId HashValue) {
	hand := hash.handleSet.Remove(hashId, obj).(*Handle)

	if hand != nil {
		hand.obj = nil
		hand.release(hash.pooledHandles)
	}
}

func (hash *SpaceHash) Reindex() {
	hash.clearTable()
	hash.handleSet.Each(func(elt interface{}) {
		hand := elt.(*Handle)
		hash.bbfunc(hand.obj)
	})
}

func (hash *SpaceHash) ReindexObject(obj interface{}, hashId HashValue) {
	hand := hash.handleSet.Remove(hashId, obj).(*Handle)

	if hand != nil {
		hand.obj = nil
		hand.release(hash.pooledHandles)

		hash.Insert(obj, hashId)
	}
}

func (hash *SpaceHash) removeOrphanedHandles(binPtr **SpaceHashBin) {
	bin := *binPtr
	for bin != nil {
		hand := bin.handle
		next := bin.next

		if hand.obj == nil {
			// orphaned handle
			*binPtr = bin.next
			hash.recycleBin(bin)

			hand.release(hash.pooledHandles)
		} else {
			binPtr = &bin.next
		}

		bin = next
	}
}

func (hash *SpaceHash) queryHelper(binPtr **SpaceHashBin, obj interface{}, f SpatialIndexQuery, data interface{}) {
restart:
	for bin := *binPtr; bin != nil; bin = bin.next {
		hand := bin.handle
		other := hand.obj

		if hand.stamp == hash.stamp || obj == other {
			continue
		} else if other != nil {
			f(obj, other, 0, data)
			hand.stamp = hash.stamp
		} else {
			hash.removeOrphanedHandles(binPtr)
			goto restart
		}
	}
}

func (hash *SpaceHash) ReindexQuery(f SpatialIndexQuery, data interface{}) {
	hash.clearTable()

	hash.handleSet.Each(func(elt interface{}) {
		// queryRehash_helper
		hand := elt.(*Handle)

		bb := hash.SpatialIndex.bbfunc(hand.obj)

		l := math.Floor(bb.L / hash.celldim)
		r := math.Floor(bb.R / hash.celldim)
		b := math.Floor(bb.B / hash.celldim)
		t := math.Floor(bb.T / hash.celldim)

		for i := l; i <= r; i++ {
			for j := b; j <= t; j++ {
				idx := hashFunc(HashValue(i), HashValue(j), HashValue(hash.numCells))
				bin := hash.table[idx]

				if bin.containsHandle(hand) {
					continue
				}

				hand.retain()
				hash.queryHelper(&bin, hand.obj, f, data)

				newBin := hash.getEmptyBin()
				newBin.handle = hand
				newBin.next = bin
				hash.table[idx] = newBin
			}
		}

		hash.stamp++
	})

	hash.CollideStatic(hash.staticIndex, f, data)
}

func (hash *SpaceHash) Query(obj interface{}, bb *BB, f SpatialIndexQuery, data interface{}) {
	dim := hash.celldim
	l := math.Floor(bb.L / dim)
	r := math.Floor(bb.R / dim)
	b := math.Floor(bb.B / dim)
	t := math.Floor(bb.T / dim)

	n := hash.numCells

	for i := l; i <= r; i++ {
		for j := b; j <= t; j++ {
			hash.queryHelper(&hash.table[hashFunc(HashValue(i), HashValue(j), HashValue(n))], obj, f, data)
		}
	}

	hash.stamp++
}

func (hash *SpaceHash) segmentQueryHelper(binPtr **SpaceHashBin, obj interface{}, f SpatialIndexSegmentQuery, data interface{}) float64 {
	t := 1.0

restart:
	for bin := *binPtr; bin != nil; bin = bin.next {
		hand := bin.handle
		other := hand.obj

		if hand.stamp == hash.stamp {
			continue
		} else if other != nil {
			t = math.Min(t, f(obj, other, data))
			hand.stamp = hash.stamp
		} else {
			hash.removeOrphanedHandles(binPtr)
			goto restart
		}
	}

	return t
}

// modified from http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
func (hash *SpaceHash) SegmentQuery(obj interface{}, a, b *Vector, t_exit float64, f SpatialIndexSegmentQuery, data interface{}) {
	a = a.Mult(1.0 / hash.celldim)
	b = b.Mult(1.0 / hash.celldim)

	cellX := int(math.Floor(a.X))
	cellY := int(math.Floor(a.Y))

	t := 0.0

	var xInc, yInc int
	var tempV, tempH float64

	if b.X > a.X {
		xInc = 1
		tempH = math.Floor(a.X+1.0) - a.X
	} else {
		xInc = -1
		tempH = a.X - math.Floor(a.X)
	}

	if b.Y > a.Y {
		yInc = 1
		tempV = math.Floor(a.Y+1.0) - a.Y
	} else {
		yInc = -1
		tempV = a.Y - math.Floor(a.Y)
	}

	dx := math.Abs(b.X - a.X)
	dy := math.Abs(b.Y - a.Y)
	var dtdx, dtdy float64
	if dx != 0 {
		dtdx = 1.0 / dx
	} else {
		dtdx = INFINITY
	}

	if dy != 0 {
		dtdy = 1.0 / dy
	} else {
		dtdy = INFINITY
	}

	var nextH, nextV float64
	if tempH != 0 {
		nextH = tempH * dtdx
	} else {
		nextH = dtdx
	}
	if tempV != 0 {
		nextV = tempV * dtdy
	} else {
		nextV = dtdy
	}

	for t < t_exit {
		idx := hashFunc(HashValue(cellX), HashValue(cellY), HashValue(hash.numCells))
		t_exit = math.Min(t_exit, hash.segmentQueryHelper(&hash.table[idx], obj, f, data))

		if nextV < nextH {
			cellY += yInc
			t = nextV
			nextV += dtdy
		} else {
			cellX += xInc
			t = nextH
			nextH += dtdx
		}
	}

	hash.stamp++
}

type SpaceHashBin struct {
	handle *Handle
	next   *SpaceHashBin
}

func (bin *SpaceHashBin) containsHandle(hand *Handle) bool {
	for item := bin; item != nil; item = item.next {
		if item.handle == hand {
			return true
		}
	}

	return false
}

func hashFunc(x, y, n HashValue) HashValue {
	return (x*1640531513 ^ y*2654435789) % n
}

type Handle struct {
	obj     interface{}
	retains int
	stamp   uint
}

func (hand *Handle) Init(obj interface{}) {
	hand.obj = obj
	hand.retains = 0
	hand.stamp = 0
}

func (hand *Handle) retain() {
	hand.retains++
}

func (hand *Handle) release(pooledHandles []*Handle) {
	hand.retains--
	if hand.retains == 0 {
		pooledHandles = append(pooledHandles, hand)
	}
}

func handleSetTrans(obj, elt interface{}) interface{} {
	hash := elt.(*SpaceHash)
	if len(hash.pooledHandles) == 0 {
		for i := 0; i < 256; i++ {
			hash.pooledHandles = append(hash.pooledHandles, &Handle{})
		}
	}
	hand := hash.pooledHandles[0]
	hand.Init(obj)
	hand.retain()
	hash.pooledHandles = hash.pooledHandles[1:]
	return hand
}

func (hash *SpaceHash) recycleBin(bin *SpaceHashBin) {
	bin.next = hash.pooledBins
	hash.pooledBins = bin
}

func (hash *SpaceHash) clearTableCell(idx int) {
	bin := hash.table[idx]
	for bin != nil {
		next := bin.next

		bin.handle.release(hash.pooledHandles)
		hash.recycleBin(bin)

		bin = next
	}

	hash.table[idx] = nil
}

func (hash *SpaceHash) clearTable() {
	for i := 0; i < hash.numCells; i++ {
		hash.clearTableCell(i)
	}
}

func (hash *SpaceHash) getEmptyBin() *SpaceHashBin {
	bin := hash.pooledBins

	if bin != nil {
		hash.pooledBins = bin.next
		return bin
	}

	// pool is exhausted, make more
	for i := 0; i < 256; i++ {
		hash.recycleBin(&SpaceHashBin{})
	}
	return &SpaceHashBin{}
}
