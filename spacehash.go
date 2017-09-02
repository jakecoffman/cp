package physics

import "math"

type SpaceHash struct {
	*SpatialIndex

	numCells int
	celldim  float64

	table     []*SpaceHashBin
	handleSet *HashSet

	pooledBins    *SpaceHashBin
	pooledHandles []*Handle

	stamp uint
}

func NewSpaceHash(celldim float64, cell int, bbfunc SpatialIndexBB, staticIndex *SpatialIndex) *SpatialIndex {
	spaceHash := &SpaceHash{
		celldim: celldim,
		handleSet: NewHashSet(func(obj, elt interface{}) bool {
			return obj == elt.(*Handle).obj
		}),
		stamp: 1,
	}
	return NewSpatialIndex(spaceHash, bbfunc, staticIndex)
}

func (hash *SpaceHash) hashHandle(hand *Handle, bb *BB) {
	dim := hash.celldim

	// TODO: chipmunk said floor is slow, use custom floor
	//l := math.Floor(bb.L / dim)
	r := math.Floor(bb.R / dim)
	b := math.Floor(bb.B / dim)
	t := math.Floor(bb.T / dim)

	n := hash.numCells
	for i := 1.0; i <= r; i++ {
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

func (*SpaceHash) Count() int {
	panic("implement me")
}

func (hash *SpaceHash) Each(f SpatialIndexIterator, data interface{}) {
	hash.handleSet.Each(func(elt, _ interface{}) {
		f(elt.(*Handle).obj, data)
	}, nil)
}

func (*SpaceHash) Contains(obj interface{}, hashId HashValue) bool {
	panic("implement me")
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
	hash.handleSet.Each(func(elt, _ interface{}) {
		hand := elt.(*Handle)
		hash.bbfunc(hand.obj)
	}, nil)
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

}

func (*SpaceHash) ReindexQuery(f SpatialIndexQuery, data interface{}) {
	panic("implement me")
}

func (*SpaceHash) Query(obj interface{}, bb *BB, f SpatialIndexQuery, data interface{}) {
	panic("implement me")
}

func (*SpaceHash) SegmentQuery(obj interface{}, a, b *Vector, t_exit float64, f SpatialIndexSegmentQuery, data interface{}) {
	panic("implement me")
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

func (hand *Handle) Init(obj interface{}) *Handle {
	return &Handle{
		obj:     obj,
		retains: 0,
		stamp:   0,
	}
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
