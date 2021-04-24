package cp

import "math"

type PolyLine struct {
	Verts []Vector
}

type PolyLineSet struct {
	Lines []*PolyLine
}

func Next(i, count int) int {
	return (i + 1) % count
}

func Sharpness(a, b, c Vector) float64 {
	return a.Sub(b).Normalize().Dot(c.Sub(b).Normalize())
}

func (pl *PolyLine) Push(v Vector) *PolyLine {
	pl.Verts = append(pl.Verts, v)
	return pl
}

func (pl *PolyLine) Enqueue(v Vector) *PolyLine {
	pl.Verts = append([]Vector{v}, pl.Verts...)
	return pl
}

func (pl *PolyLine) IsClosed() bool {
	return len(pl.Verts) > 1 && pl.Verts[0].Equal(pl.Verts[len(pl.Verts)-1])
}

func (pl *PolyLine) IsShort(count, start, end int, min float64) bool {
	var length float64
	for i := start; i != end; Next(i, count) {
		length += pl.Verts[i].Distance(pl.Verts[Next(i, count)])
		if length > min {
			return false
		}
	}
	return true
}

// Join similar adjacent line segments together. Works well for hard edged shapes.
// 'tol' is the minimum anglular difference in radians of a vertex.
func (pl *PolyLine) SimplifyVertexes(tol float64) *PolyLine {
	reduced := PolyLine{Verts: []Vector{pl.Verts[0], pl.Verts[1]}}
	minSharp := -math.Cos(tol)

	for i := 2; i < len(pl.Verts); i++ {
		vert := pl.Verts[i]
		sharp := Sharpness(reduced.Verts[len(reduced.Verts)-2], reduced.Verts[len(reduced.Verts)-1], vert)
		if sharp <= minSharp {
			reduced.Verts[len(reduced.Verts)-1] = vert
		} else {
			reduced.Push(vert)
		}
	}

	if pl.IsClosed() && Sharpness(reduced.Verts[len(reduced.Verts)-2], reduced.Verts[0], reduced.Verts[1]) < minSharp {
		reduced.Verts[0] = reduced.Verts[len(reduced.Verts)-2]
	}
	return &reduced
}

// Recursive function used by cpPolylineSimplifyCurves().
func DouglasPeucker(verts []Vector, reduced *PolyLine, length, start, end int, min, tol float64) *PolyLine {
	// Early exit if the points are adjacent
	if (end-start+length)%length < 2 {
		return reduced
	}

	a := verts[start]
	b := verts[end]

	// Check if the length is below the threshold
	if a.Near(b, min) && reduced.IsShort(length, start, end, min) {
		return reduced
	}

	// Find the maximal vertex to split and recurse on
	var max float64
	maxi := start

	n := b.Sub(a).Perp().Normalize()
	d := n.Dot(a)

	for i := Next(start, length); i != end; i = Next(i, length) {
		dist := math.Abs(n.Dot(verts[i]) - d)

		if dist > max {
			max = dist
			maxi = i
		}
	}

	if max > tol {
		reduced = DouglasPeucker(verts, reduced, length, start, maxi, min, tol)
		reduced.Push(verts[maxi])
		reduced = DouglasPeucker(verts, reduced, length, maxi, end, min, tol)
	}

	return reduced
}

// Recursively reduce the vertex count on a polyline. Works best for smooth shapes.
// 'tol' is the maximum error for the reduction.
// The reduced polyline will never be farther than this distance from the original polyline.
func (pl *PolyLine) SimplifyCurves(tol float64) *PolyLine {
	reduced := &PolyLine{}

	min := tol / 2.0

	if pl.IsClosed() {
		start, end := LoopIndexes(pl.Verts, len(pl.Verts)-1)

		reduced = reduced.Push(pl.Verts[start])
		reduced = DouglasPeucker(pl.Verts, reduced, len(pl.Verts)-1, start, end, min, tol)
		reduced = reduced.Push(pl.Verts[end])
		reduced = DouglasPeucker(pl.Verts, reduced, len(pl.Verts)-1, end, start, min, tol)
		reduced = reduced.Push(pl.Verts[start])
	} else {
		reduced = reduced.Push(pl.Verts[0])
		reduced = DouglasPeucker(pl.Verts, reduced, len(pl.Verts), 0, len(pl.Verts)-1, min, tol)
		reduced = reduced.Push(pl.Verts[len(pl.Verts)-1])
	}

	return reduced
}

// Find the polyline that ends with v.
func (pls *PolyLineSet) FindEnds(v Vector) int {
	for i, line := range pls.Lines {
		if line.Verts != nil && line.Verts[len(line.Verts)-1].Equal(v) {
			return i
		}
	}
	return -1
}

// Find the polyline that starts with v.
func (pls *PolyLineSet) FindStarts(v Vector) int {
	for i, line := range pls.Lines {
		if line.Verts != nil && line.Verts[0].Equal(v) {
			return i
		}
	}
	return -1
}

// Add a new polyline to a polyline set.
func (pls *PolyLineSet) Push(v *PolyLine) {
	pls.Lines = append(pls.Lines, v)
}

// Join two cpPolylines in a polyline set together.

// this may deletion could be slow? https://yourbasic.org/golang/delete-element-slice/
func (pls *PolyLineSet) Join(before, after int) {
	pls.Lines[before].Verts = append(pls.Lines[before].Verts, pls.Lines[after].Verts...)
	copy(pls.Lines[after:], pls.Lines[after+1:])
	pls.Lines = pls.Lines[:len(pls.Lines)-1]
}

// Add a segment to a polyline set.
// A segment will either start a new polyline, join two others, or add to or loop an existing polyline.
func PolyLineCollectSegment(v0, v1 Vector, pls *PolyLineSet) {

	before := pls.FindEnds(v0)
	after := pls.FindStarts(v1)

	if before >= 0 && after >= 0 {
		if before == after {
			pls.Lines[before].Push(v1)
		} else {
			pls.Join(before, after)
		}
	} else if before >= 0 {
		pls.Lines[before].Push(v1)
	} else if after >= 0 {
		pls.Lines[after].Enqueue(v0)
	} else {
		pls.Push(&PolyLine{Verts: []Vector{v0, v1}})
	}
}
