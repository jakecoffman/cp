package cp

// This is a user defined function that gets passed in to the Marching process
// the user establishes a PolyLineSet, passes a pointer to their function, and they
// populate it. In most cases you want to use PolyLineCollectSegment instead of defining your own
type MarchSegmentFunc func(v0 Vector, v1 Vector, segmentData *PolyLineSet)

// This is a user defined function that gets passed every single point from the bounding
// box the user passes into the March process - you can use this to sample an image and
// check for alpha values or really any 2d matrix you define like a tile map.
// NOTE: I could not determine a use case for the sample_data pointer from the original code
// so I removed it here - open to adding it back in if there is a reason.
type MarchSampleFunc func(point Vector) float64

type MarchCellFunc func(t, a, b, c, d, x0, x1, y0, y1 float64, marchSegment MarchSegmentFunc, segmentData *PolyLineSet)

// The looping and sample caching code is shared between cpMarchHard() and cpMarchSoft().
func MarchCells(bb BB, xSamples int64, ySamples int64, t float64, marchSegment MarchSegmentFunc, marchSample MarchSampleFunc, marchCell MarchCellFunc) *PolyLineSet {
	var x_denom, y_denom float64
	x_denom = 1.0 / float64(xSamples-1)
	y_denom = 1.0 / float64(ySamples-1)

	buffer := make([]float64, xSamples)
	var i, j int64
	for i = 0; i < xSamples; i++ {
		buffer[i] = marchSample(Vector{Lerp(bb.L, bb.R, float64(i)*x_denom), bb.B})
	}
	segmentData := &PolyLineSet{}

	for j = 0; j < ySamples-1; j++ {
		y0 := Lerp(bb.B, bb.T, float64(j+0)*y_denom)
		y1 := Lerp(bb.B, bb.T, float64(j+1)*y_denom)

		a := buffer[0]
		b := buffer[0]
		c := marchSample(Vector{bb.L, y1})
		d := c
		buffer[0] = d

		for i = 0; i < xSamples-1; i++ {
			x0 := Lerp(bb.L, bb.R, float64(i+0)*x_denom)
			x1 := Lerp(bb.L, bb.R, float64(i+1)*x_denom)

			a = b
			b = buffer[i+1]
			c = d
			d = marchSample(Vector{x1, y1})
			buffer[i+1] = d

			marchCell(t, a, b, c, d, x0, x1, y0, y1, marchSegment, segmentData)
		}
	}

	return segmentData
}

func seg(v0 Vector, v1 Vector, marchSegment MarchSegmentFunc, segmentData *PolyLineSet) {
	if !v0.Equal(v1) {
		marchSegment(v1, v0, segmentData)
	}
}

func midlerp(x0, x1, s0, s1, t float64) float64 {
	return Lerp(x0, x1, (t-s0)/(s1-s0))
}

func MarchCellSoft(t, a, b, c, d, x0, x1, y0, y1 float64, marchSegment MarchSegmentFunc, segmentData *PolyLineSet) {
	at := 0
	bt := 0
	ct := 0
	dt := 0
	if a > t {
		at = 1
	}
	if b > t {
		bt = 1
	}
	if c > t {
		ct = 1
	}
	if d > t {
		dt = 1
	}

	switch (at)<<0 | (bt)<<1 | (ct)<<2 | (dt)<<3 {
	case 0x1:
		seg(Vector{x0, midlerp(y0, y1, a, c, t)}, Vector{midlerp(x0, x1, a, b, t), y0}, marchSegment, segmentData)
	case 0x2:
		seg(Vector{midlerp(x0, x1, a, b, t), y0}, Vector{x1, midlerp(y0, y1, b, d, t)}, marchSegment, segmentData)
	case 0x3:
		seg(Vector{x0, midlerp(y0, y1, a, c, t)}, Vector{x1, midlerp(y0, y1, b, d, t)}, marchSegment, segmentData)
	case 0x4:
		seg(Vector{midlerp(x0, x1, c, d, t), y1}, Vector{x0, midlerp(y0, y1, a, c, t)}, marchSegment, segmentData)
	case 0x5:
		seg(Vector{midlerp(x0, x1, c, d, t), y1}, Vector{midlerp(x0, x1, a, b, t), y0}, marchSegment, segmentData)
	case 0x6:
		seg(Vector{midlerp(x0, x1, a, b, t), y0}, Vector{x1, midlerp(y0, y1, b, d, t)}, marchSegment, segmentData)
		seg(Vector{midlerp(x0, x1, c, d, t), y1}, Vector{x0, midlerp(y0, y1, a, c, t)}, marchSegment, segmentData)
	case 0x7:
		seg(Vector{midlerp(x0, x1, c, d, t), y1}, Vector{x1, midlerp(y0, y1, b, d, t)}, marchSegment, segmentData)
	case 0x8:
		seg(Vector{x1, midlerp(y0, y1, b, d, t)}, Vector{midlerp(x0, x1, c, d, t), y1}, marchSegment, segmentData)
	case 0x9:
		seg(Vector{x0, midlerp(y0, y1, a, c, t)}, Vector{midlerp(x0, x1, a, b, t), y0}, marchSegment, segmentData)
		seg(Vector{x1, midlerp(y0, y1, b, d, t)}, Vector{midlerp(x0, x1, c, d, t), y1}, marchSegment, segmentData)
	case 0xA:
		seg(Vector{midlerp(x0, x1, a, b, t), y0}, Vector{midlerp(x0, x1, c, d, t), y1}, marchSegment, segmentData)
	case 0xB:
		seg(Vector{x0, midlerp(y0, y1, a, c, t)}, Vector{midlerp(x0, x1, c, d, t), y1}, marchSegment, segmentData)
	case 0xC:
		seg(Vector{x1, midlerp(y0, y1, b, d, t)}, Vector{x0, midlerp(y0, y1, a, c, t)}, marchSegment, segmentData)
	case 0xD:
		seg(Vector{x1, midlerp(y0, y1, b, d, t)}, Vector{midlerp(x0, x1, a, b, t), y0}, marchSegment, segmentData)
	case 0xE:
		seg(Vector{midlerp(x0, x1, a, b, t), y0}, Vector{x0, midlerp(y0, y1, a, c, t)}, marchSegment, segmentData)
	}
}

/// Trace an anti-aliased contour of an image along a particular threshold.
/// The given number of samples will be taken and spread across the bounding box area using the sampling function and context.
/// The segment function will be called for each segment detected that lies along the density contour for @c threshold.
func MarchSoft(bb BB, xSamples, ySamples int64, t float64, marchSegment MarchSegmentFunc, marchSample MarchSampleFunc) *PolyLineSet {
	return MarchCells(bb, xSamples, ySamples, t, marchSegment, marchSample, MarchCellSoft)
}

func segs(a, b, c Vector, marchSegment MarchSegmentFunc, segmentData *PolyLineSet) {
	seg(b, c, marchSegment, segmentData)
	seg(a, b, marchSegment, segmentData)
}

func MarchCellHard(t, a, b, c, d, x0, x1, y0, y1 float64, marchSegment MarchSegmentFunc, segmentData *PolyLineSet) {
	xm := Lerp(x0, x1, 0.5)
	ym := Lerp(y0, y1, 0.5)

	at := 0
	bt := 0
	ct := 0
	dt := 0
	if a > t {
		at = 1
	}
	if b > t {
		bt = 1
	}
	if c > t {
		ct = 1
	}
	if d > t {
		dt = 1
	}

	switch (at)<<0 | (bt)<<1 | (ct)<<2 | (dt)<<3 {
	case 0x1:
		segs(Vector{x0, ym}, Vector{xm, ym}, Vector{xm, y0}, marchSegment, segmentData)
	case 0x2:
		segs(Vector{xm, y0}, Vector{xm, ym}, Vector{x1, ym}, marchSegment, segmentData)
	case 0x3:
		seg(Vector{x0, ym}, Vector{x1, ym}, marchSegment, segmentData)
	case 0x4:
		segs(Vector{xm, y1}, Vector{xm, ym}, Vector{x0, ym}, marchSegment, segmentData)
	case 0x5:
		seg(Vector{xm, y1}, Vector{xm, y0}, marchSegment, segmentData)
	case 0x6:
		segs(Vector{xm, y0}, Vector{xm, ym}, Vector{x0, ym}, marchSegment, segmentData)
		segs(Vector{xm, y1}, Vector{xm, ym}, Vector{x1, ym}, marchSegment, segmentData)
	case 0x7:
		segs(Vector{xm, y1}, Vector{xm, ym}, Vector{x1, ym}, marchSegment, segmentData)
	case 0x8:
		segs(Vector{x1, ym}, Vector{xm, ym}, Vector{xm, y1}, marchSegment, segmentData)
	case 0x9:
		segs(Vector{x1, ym}, Vector{xm, ym}, Vector{xm, y0}, marchSegment, segmentData)
		segs(Vector{x0, ym}, Vector{xm, ym}, Vector{xm, y1}, marchSegment, segmentData)
	case 0xA:
		seg(Vector{xm, y0}, Vector{xm, y1}, marchSegment, segmentData)
	case 0xB:
		segs(Vector{x0, ym}, Vector{xm, ym}, Vector{xm, y1}, marchSegment, segmentData)
	case 0xC:
		seg(Vector{x1, ym}, Vector{x0, ym}, marchSegment, segmentData)
	case 0xD:
		segs(Vector{x1, ym}, Vector{xm, ym}, Vector{xm, y0}, marchSegment, segmentData)
	case 0xE:
		segs(Vector{xm, y0}, Vector{xm, ym}, Vector{x0, ym}, marchSegment, segmentData)
	}
}

/// Trace an aliased curve of an image along a particular threshold.
/// The given number of samples will be taken and spread across the bounding box area using the sampling function and context.
/// The segment function will be called for each segment detected that lies along the density contour for @c threshold.
func MarchHard(bb BB, xSamples, ySamples int64, t float64, marchSegment MarchSegmentFunc, marchSample MarchSampleFunc) *PolyLineSet {
	return MarchCells(bb, xSamples, ySamples, t, marchSegment, marchSample, MarchCellHard)
}
