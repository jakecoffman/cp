package cp

// This is a user defined function that gets passed in to the Marching process
// the user establishes a PolyLineSet, passes a pointer to their function, and they
// populate it. In most cases you want to use PolyLineCollectSegment instead of defining your own
type MarchSegmentFunc func(v0 Vector, v1 Vector, segment_data *PolyLineSet)

// This is a user defined function that gets passed every single point from the bounding
// box the user passes into the March process - you can use this to sample an image and
// check for alpha values or really any 2d matrix you define like a tile map.
// NOTE: I could not determine a use case for the sample_data pointer from the original code
// so I removed it here - open to adding it back in if there is a reason.
type MarchSampleFunc func(point Vector) float64

type MarchCellFunc func(t, a, b, c, d, x0, x1, y0, y1 float64, march_segment MarchSegmentFunc, segment_data *PolyLineSet)

// The looping and sample caching code is shared between cpMarchHard() and cpMarchSoft().
func MarchCells(
	bb BB,
	x_samples int64,
	y_samples int64,
	t float64,
	march_segment MarchSegmentFunc, segment_data *PolyLineSet,
	march_sample MarchSampleFunc,
	march_cell MarchCellFunc) {

	var x_denom, y_denom float64
	x_denom = 1.0 / float64(x_samples-1)
	y_denom = 1.0 / float64(y_samples-1)

	buffer := make([]float64, x_samples)
	var i, j int64
	for i = 0; i < x_samples; i++ {
		buffer[i] = march_sample(Vector{Lerp(bb.L, bb.R, float64(i)*x_denom), bb.B})
	}

	for j = 0; j < y_samples-1; j++ {
		y0 := Lerp(bb.B, bb.T, float64(j+0)*y_denom)
		y1 := Lerp(bb.B, bb.T, float64(j+1)*y_denom)

		a := buffer[0]
		b := buffer[0]
		c := march_sample(Vector{bb.L, y1})
		d := c
		buffer[0] = d

		for i = 0; i < x_samples-1; i++ {
			x0 := Lerp(bb.L, bb.R, float64(i+0)*x_denom)
			x1 := Lerp(bb.L, bb.R, float64(i+1)*x_denom)

			a = b
			b = buffer[i+1]
			c = d
			d = march_sample(Vector{x1, y1})
			buffer[i+1] = d

			march_cell(t, a, b, c, d, x0, x1, y0, y1, march_segment, segment_data)
		}
	}
}

func seg(v0 Vector, v1 Vector, march_segment MarchSegmentFunc, segment_data *PolyLineSet) {
	if !v0.Equal(v1) {
		march_segment(v1, v0, segment_data)
	}
}

func midlerp(x0, x1, s0, s1, t float64) float64 {
	return Lerp(x0, x1, (t-s0)/(s1-s0))
}

func MarchCellSoft(t, a, b, c, d, x0, x1, y0, y1 float64, march_segment MarchSegmentFunc, segment_data *PolyLineSet) {

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
		seg(Vector{x0, midlerp(y0, y1, a, c, t)}, Vector{midlerp(x0, x1, a, b, t), y0}, march_segment, segment_data)
	case 0x2:
		seg(Vector{midlerp(x0, x1, a, b, t), y0}, Vector{x1, midlerp(y0, y1, b, d, t)}, march_segment, segment_data)
	case 0x3:
		seg(Vector{x0, midlerp(y0, y1, a, c, t)}, Vector{x1, midlerp(y0, y1, b, d, t)}, march_segment, segment_data)
	case 0x4:
		seg(Vector{midlerp(x0, x1, c, d, t), y1}, Vector{x0, midlerp(y0, y1, a, c, t)}, march_segment, segment_data)
	case 0x5:
		seg(Vector{midlerp(x0, x1, c, d, t), y1}, Vector{midlerp(x0, x1, a, b, t), y0}, march_segment, segment_data)
	case 0x6:
		seg(Vector{midlerp(x0, x1, a, b, t), y0}, Vector{x1, midlerp(y0, y1, b, d, t)}, march_segment, segment_data)
		seg(Vector{midlerp(x0, x1, c, d, t), y1}, Vector{x0, midlerp(y0, y1, a, c, t)}, march_segment, segment_data)
	case 0x7:
		seg(Vector{midlerp(x0, x1, c, d, t), y1}, Vector{x1, midlerp(y0, y1, b, d, t)}, march_segment, segment_data)
	case 0x8:
		seg(Vector{x1, midlerp(y0, y1, b, d, t)}, Vector{midlerp(x0, x1, c, d, t), y1}, march_segment, segment_data)
	case 0x9:
		seg(Vector{x0, midlerp(y0, y1, a, c, t)}, Vector{midlerp(x0, x1, a, b, t), y0}, march_segment, segment_data)
		seg(Vector{x1, midlerp(y0, y1, b, d, t)}, Vector{midlerp(x0, x1, c, d, t), y1}, march_segment, segment_data)
	case 0xA:
		seg(Vector{midlerp(x0, x1, a, b, t), y0}, Vector{midlerp(x0, x1, c, d, t), y1}, march_segment, segment_data)
	case 0xB:
		seg(Vector{x0, midlerp(y0, y1, a, c, t)}, Vector{midlerp(x0, x1, c, d, t), y1}, march_segment, segment_data)
	case 0xC:
		seg(Vector{x1, midlerp(y0, y1, b, d, t)}, Vector{x0, midlerp(y0, y1, a, c, t)}, march_segment, segment_data)
	case 0xD:
		seg(Vector{x1, midlerp(y0, y1, b, d, t)}, Vector{midlerp(x0, x1, a, b, t), y0}, march_segment, segment_data)
	case 0xE:
		seg(Vector{midlerp(x0, x1, a, b, t), y0}, Vector{x0, midlerp(y0, y1, a, c, t)}, march_segment, segment_data)
	}
}

/// Trace an anti-aliased contour of an image along a particular threshold.
/// The given number of samples will be taken and spread across the bounding box area using the sampling function and context.
/// The segment function will be called for each segment detected that lies along the density contour for @c threshold.
func MarchSoft(
	bb BB, x_samples, y_samples int64, t float64,
	march_segment MarchSegmentFunc, segment_data *PolyLineSet,
	march_sample MarchSampleFunc) {
	MarchCells(bb, x_samples, y_samples, t, march_segment, segment_data, march_sample, MarchCellSoft)
}

func segs(a, b, c Vector, march_segment MarchSegmentFunc, segment_data *PolyLineSet) {
	seg(b, c, march_segment, segment_data)
	seg(a, b, march_segment, segment_data)
}

func MarchCellHard(t, a, b, c, d, x0, x1, y0, y1 float64, march_segment MarchSegmentFunc, segment_data *PolyLineSet) {
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
		segs(Vector{x0, ym}, Vector{xm, ym}, Vector{xm, y0}, march_segment, segment_data)
	case 0x2:
		segs(Vector{xm, y0}, Vector{xm, ym}, Vector{x1, ym}, march_segment, segment_data)
	case 0x3:
		seg(Vector{x0, ym}, Vector{x1, ym}, march_segment, segment_data)
	case 0x4:
		segs(Vector{xm, y1}, Vector{xm, ym}, Vector{x0, ym}, march_segment, segment_data)
	case 0x5:
		seg(Vector{xm, y1}, Vector{xm, y0}, march_segment, segment_data)
	case 0x6:
		segs(Vector{xm, y0}, Vector{xm, ym}, Vector{x0, ym}, march_segment, segment_data)
		segs(Vector{xm, y1}, Vector{xm, ym}, Vector{x1, ym}, march_segment, segment_data)
	case 0x7:
		segs(Vector{xm, y1}, Vector{xm, ym}, Vector{x1, ym}, march_segment, segment_data)
	case 0x8:
		segs(Vector{x1, ym}, Vector{xm, ym}, Vector{xm, y1}, march_segment, segment_data)
	case 0x9:
		segs(Vector{x1, ym}, Vector{xm, ym}, Vector{xm, y0}, march_segment, segment_data)
		segs(Vector{x0, ym}, Vector{xm, ym}, Vector{xm, y1}, march_segment, segment_data)
	case 0xA:
		seg(Vector{xm, y0}, Vector{xm, y1}, march_segment, segment_data)
	case 0xB:
		segs(Vector{x0, ym}, Vector{xm, ym}, Vector{xm, y1}, march_segment, segment_data)
	case 0xC:
		seg(Vector{x1, ym}, Vector{x0, ym}, march_segment, segment_data)
	case 0xD:
		segs(Vector{x1, ym}, Vector{xm, ym}, Vector{xm, y0}, march_segment, segment_data)
	case 0xE:
		segs(Vector{xm, y0}, Vector{xm, ym}, Vector{x0, ym}, march_segment, segment_data)
	}
}

/// Trace an aliased curve of an image along a particular threshold.
/// The given number of samples will be taken and spread across the bounding box area using the sampling function and context.
/// The segment function will be called for each segment detected that lies along the density contour for @c threshold.
func MarchHard(
	bb BB, x_samples, y_samples int64, t float64,
	march_segment MarchSegmentFunc, segment_data *PolyLineSet,
	march_sample MarchSampleFunc) {
	MarchCells(bb, x_samples, y_samples, t, march_segment, segment_data, march_sample, MarchCellHard)
}
