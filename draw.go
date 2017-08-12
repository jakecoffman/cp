package physics

import "image/color"

//Draw flags
const (
	DRAW_SHAPES           = 1 << 0
	DRAW_CONSTRAINTS      = 1 << 1
	DRAW_COLLISION_POINTS = 1 << 2
)

type Drawer interface {
	DrawCircle(pos *Vector, angle, radius float64, outline, fill color.Color, data interface{})
	DrawSegment(a, b *Vector, fill color.Color, data interface{})
	DrawFatSegment(a, b *Vector, radius float64, outline, fill color.Color, data interface{})
	DrawPolygon(count uint, verts []*Vector, radius float64, outline, fill color.Color, data interface{})
	DrawDot(size float64, pos *Vector, fill color.Color, data interface{})

	Flags() int
	OutlineColor() color.Color
	ShapeColor(shape *Shape, data interface{}) color.Color
	ConstraintColor() color.Color
	CollisionPointColor() color.Color
	Data() interface{}
}

func DrawShape(shape *Shape, options Drawer) {
	body := shape.body
	data := options.Data()

	outline := options.OutlineColor()
	fill := options.ShapeColor(shape, data)

	switch shape.class.(type) {
	case *Circle:
		circle := shape.class.(*Circle)
		options.DrawCircle(circle.tc, body.a, circle.r, outline, fill, data)
	case *Segment:
		seg := shape.class.(*Segment)
		options.DrawFatSegment(seg.ta, seg.tb, seg.r, outline, fill, data)
	case *PolyShape:
		poly := shape.class.(*PolyShape)

		count := poly.count
		planes := poly.planes
		verts := make([]*Vector, count)

		var i uint
		for i = 0; i < count; i++ {
			verts[i] = planes[i].v0
		}
		options.DrawPolygon(count, verts, poly.r, outline, fill, data)
	default:
		panic("Unknown shape type")
	}
}

func DrawConstraint(constraint *Constraint, options Drawer) {
	// TODO
	return
}

func DrawSpace(space *Space, options Drawer) {
	if options.Flags() & DRAW_SHAPES == 1 {
		space.dynamicShapes.class.Each(func(obj interface{}, data interface{}) {
			DrawShape(obj.(*Shape), data.(Drawer))
		}, options)
		space.staticShapes.class.Each(func(obj interface{}, data interface{}) {
			DrawShape(obj.(*Shape), data.(Drawer))
		}, options)
	}

	if options.Flags() & DRAW_CONSTRAINTS == 1 {
		for _, constraint := range space.constraints {
			DrawConstraint(constraint, options)
		}
	}

	if options.Flags() & DRAW_COLLISION_POINTS == 1 {
		arbiters := space.arbiters
		drawSeg := options.DrawSegment
		data := options.Data()

		for _, arb := range arbiters {
			n := arb.n

			for j := 0; j < len(arbiters); j++ {
				p1 := arb.body_a.p.Add(arb.contacts[j].r1)
				p2 := arb.body_b.p.Add(arb.contacts[j].r2)

				a := p1.Add(n.Mult(-2))
				b := p2.Add(n.Mult(2))
				drawSeg(a, b, options.CollisionPointColor(), data)
			}
		}
	}
}
