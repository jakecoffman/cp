package physics

//Draw flags
const (
	DRAW_SHAPES           = 1 << 0
	DRAW_CONSTRAINTS      = 1 << 1
	DRAW_COLLISION_POINTS = 1 << 2
)

// 16 bytes
type FColor struct {
	R, G, B, A float32
}

type Drawer interface {
	DrawCircle(pos *Vector, angle, radius float64, outline, fill FColor, data interface{})
	DrawSegment(a, b *Vector, fill FColor, data interface{})
	DrawFatSegment(a, b *Vector, radius float64, outline, fill FColor, data interface{})
	DrawPolygon(count uint, verts []*Vector, radius float64, outline, fill FColor, data interface{})
	DrawDot(size float64, pos *Vector, fill FColor, data interface{})

	Flags() int
	OutlineColor() FColor
	ShapeColor(shape *Shape, data interface{}) FColor
	ConstraintColor() FColor
	CollisionPointColor() FColor
	Data() interface{}
}

func DrawShape(shape *Shape, options Drawer) {
	body := shape.body
	data := options.Data()

	outline := options.OutlineColor()
	fill := options.ShapeColor(shape, data)

	switch shape.Class.(type) {
	case *Circle:
		circle := shape.Class.(*Circle)
		options.DrawCircle(circle.tc, body.a, circle.r, outline, fill, data)
	case *Segment:
		seg := shape.Class.(*Segment)
		options.DrawFatSegment(seg.ta, seg.tb, seg.r, outline, fill, data)
	case *PolyShape:
		poly := shape.Class.(*PolyShape)

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
		drawSeg := options.DrawSegment
		data := options.Data()

		for _, arb := range space.arbiters {
			n := arb.n

			for j := 0; j < len(space.arbiters); j++ {
				p1 := arb.body_a.p.Add(arb.contacts[j].r1)
				p2 := arb.body_b.p.Add(arb.contacts[j].r2)

				a := p1.Add(n.Mult(-2))
				b := p2.Add(n.Mult(2))
				drawSeg(a, b, options.CollisionPointColor(), data)
			}
		}
	}
}
