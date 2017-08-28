package examples

import (
	. "github.com/jakecoffman/physics"
)

type DrawOptions struct {
	flags                               int
	outline, constraint, collisionPoint FColor
	data                                interface{}
}

func NewDrawOptions(flags int, outline, constraint, collisionPoint FColor, data interface{}) *DrawOptions {
	return &DrawOptions{flags, outline, constraint, collisionPoint, data}
}

func (*DrawOptions) DrawCircle(pos Vector, angle, radius float64, outline, fill FColor, data interface{}) {
	DrawCircle(pos, angle, radius, outline, fill)
}

func (*DrawOptions) DrawSegment(a, b Vector, fill FColor, data interface{}) {
	DrawSegment(a, b, fill)
}

func (*DrawOptions) DrawFatSegment(a, b Vector, radius float64, outline, fill FColor, data interface{}) {
	DrawFatSegment(a, b, radius, outline, fill)
}

func (*DrawOptions) DrawPolygon(count uint, verts []Vector, radius float64, outline, fill FColor, data interface{}) {
	DrawPolygon(count, verts, radius, outline, fill)
}

func (*DrawOptions) DrawDot(size float64, pos Vector, fill FColor, data interface{}) {
	DrawDot(size, pos, fill)
}

func (d *DrawOptions) Flags() int {
	return d.flags
}

func (d *DrawOptions) OutlineColor() FColor {
	return d.outline
}

func (*DrawOptions) ShapeColor(shape *Shape, data interface{}) FColor {
	return ColorForShape(shape, data)
}

func (d *DrawOptions) ConstraintColor() FColor {
	return d.constraint
}

func (d *DrawOptions) CollisionPointColor() FColor {
	return d.collisionPoint
}

func (d *DrawOptions) Data() interface{} {
	return d.data
}
