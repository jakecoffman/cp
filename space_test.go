package cp

import "testing"

func TestSpace_ShapeQuery(t *testing.T) {
	space := NewSpace()
	circle := space.AddShape(NewCircle(space.StaticBody, 1, Vector{}))
	space.ShapeQuery(circle, func(shape *Shape, points *ContactPointSet) {
		t.Fatal("Shouldn't collide with itself")
	})
	box := NewBox(NewBody(1, 1), 1, 1, 1)

	var called int
	space.ShapeQuery(box, func(shape *Shape, points *ContactPointSet) {
		called++
	})
	if called != 1 {
		t.Error("Expected box to collide with circle")
	}

	box.body.SetPosition(Vector{3, 0})
	space.ShapeQuery(box, func(shape *Shape, points *ContactPointSet) {
		t.Error("Box should be just out of range")
	})
}

func TestSpace_ReindexShape(t *testing.T) {
	space := NewSpace()
	circle := space.AddShape(NewCircle(space.StaticBody, 1, Vector{}))
	space.ReindexShape(circle)
	circle.body.SetPosition(Vector{X: 12.0, Y: 34.0})
	space.ReindexShape(circle)
}
