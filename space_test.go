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
	bb1 := circle.bb
	space.ReindexShape(circle)
	bb2 := circle.bb
	// check unchanged
	if got, want := bb1.String(), bb2.String(); got != want {
		t.Errorf("got [%[1]v:%[1]T] want [%[2]v:%[2]T]", got, want)
	}
	circle.body.SetPosition(Vector{X: 12.0, Y: 34.0})
	space.ReindexShape(circle)
	bb3 := circle.bb
	// check changed
	if got, want := bb2.String(), bb3.String(); got == want {
		t.Errorf("got [%[1]v:%[1]T] want [%[2]v:%[2]T]", got, want)
	}
}
