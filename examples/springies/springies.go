package main

import "github.com/jakecoffman/physics/examples"
import . "github.com/jakecoffman/physics"

func main() {
	space := NewSpace()

	body1 := addBar(space, Vector{-240, 160}, Vector{-160, 80}, 1)
	body2 := addBar(space, Vector{-160, 80}, Vector{-80, 160}, 1)
	body3 := addBar(space, Vector{0, 160}, Vector{80, 0}, 1)
	body4 := addBar(space, Vector{160, 160}, Vector{240, 160}, 1)
	body5 := addBar(space, Vector{-240, 0}, Vector{-160, -80}, 1)
	body6 := addBar(space, Vector{-160, -80}, Vector{-80, 0}, 1)
	body7 := addBar(space, Vector{-80, 0}, Vector{0, 0}, 1)
	body8 := addBar(space, Vector{0, -80}, Vector{80, -80}, 1)
	body9 := addBar(space, Vector{240, 80}, Vector{160, 0}, 1)
	body10 := addBar(space, Vector{160, 0}, Vector{240, -80}, 1)
	body11 := addBar(space, Vector{-240, -80}, Vector{-160, -160}, 1)
	body12 := addBar(space, Vector{-160, -160}, Vector{-80, -160}, 1)
	body13 := addBar(space, Vector{0, -160}, Vector{80, -160}, 1)
	body14 := addBar(space, Vector{160, -160}, Vector{240, -160}, 1)

	space.AddConstraint(NewPivotJoint2(body1, body2, Vector{40, -40}, Vector{-40, -40}))
	space.AddConstraint(NewPivotJoint2(body5, body6, Vector{40, -40}, Vector{-40, -40}))
	space.AddConstraint(NewPivotJoint2(body6, body7, Vector{40, 40}, Vector{-40, 0}))
	space.AddConstraint(NewPivotJoint2(body9, body10, Vector{-40, -40}, Vector{-40, 40}))
	space.AddConstraint(NewPivotJoint2(body11, body12, Vector{40, -40}, Vector{-40, 0}))

	stiff := 100.0
	damp := 0.5
	space.AddConstraint(newSpring(space.StaticBody, body1, Vector{-320, 240}, Vector{-40, 40}, 0, stiff, damp))
	space.AddConstraint(newSpring(space.StaticBody, body1, Vector{-320, 80}, Vector{-40, 40}, 0, stiff, damp))
	space.AddConstraint(newSpring(space.StaticBody, body1, Vector{-160, 240}, Vector{-40, 40}, 0, stiff, damp))

	space.AddConstraint(newSpring(space.StaticBody, body2, Vector{-160, 240}, Vector{40, 40}, 0, stiff, damp))
	space.AddConstraint(newSpring(space.StaticBody, body2, Vector{0, 240}, Vector{40, 40}, 0, stiff, damp))

	space.AddConstraint(newSpring(space.StaticBody, body3, Vector{80, 240}, Vector{-40, 80}, 0, stiff, damp))

	space.AddConstraint(newSpring(space.StaticBody, body4, Vector{80, 240}, Vector{-40, 0}, 0, stiff, damp))
	space.AddConstraint(newSpring(space.StaticBody, body4, Vector{320, 240}, Vector{40, 0}, 0, stiff, damp))

	space.AddConstraint(newSpring(space.StaticBody, body5, Vector{-320, 80}, Vector{-40, 40}, 0, stiff, damp))

	space.AddConstraint(newSpring(space.StaticBody, body9, Vector{320, 80}, Vector{40, 40}, 0, stiff, damp))

	space.AddConstraint(newSpring(space.StaticBody, body10, Vector{320, 0}, Vector{40, -40}, 0, stiff, damp))
	space.AddConstraint(newSpring(space.StaticBody, body10, Vector{320, -160}, Vector{40, -40}, 0, stiff, damp))

	space.AddConstraint(newSpring(space.StaticBody, body11, Vector{-320, -160}, Vector{-40, 40}, 0, stiff, damp))

	space.AddConstraint(newSpring(space.StaticBody, body12, Vector{-240, -240}, Vector{-40, 0}, 0, stiff, damp))
	space.AddConstraint(newSpring(space.StaticBody, body12, Vector{0, -240}, Vector{40, 0}, 0, stiff, damp))

	space.AddConstraint(newSpring(space.StaticBody, body13, Vector{0, -240}, Vector{-40, 0}, 0, stiff, damp))
	space.AddConstraint(newSpring(space.StaticBody, body13, Vector{80, -240}, Vector{40, 0}, 0, stiff, damp))

	space.AddConstraint(newSpring(space.StaticBody, body14, Vector{80, -240}, Vector{-40, 0}, 0, stiff, damp))
	space.AddConstraint(newSpring(space.StaticBody, body14, Vector{240, -240}, Vector{40, 0}, 0, stiff, damp))
	space.AddConstraint(newSpring(space.StaticBody, body14, Vector{320, -160}, Vector{40, 0}, 0, stiff, damp))

	space.AddConstraint(newSpring(body1, body5, Vector{40, -40}, Vector{-40, 40}, 0, stiff, damp))
	space.AddConstraint(newSpring(body1, body6, Vector{40, -40}, Vector{40, 40}, 0, stiff, damp))
	space.AddConstraint(newSpring(body2, body3, Vector{40, 40}, Vector{-40, 80}, 0, stiff, damp))
	space.AddConstraint(newSpring(body3, body4, Vector{-40, 80}, Vector{-40, 0}, 0, stiff, damp))
	space.AddConstraint(newSpring(body3, body4, Vector{40, -80}, Vector{-40, 0}, 0, stiff, damp))
	space.AddConstraint(newSpring(body3, body7, Vector{40, -80}, Vector{40, 0}, 0, stiff, damp))
	space.AddConstraint(newSpring(body3, body7, Vector{-40, 80}, Vector{-40, 0}, 0, stiff, damp))
	space.AddConstraint(newSpring(body3, body8, Vector{40, -80}, Vector{40, 0}, 0, stiff, damp))
	space.AddConstraint(newSpring(body3, body9, Vector{40, -80}, Vector{-40, -40}, 0, stiff, damp))
	space.AddConstraint(newSpring(body4, body9, Vector{40, 0}, Vector{40, 40}, 0, stiff, damp))
	space.AddConstraint(newSpring(body5, body11, Vector{-40, 40}, Vector{-40, 40}, 0, stiff, damp))
	space.AddConstraint(newSpring(body5, body11, Vector{40, -40}, Vector{40, -40}, 0, stiff, damp))
	space.AddConstraint(newSpring(body7, body8, Vector{40, 0}, Vector{-40, 0}, 0, stiff, damp))
	space.AddConstraint(newSpring(body8, body12, Vector{-40, 0}, Vector{40, 0}, 0, stiff, damp))
	space.AddConstraint(newSpring(body8, body13, Vector{-40, 0}, Vector{-40, 0}, 0, stiff, damp))
	space.AddConstraint(newSpring(body8, body13, Vector{40, 0}, Vector{40, 0}, 0, stiff, damp))
	space.AddConstraint(newSpring(body8, body14, Vector{40, 0}, Vector{-40, 0}, 0, stiff, damp))
	space.AddConstraint(newSpring(body10, body14, Vector{40, -40}, Vector{-40, 0}, 0, stiff, damp))
	space.AddConstraint(newSpring(body10, body14, Vector{40, -40}, Vector{-40, 0}, 0, stiff, damp))

	examples.Main(space, 640, 480, 1.0/60.0, update)
}

func springForce(spring *DampedSpring, dist float64) float64 {
	clamp := 20.0
	return Clamp(spring.RestLength-dist, -clamp, clamp)*spring.Stiffness
}

func newSpring(a, b *Body, anchorA, anchorB Vector, restLength, stiff, damp float64) *Constraint {
	constraint := NewDampedSpring(a, b, anchorA, anchorB, restLength, stiff, damp)
	spring := constraint.Class.(*DampedSpring)
	spring.SpringForceFunc = springForce
	return spring.Constraint
}

func addBar(space *Space, a, b Vector, group uint) *Body {
	center := a.Add(b).Mult(1.0/2.0)
	length := b.Sub(a).Length()
	mass := length/160.0

	body := space.AddBody(NewBody(mass, mass*length*length/12.0))
	body.SetPosition(center)

	shape := space.AddShape(NewSegment(body, a.Sub(center), b.Sub(center), 10))
	shape.SetFilter(NewShapeFilter(group, ALL_CATEGORIES, ALL_CATEGORIES))
	return body
}

func update(space *Space, dt float64) {
	space.Step(dt)
}