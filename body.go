package physics

import "math"

/// Rigid body velocity update function type.
type BodyVelocityFunc func(body Body, gravity Vector, damping float64, dt float64)

/// Rigid body position update function type.
type BodyPositionFunc func(body Body, dt float64)

type Body struct {
	// Integration functions
	velocity_func BodyVelocityFunc
	position_func BodyPositionFunc

	// mass and it's inverse
	m     float64
	m_inv float64

	// moment of inertia and it's inverse
	i     float64
	i_inv float64

	// center of gravity
	cog Vector

	// position, velocity, force
	p Vector
	v Vector
	f Vector

	// Angle, angular velocity, torque (radians)
	a float64
	w float64
	t float64

	transform Transform

	userData interface{}

	// "pseudo-velocities" used for eliminating overlap.
	// Erin Catto has some papers that talk about what these are.
	v_bias Vector
	w_bias float64

	space *Space

	shapeList      []*Shape
	arbiterList    []*Arbiter
	constraintList []*Constraint

	sleeping struct {
		root     *Body
		next     *Body
		idleTime float64
	}
}

func NewBody(mass, moment float64) Body {
	return Body {

	}
}

// body types
const (
	BODY_DYNAMIC = iota
	BODY_KINEMATIC
	BODY_STATIC
)

func (b *Body) SetType(typ int) {
	oldType := b.GetType()
	if oldType == typ {
		return
	}

	if typ == BODY_STATIC {
		b.sleeping.idleTime = INFINITY
	} else {
		b.sleeping.idleTime = 0
	}

	if typ == BODY_DYNAMIC {
		b.m = 0
		b.i = 0
		b.m_inv = INFINITY
		b.i_inv = INFINITY

		b.BodyAccumulateMassFromShapes()
	} else {
		b.m = INFINITY
		b.i = INFINITY
		b.m_inv = 0
		b.i_inv = 0

		b.v = VectorZero()
		b.w = 0
	}

	if b.space != nil {
		panic("Implement setting types on bodies that are already in a space")
	}
}

func (b *Body) GetType() int {
	if b.sleeping.idleTime == math.MaxFloat64 {
		return BODY_STATIC
	}
	if b.m == math.MaxFloat64 {
		return BODY_KINEMATIC
	}
	return BODY_DYNAMIC
}

// Should *only* be called when shapes with mass info are modified, added or removed.
func (b *Body) BodyAccumulateMassFromShapes() {
	if b == nil || b.GetType() != BODY_DYNAMIC {
		return
	}

	b.m = 0
	b.i = 0
	b.cog = VectorZero()

	// TODO Workin on it
	//pos := b.Position()
	//for _, shape := range b.shapeList {
	//	info := shape.massInfo
	//	m := info.m
	//
	//	if shape.massInfo.m > 0 {
	//		b.i += m*info.i +
	//	}
	//}
}

func (b *Body) Position() *Vector {
	return TransformPoint(b.transform, b.p)
}

func TransformPoint(t Transform, p Vector) *Vector {
	return &Vector{X: t.a*p.X + t.c*p.Y + t.tx, Y: t.b*p.X + t.d*p.Y + t.ty}
}