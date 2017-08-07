package physics

import (
	"log"
	"math"
)

/// Rigid body velocity update function type.
type BodyVelocityFunc func(body *Body, gravity *Vector, damping float64, dt float64)

/// Rigid body position update function type.
type BodyPositionFunc func(body *Body, dt float64)

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
	cog *Vector

	// position, velocity, force
	p *Vector
	v *Vector
	f *Vector

	// Angle, angular velocity, torque (radians)
	a float64
	w float64
	t float64

	transform *Transform

	userData interface{}

	// "pseudo-velocities" used for eliminating overlap.
	// Erin Catto has some papers that talk about what these are.
	v_bias *Vector
	w_bias float64

	space *Space

	shapeList      []Shaper
	arbiterList    []*Arbiter
	constraintList []*Constraint

	sleeping struct {
		root     *Body
		next     *Body
		idleTime float64
	}
}

func NewBody(mass, moment float64) *Body {
	return &Body{
		cog:           VectorZero(),
		p:             VectorZero(),
		v:             VectorZero(),
		f:             VectorZero(),
		transform:     NewTransformIdentity(),
		velocity_func: BodyUpdateVelocity,
		position_func: BodyUpdatePosition,
	}
}

// body types
const (
	BODY_DYNAMIC = iota
	BODY_KINEMATIC
	BODY_STATIC
)

func (body *Body) SetType(typ int) {
	oldType := body.GetType()
	if oldType == typ {
		return
	}

	if typ == BODY_STATIC {
		body.sleeping.idleTime = INFINITY
	} else {
		body.sleeping.idleTime = 0
	}

	if typ == BODY_DYNAMIC {
		body.m = 0
		body.i = 0
		body.m_inv = INFINITY
		body.i_inv = INFINITY

		body.AccumulateMassFromShapes()
	} else {
		body.m = INFINITY
		body.i = INFINITY
		body.m_inv = 0
		body.i_inv = 0

		body.v = VectorZero()
		body.w = 0
	}

	if body.space != nil {
		panic("TODO implement setting types on bodies that are already in a space")
	}
}

func (body *Body) GetType() int {
	if body.sleeping.idleTime == math.MaxFloat64 {
		return BODY_STATIC
	}
	if body.m == math.MaxFloat64 {
		return BODY_KINEMATIC
	}
	return BODY_DYNAMIC
}

// Should *only* be called when shapes with mass info are modified, added or removed.
func (body *Body) AccumulateMassFromShapes() {
	if body == nil || body.GetType() != BODY_DYNAMIC {
		return
	}

	body.m = 0
	body.i = 0
	body.cog = VectorZero()

	// cache position, realign at the end
	pos := body.Position()

	for _, shape := range body.shapeList {
		info := shape.MassInfo()
		m := info.m

		if info.m > 0 {
			msum := body.m + m
			body.i += m*info.i + body.cog.DistanceSq(info.cog)*(m*body.m)/msum
			body.cog = body.cog.Lerp(info.cog, m/msum)
			body.m = msum
		}
	}

	body.m_inv = 1.0 / body.m
	body.i_inv = 1.0 / body.i

	body.SetPosition(pos)
}

func (body *Body) Position() *Vector {
	return body.transform.Point(body.p)
}

func (body *Body) SetPosition(position *Vector) {
	body.Activate()
	body.p = body.transform.Vect(body.cog).Add(position)
	body.SetTransform(body.p, body.a)
}

func (body *Body) SetTransform(p *Vector, a float64) {
	rot := ForAngle(a)
	c := body.cog

	body.transform = NewTransformTranspose(
		rot.X, -rot.Y, p.X-(c.X*rot.X-c.Y*rot.Y),
		rot.Y, rot.X, p.Y-(c.X*rot.Y+c.Y*rot.X),
	)
}

func (body *Body) Activate() {
	if body == nil || body.GetType() != BODY_DYNAMIC {
		return
	}

	body.sleeping.idleTime = 0

	root := body.ComponentRoot()
	if root != nil && root.IsSleeping() {
		space := root.space
		body := root
		for {
			if body == nil {
				break
			}
			next := body.sleeping.next
			body.sleeping.idleTime = 0
			body.sleeping.root = nil
			body.sleeping.next = nil
			space.Activate(body)

			body = next
		}

		for i := 0; i < len(space.sleepingComponents); i++ {
			if space.sleepingComponents[i] == root {
				space.sleepingComponents = append(space.sleepingComponents[:i], space.sleepingComponents[i+1:]...)
				break
			}
		}
	}

	for _, arbiter := range body.arbiterList {
		var other *Body
		if arbiter.body_a == body {
			other = arbiter.body_b
		} else {
			other = arbiter.body_a
		}
		if other.GetType() == BODY_STATIC {
			other.sleeping.idleTime = 0
		}
	}

}

func (body *Body) ComponentRoot() *Body {
	if body != nil {
		return body.sleeping.root
	}
	return nil
}

func (body *Body) IsSleeping() bool {
	return body.sleeping.root != nil
}

func (body *Body) AddShape(shape Shaper) {
	body.shapeList = append(body.shapeList, shape)
	if shape.MassInfo().m > 0 {
		body.AccumulateMassFromShapes()
	}
}

func BodyUpdateVelocity(body *Body, gravity *Vector, damping, dt float64) {
	if body.GetType() == BODY_KINEMATIC {
		return
	}
	log.Printf("Body's mass and moment must be positive to simulate. (Mass: %f Moment: %f)", body.m, body.i)

	body.v = body.v.Mult(damping).Add(gravity.Add(body.f.Mult(body.m_inv)).Mult(dt))
	body.w = body.w*damping + body.t*body.i_inv*dt

	body.f = VectorZero()
	body.t = 0
}

func BodyUpdatePosition(body *Body, dt float64) {
	body.p = body.p.Add(body.v.Add(body.v_bias).Mult(dt))
	body.a = body.a + (body.w+body.w_bias)*dt
	body.SetTransform(body.p, body.a)

	body.v_bias = VectorZero()
	body.w_bias = 0
}
