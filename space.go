package physics

import (
	"log"
	"math"
)

type Space struct {
	Iterations uint // must be non-zero

	gravity *Vector
	damping float64

	idleSpeedThreshold float64
	SleepTimeThreshold float64

	collisionSlop        float64
	collisionBias        float64
	collisionPersistence uint

	userData interface{}

	stamp   uint
	curr_dt float64

	dynamicBodies      []*Body
	staticBodies       []*Body
	rousedBodies       []*Body
	sleepingComponents []interface{} // <- IDK what this is

	shapeIDCounter int
	staticShapes   map[int]Shaper
	dynamicShapes  map[int]Shaper

	constraints []*Constraint

	arbiters []*Arbiter
	//cpContactBufferHeader *contactBuffersHead;
	cachedArbiters map[int]*Arbiter
	pooledArbiters []*Arbiter

	allocatedBuffers []interface{}
	locked           int

	usesWildcards     bool
	collisionHandlers map[int]*CollisionHandler
	defaultHandler    CollisionHandler

	skipPostStep      bool
	postStepCallbacks []PostStepCallback

	*Body // staticBody
}

func NewSpace() *Space {
	staticBody := NewBody(0, 0)
	staticBody.SetType(BODY_STATIC)
	space := &Space{
		Iterations:           10,
		gravity:              VectorZero(),
		damping:              1.0,
		collisionSlop:        0.1,
		collisionBias:        math.Pow(0.9, 60),
		collisionPersistence: 3,
		locked:               0,
		stamp:                0,
		shapeIDCounter:       0,
		staticShapes:         map[int]Shaper{},
		dynamicShapes:        map[int]Shaper{},
		allocatedBuffers:     []interface{}{},
		dynamicBodies:        []*Body{},
		staticBodies:         []*Body{},
		sleepingComponents:   []interface{}{},
		rousedBodies:         []*Body{},
		SleepTimeThreshold:   math.MaxFloat64,
		idleSpeedThreshold:   0.0,
		arbiters:             []*Arbiter{},
		pooledArbiters:       []*Arbiter{},
		cachedArbiters:       map[int]*Arbiter{},
		constraints:          []*Constraint{},
		collisionHandlers:    map[int]*CollisionHandler{},
		postStepCallbacks:    []PostStepCallback{},
	}
	space.SetStaticBody(staticBody)
	return space
}

func (space *Space) SetStaticBody(body *Body) {
	if space.Body != nil {
		log.Println("Internal Error: Changing the designated static body while the old one still had shapes attached.")
		space.Body.space = nil
	}
	space.Body = body
	body.space = space
}

func (space *Space) Activate(body *Body) {
	if space.locked > 0 {
		if !Contains(space.rousedBodies, body) {
			space.rousedBodies = append(space.rousedBodies, body)
		}
		return
	}

	space.dynamicBodies = append(space.dynamicBodies, body)

	for _, shape := range body.shapeList {
		delete(space.staticShapes, shape.HashId())
		space.dynamicShapes[shape.HashId()] = shape
	}

	for _, arbiter := range body.arbiterList {
		bodyA := arbiter.body_a

		// Arbiters are shared between two bodies that are always woken up together.
		// You only want to restore the arbiter once, so bodyA is arbitrarily chosen to own the arbiter.
		// The edge case is when static bodies are involved as the static bodies never actually sleep.
		// If the static body is bodyB then all is good. If the static body is bodyA, that can easily be checked.
		if body == bodyA || bodyA.GetType() == BODY_STATIC {
			panic("Not implemented") // TODO need to figure out what contact buffer array stuff is
		}
	}

	for _, constraint := range body.constraintList {
		if body == constraint.a || constraint.a.GetType() == BODY_STATIC {
			space.constraints = append(space.constraints, constraint)
		}
	}
}

type PostStepCallback struct {
	callback PostStepCallbackFunc
	key      interface{}
	data     interface{}
}

type PostStepCallbackFunc func(space *Space, key interface{}, data interface{})

func Contains(bodies []*Body, body *Body) bool {
	for i := 0; i < len(bodies); i++ {
		if bodies[i] == body {
			return true
		}
	}
	return false
}

func (space *Space) AddShape(shape Shaper) Shaper {
	var body *Body = shape.Body()

	// TODO assertions

	isStatic := body.GetType() == BODY_STATIC
	if !isStatic {
		body.Activate()
	}

	body.AddShape(shape)

	space.shapeIDCounter += 1
	shape.SetHashId(space.shapeIDCounter)

	if isStatic {
		space.staticShapes[shape.HashId()] = shape
	} else {
		space.dynamicShapes[shape.HashId()] = shape
	}

	shape.SetSpace(space)
	return shape
}
