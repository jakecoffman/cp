package physics

import "math"

type Space struct {
	iterations int

	gravity Vector
	damping float64

	idleSpeedThreshold float64
	sleepTimeThreshold float64

	collisionSlop        float64
	collisionBias        float64
	collisionPersistence uint

	userData interface{}

	stamp   uint
	curr_dt float64

	dynamicBodies      []Body
	staticBodies       []Body
	rousedBodies       []Body
	sleepingComponents []interface{} // <- IDK what this is

	shapeIDCounter int
	staticShapes   []*Shape
	dynamicShapes  []*Shape

	constraints []Constraint

	arbiters []Arbiter
	//cpContactBufferHeader *contactBuffersHead;
	cachedArbiters map[int]Arbiter
	pooledArbiters []Arbiter

	allocatedBuffers []interface{}
	locked           int

	usesWildcards     bool
	collisionHandlers map[int]CollisionHandler
	defaultHandler    CollisionHandler

	skipPostStep      bool
	postStepCallbacks []PostStepCallback

	staticBody  Body
	_staticBody Body
}

func NewSpace() *Space {
	_staticBody := NewBody(0, 0)
	// TODO More initilization _staticBody
	return &Space{
		iterations:           10,
		gravity:              VectorZero(),
		damping:              1.0,
		collisionSlop:        0.1,
		collisionBias:        math.Pow(0.9, 60),
		collisionPersistence: 3,
		locked:               0,
		stamp:                0,
		shapeIDCounter:       0,
		staticShapes:         []*Shape{}, // TODO: these are actually trees in Chipmunk?
		dynamicShapes:        []*Shape{},
		allocatedBuffers:     []interface{}{},
		dynamicBodies:        []Body{},
		staticBodies:         []Body{},
		sleepingComponents:   []interface{}{},
		rousedBodies:         []Body{},
		sleepTimeThreshold:   math.MaxFloat64,
		idleSpeedThreshold:   0.0,
		arbiters:             []Arbiter{},
		pooledArbiters:       []Arbiter{},
		cachedArbiters:       map[int]Arbiter{},
		constraints:          []Constraint{},
		collisionHandlers:    map[int]CollisionHandler{},
		postStepCallbacks:    []PostStepCallback{},
		_staticBody: _staticBody,
	}
}

type PostStepCallback struct {
	callback PostStepCallbackFunc
	key      interface{}
	data     interface{}
}

type PostStepCallbackFunc func(space *Space, key interface{}, data interface{})
