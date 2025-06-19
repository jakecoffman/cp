package cp

import "slices"

// SpaceArbiterSetFilter throws away old arbiters.
func SpaceArbiterSetFilter(arb *Arbiter, space *Space) bool {
	// TODO: should make an arbiter state for this so it doesn't require filtering arbiters for dangling body pointers on body removal.
	// Preserve arbiters on sensors and rejected arbiters for sleeping objects.
	// This prevents errant separate callbacks from happening.

	a := arb.body_a
	b := arb.body_b

	if (a.GetType() == BODY_STATIC || a.IsSleeping()) && (b.GetType() == BODY_STATIC || b.IsSleeping()) {
		return true
	}

	ticks := space.stamp - arb.stamp

	if ticks >= 1 && arb.state != CP_ARBITER_STATE_CACHED {
		arb.state = CP_ARBITER_STATE_CACHED
		handler := arb.handler
		handler.SeparateFunc(arb, space, handler.UserData)
	}

	if ticks >= space.collisionPersistence {
		arb.contacts = nil
		arb.count = 0
		space.pooledArbiters.Put(arb)
		return false
	}

	return true
}

func CachedArbitersFilter(arb *Arbiter, space *Space, shape *Shape, body *Body) bool {
	// Match on the filter shape, or if it's NULL the filter body
	if (body == arb.body_a && (shape == arb.a || shape == nil)) ||
		(body == arb.body_b && (shape == arb.b || shape == nil)) {
		// Call separate when removing shapes.
		if shape != nil && arb.state != CP_ARBITER_STATE_CACHED {
			// Invalidate the arbiter since one of the shapes was removed
			arb.state = CP_ARBITER_STATE_INVALIDATED

			handler := arb.handler
			handler.SeparateFunc(arb, space, handler.UserData)
		}

		arb.Unthread()

		if index := slices.Index(space.arbiters, arb); index != -1 {
			space.arbiters = slices.Delete(space.arbiters, index, index+1)
		}

		space.pooledArbiters.Put(arb)
		return false
	}

	return true
}
