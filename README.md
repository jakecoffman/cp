# cp
[![GoDoc](https://godoc.org/github.com/jakecoffman/cp?status.svg)](http://godoc.org/github.com/jakecoffman/cp)

[Chipmunk2D](https://github.com/slembcke/Chipmunk2D) ported to Go

## project status

Beta stage -- most features are implemented and the demos are very close to Chipmunk2D demos.

Some of the API may still change over time.

## run the examples

```
go get github.com/jakecoffman/cp
cd $GOPATH/src/github.com/jakecoffman/cp/examples
go get -d ./...
go run logosmash/logosmash.go
```

## documentation

The official chipmunk docs are a really good place to start: https://chipmunk-physics.net/release/ChipmunkLatest-Docs/

## features

- Designed specifically for 2D video games.
- Circle, convex polygon, and beveled line segment collision primitives.
- Multiple collision primitives can be attached to a single rigid body.
- Fast broad phase collision detection by using a bounding box tree with great temporal coherence or a spatial hash.
- Extremely fast impulse solving by utilizing Erin Catto’s contact persistence algorithm.
- Supports sleeping objects that have come to rest to reduce the CPU load.
- Support for collision event callbacks based on user definable object types types.
- Flexible collision filtering system with layers, exclusion groups and callbacks.
- Can be used to create all sorts of effects like one way platforms or buoyancy areas. (Examples included)
- Supports nearest point, segment (raycasting), shape and bounding box queries to the collision detection system.
- Collision impulses amounts can be retrieved for gameplay effects, sound effects, etc.
- Large variety of joints – easily make vehicles, ragdolls, and more.
- Joint callbacks.
- Can be used to easily implement breakable or animated joints. (Examples included)
- Maintains a contact graph of all colliding objects.
- No external dependencies.
- Many language bindings available.
- Simple, read the documentation and see!
- Unrestrictive MIT license
