//go:build debug

package cp

import "fmt"

func assert(truth bool, msg ...interface{}) {
	if !truth {
		panic(fmt.Sprint("Assertion failed: ", msg))
	}
}

func assertSpaceUnlocked(space *Space) {
	assert(!space.locked, "This operation cannot be done safely during a call to Space.Step() or during a query. Put these calls into a post-step callback.")
}
