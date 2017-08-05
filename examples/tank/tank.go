package main

import "github.com/jakecoffman/physics"

func main() {
	var space *physics.Space = physics.NewSpace()
	space.Iterations = 10
	space.SleepTimeThreshold = 0.5


}
