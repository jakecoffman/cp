//+build debug

package cp

func assert(truth bool, msg ...interface{}) {
	if !truth {
		panic(fmt.Sprint("Assertion failed: ", msg))
	}
}
