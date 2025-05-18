//go:build !debug

package cp

func assert(_ bool, _ ...interface{}) {}

func assertSpaceUnlocked(_ *Space) {}
