package physics

import (
	"testing"
)

type thing struct {
	value string
}

func TestHashSet(t *testing.T) {
	hash := NewHashSet(func(ptr, elt interface{}) bool {
		return ptr.(*thing).value == elt.(*thing).value
	})

	one := &thing{value: "one"}
	data := hash.Insert(HashValue(0), one, nil, one)
	if data.(*thing) != one {
		t.Errorf("Not equal")
	}
	if data.(*thing).value != one.value {
		t.Errorf("Value not equal")
	}

	if hash.Count() != 1 {
		t.Errorf("Count not updated")
	}

	two := &thing{value: "two"}
	data = hash.Insert(HashValue(0), two, nil, two)
	if data.(*thing) != two {
		t.Errorf("Not equal")
	}
	if data.(*thing).value != two.value {
		t.Errorf("Value not equal")
	}

	if hash.Count() != 2 {
		t.Errorf("Count not updated")
	}

	data = hash.Remove(HashValue(0), one)
	if data.(*thing) != one {
		t.Errorf("Not equal")
	}
	if data.(*thing).value != one.value {
		t.Errorf("Value not equal")
	}

	if hash.Count() != 1 {
		t.Errorf("Count not updated")
	}

	data = hash.Remove(HashValue(0), two)
	if data.(*thing) != two {
		t.Errorf("Not equal")
	}
	if data.(*thing).value != two.value {
		t.Errorf("Value not equal")
	}

	if hash.Count() != 0 {
		t.Errorf("Count not updated")
	}
}
