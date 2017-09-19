package cp

import (
	"testing"
	"unsafe"
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

func TestHashValue(t *testing.T) {
	a := NewCircle(NewBody(1, MomentForCircle(1, 0, 1, Vector{})), 1, Vector{})
	b := NewCircle(NewBody(2, MomentForCircle(2, 0, 2, Vector{})), 2, Vector{})
	z := NewCircle(NewBody(2, MomentForCircle(2, 0, 2, Vector{})), 2, Vector{})

	arbHashId := HashPair(HashValue(unsafe.Pointer(a)), HashValue(unsafe.Pointer(b)))

	c := a
	d := b

	arbHashId2 := HashPair(HashValue(unsafe.Pointer(c)), HashValue(unsafe.Pointer(d)))

	if arbHashId != arbHashId2 {
		t.Errorf("FAIL")
	}

	arbHashId3 := HashPair(HashValue(unsafe.Pointer(c)), HashValue(unsafe.Pointer(z)))

	if arbHashId2 == arbHashId3 {
		t.Errorf("Should not be equal")
	}
}
