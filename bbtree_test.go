package cp

import "testing"

func TestBBTree_GetMasterTree(t *testing.T) {
	bbTree := &BBTree{}
	node := bbTree.NodeFromPool()

	if node.parent == nil {
		t.Fatal()
	}

	count := 1

	for {
		if node.parent != nil {
			node = node.parent
			count++
		} else {
			break
		}
	}

	for i := 1; i < count*2; i++ {
		node = bbTree.NodeFromPool()
	}

}
