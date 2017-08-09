package physics

type ContactBufferHeader struct {
	stamp       uint
	next        *ContactBufferHeader
	numContacts uint
	buffer      *ContactBuffer
}

type ContactBuffer struct {
	header   *ContactBufferHeader
	contacts [CONTACTS_BUFFER_SIZE]*Contact
}

func (*ContactBufferHeader) Init(stamp uint, splice *ContactBufferHeader) *ContactBufferHeader {
	header := &ContactBufferHeader{
		buffer: &ContactBuffer{},
	}
	header.stamp = stamp
	if splice != nil {
		header.next = splice.next
	} else {
		header.next = header
	}
	header.numContacts = 0
	return header
}
