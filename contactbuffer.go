package physics

type ContactBuffer struct {
	// header
	stamp       uint
	next        *ContactBuffer
	numContacts uint

	// buffer itself
	contacts [CONTACTS_BUFFER_SIZE]*Contact
}

func NewContactBuffer(stamp uint, slice *ContactBuffer) *ContactBuffer {
	buffer := &ContactBuffer{}
	buffer.contacts = [CONTACTS_BUFFER_SIZE]*Contact{}
	for i := range buffer.contacts {
		buffer.contacts[i] = &Contact{}
	}
	return buffer.InitHeader(stamp, slice)
}

func (c *ContactBuffer) InitHeader(stamp uint, splice *ContactBuffer) *ContactBuffer {
	c.stamp = stamp
	if splice != nil {
		c.next = splice.next
	} else {
		c.next = c
	}
	c.numContacts = 0

	return c
}
