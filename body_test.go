package physics

import (
	"log"
	"testing"
)

const size = 32

func TestBodyUpdateVelocity(t *testing.T) {
	//space := NewSpace()
	//radius := (&Vector{size, size}).Length()
	//body := space.AddBody(NewBody(1, MomentForBox(1, size, size)))
	////body.SetPosition(&Vector{10, 10})
	//shape := space.AddShape(NewBox(body, size, size, radius))
	//shape.E = 0
	//shape.U = 5
	//
	//for i := 0; i < 100; i++ {
	//	log.Println(body.Position(), body.Angle()*DegreeConst)
	//	body.SetAngularVelocity(-0.1)
	//	//BodyUpdateVelocity(body, VectorZero(), 1, 0.1)
	//	//BodyUpdatePosition(body, 0.1)
	//	space.Step(0.016)
	//}


	body := NewBody(1, MomentForBox(1, size, size))
	body.SetPosition(&Vector{10, 10})

	for i := 0; i < 100; i++ {
		log.Println(body.Position(), body.Angle()*DegreeConst)
		body.SetAngularVelocity(-0.1)
		//BodyUpdateVelocity(body, VectorZero(), 1, 0.1)
		BodyUpdatePosition(body, 0.1)
		///space.Step(0.016)
	}
}
