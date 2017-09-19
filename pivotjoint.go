package cp

type PivotJoint struct {
	*Constraint
	AnchorA, AnchorB Vector

	r1, r2 Vector
	k      Mat2x2

	jAcc, bias Vector
}

func NewPivotJoint(a, b *Body, pivot Vector) *Constraint {
	var anchorA Vector
	var anchorB Vector

	if a != nil {
		anchorA = a.WorldToLocal(pivot)
	} else {
		anchorA = pivot.Clone()
	}

	if b != nil {
		anchorB = b.WorldToLocal(pivot)
	} else {
		anchorB = pivot.Clone()
	}

	return NewPivotJoint2(a, b, anchorA, anchorB)
}

func NewPivotJoint2(a, b *Body, anchorA, anchorB Vector) *Constraint {
	joint := &PivotJoint{
		AnchorA: anchorA,
		AnchorB: anchorB,
		jAcc:    Vector{},
	}
	constraint := NewConstraint(joint, a, b)
	joint.Constraint = constraint
	return constraint
}

func (joint *PivotJoint) PreStep(dt float64) {
	a := joint.Constraint.a
	b := joint.Constraint.b

	joint.r1 = a.transform.Vect(joint.AnchorA.Sub(a.cog))
	joint.r2 = b.transform.Vect(joint.AnchorB.Sub(b.cog))

	// Calculate mass tensor
	joint.k = k_tensor(a, b, joint.r1, joint.r2)

	// calculate bias velocity
	delta := b.p.Add(joint.r2).Sub(a.p.Add(joint.r1))
	joint.bias = delta.Mult(-bias_coef(joint.Constraint.errorBias, dt) / dt).Clamp(joint.Constraint.maxBias)
}

func (joint *PivotJoint) ApplyCachedImpulse(dt_coef float64) {
	apply_impulses(joint.a, joint.b, joint.r1, joint.r2, joint.jAcc.Mult(dt_coef))
}

func (joint *PivotJoint) ApplyImpulse(dt float64) {
	a := joint.Constraint.a
	b := joint.Constraint.b

	r1 := joint.r1
	r2 := joint.r2

	// compute relative velocity
	vr := relative_velocity(a, b, r1, r2)

	// compute normal impulse
	j := joint.k.Transform(joint.bias.Sub(vr))
	jOld := joint.jAcc
	joint.jAcc = joint.jAcc.Add(j).Clamp(joint.Constraint.maxForce * dt)
	j = joint.jAcc.Sub(jOld)

	apply_impulses(a, b, joint.r1, joint.r2, j)
}

func (joint *PivotJoint) GetImpulse() float64 {
	return joint.jAcc.Length()
}
