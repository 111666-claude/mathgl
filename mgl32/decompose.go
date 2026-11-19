// Copyright 2014 The go-gl Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package mgl32

import "math"

// Decompose splits an affine transformation matrix into its translation,
// rotation, signed scale and shear components such that
//
//	m = Translate3D(t) * rot * Scale3D(scale) * Shear3D(shear)
//
// It is equivalent to m.DecomposeEps(Epsilon); see DecomposeEps for the
// full documentation of the decomposition convention and failure modes.
func (m Mat4) Decompose() (t Vec3, rot Quat, scale, shear Vec3, ok bool) {
	return m.DecomposeEps(Epsilon)
}

// DecomposeEps splits an affine transformation matrix into its translation,
// rotation, signed scale and shear components such that
//
//	m = Translate3D(t) * rot * Scale3D(scale) * Shear3D(shear)
//
// within floating point accuracy. ComposeTransform performs the inverse
// operation, so for any matrix m for which DecomposeEps reports ok,
// ComposeTransform applied to the results approximates m.
//
// The returned rot is always a unit quaternion representing a proper
// rotation (its 3x3 matrix has determinant +1). scale holds the signed
// per-axis scale factors and shear holds the (xy, xz, yz) shear factors in
// the convention of Shear3D.
//
// The decomposition is performed by a Gram-Schmidt orthogonalization of the
// column vectors of the upper left 3x3 submatrix. The signs of the scale
// factors are inherently ambiguous: flipping any two of them can be
// compensated by a 180-degree rotation, and flipping all three corresponds
// to a reflection composed with a rotation. This implementation resolves
// the ambiguity by reporting positive scale factors whenever the 3x3
// submatrix has a positive determinant, and by negating all three scale
// factors (adjusting the rotation accordingly) when the determinant is
// negative, so a reflection is reported as three negative scales and rot is
// always a proper rotation. The round trip through ComposeTransform is
// unaffected by these choices.
//
// eps controls how strictly the matrix is required to be affine and
// non-singular. DecomposeEps reports ok == false (and zero values for all
// other results) when
//
//   - any element of m is NaN or infinite,
//   - the bottom row of m differs from (0, 0, 0, 1) by more than eps in any
//     element, i.e. m contains a perspective term and is not affine, or
//   - the length of any orthogonalized column of the 3x3 submatrix is not
//     larger than eps, i.e. m has a zero or near-zero scale factor and the
//     rotation and shear are not uniquely determined.
func (m Mat4) DecomposeEps(eps float32) (t Vec3, rot Quat, scale, shear Vec3, ok bool) {
	for _, e := range m {
		if math.IsNaN(float64(e)) || math.IsInf(float64(e), 0) {
			return Vec3{}, Quat{}, Vec3{}, Vec3{}, false
		}
	}

	// An affine matrix has a bottom row of exactly (0, 0, 0, 1); anything
	// else is a perspective (or otherwise non-affine) term.
	if Abs(m[3]) > eps || Abs(m[7]) > eps || Abs(m[11]) > eps || Abs(m[15]-1) > eps {
		return Vec3{}, Quat{}, Vec3{}, Vec3{}, false
	}

	t = m.Col(3).Vec3()

	// Gram-Schmidt on the columns of the 3x3 submatrix. With
	// M = R * S * K (K upper triangular with unit diagonal), column i of the
	// submatrix is a linear combination of the rotation columns 0..i, so the
	// orthonormalized columns are exactly the columns of R and the
	// normalization factors give S and K.
	c0 := m.Col(0).Vec3()
	c1 := m.Col(1).Vec3()
	c2 := m.Col(2).Vec3()

	sx := c0.Len()
	if !(sx > eps) { // also catches a NaN length
		return Vec3{}, Quat{}, Vec3{}, Vec3{}, false
	}
	u0 := c0.Mul(1 / sx)

	d01 := c1.Dot(u0)
	r1 := c1.Sub(u0.Mul(d01))
	sy := r1.Len()
	if !(sy > eps) {
		return Vec3{}, Quat{}, Vec3{}, Vec3{}, false
	}
	u1 := r1.Mul(1 / sy)

	d02 := c2.Dot(u0)
	d12 := c2.Dot(u1)
	r2 := c2.Sub(u0.Mul(d02)).Sub(u1.Mul(d12))
	sz := r2.Len()
	if !(sz > eps) {
		return Vec3{}, Quat{}, Vec3{}, Vec3{}, false
	}
	u2 := r2.Mul(1 / sz)

	scale = Vec3{sx, sy, sz}
	// c1 = sx*xy*u0 + sy*u1 and c2 = sx*xz*u0 + sy*yz*u1 + sz*u2, so the
	// shear factors are the projections divided by the scale of the axis
	// they multiply.
	shear = Vec3{d01 / sx, d02 / sx, d12 / sy}

	// A negative determinant means the transformation mirrors the
	// coordinate system. Negating all three columns keeps the matrix a
	// proper rotation (det +1) at the cost of negating all scale factors.
	if u0.Dot(u1.Cross(u2)) < 0 {
		scale = scale.Mul(-1)
		u0 = u0.Mul(-1)
		u1 = u1.Mul(-1)
		u2 = u2.Mul(-1)
	}

	rot = Mat4ToQuat(Mat4{
		u0[0], u0[1], u0[2], 0,
		u1[0], u1[1], u1[2], 0,
		u2[0], u2[1], u2[2], 0,
		0, 0, 0, 1,
	}).Normalize()

	return t, rot, scale, shear, true
}

// ComposeTransform builds an affine transformation matrix from its
// translation, rotation, signed scale and shear components:
//
//	ComposeTransform(t, rot, scale, shear) =
//	    Translate3D(t) * rot * Scale3D(scale) * Shear3D(shear)
//
// It is the inverse of Mat4.Decompose. rot is normalized before use, so it
// does not need to be an exact unit quaternion, but it must be non-zero.
// shear holds the (xy, xz, yz) shear factors in the convention of Shear3D;
// pass Vec3{} for no shear.
func ComposeTransform(t Vec3, rot Quat, scale, shear Vec3) Mat4 {
	return Translate3D(t.X(), t.Y(), t.Z()).
		Mul4(rot.Normalize().Mat4()).
		Mul4(Scale3D(scale.X(), scale.Y(), scale.Z())).
		Mul4(Shear3D(shear.X(), shear.Y(), shear.Z()))
}
