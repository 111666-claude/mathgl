// Copyright 2014 The go-gl Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package mgl32

import (
	"math"
	"testing"
)

// decomposeTolerance is the configurable error bound used for the
// recomposition comparisons. It is deliberately loose enough to hold for
// both mgl32 and mgl64.
const decomposeTolerance = 1e-4

// vec3AbsApprox reports whether got and want are elementwise within tol.
// Unlike ApproxEqualThreshold it uses a plain absolute difference, which
// also works when the expected value is exactly zero.
func vec3AbsApprox(got, want Vec3, tol float32) bool {
	return Abs(got[0]-want[0]) < tol && Abs(got[1]-want[1]) < tol && Abs(got[2]-want[2]) < tol
}

func TestComposeDecomposeRoundTrip(t *testing.T) {
	axis := Vec3{1, 2, 3}.Normalize()
	cases := []struct {
		name  string
		tr    Vec3
		rot   Quat
		scale Vec3
		shear Vec3
	}{
		{"identity", Vec3{}, QuatIdent(), Vec3{1, 1, 1}, Vec3{}},
		{"translation", Vec3{1, -2, 3}, QuatIdent(), Vec3{1, 1, 1}, Vec3{}},
		{"rotation", Vec3{}, QuatRotate(0.7, axis), Vec3{1, 1, 1}, Vec3{}},
		{"uniform scale", Vec3{}, QuatIdent(), Vec3{2.5, 2.5, 2.5}, Vec3{}},
		{"non-uniform scale+rotation", Vec3{4, 5, 6}, QuatRotate(-1.2, axis), Vec3{2, 0.5, 3}, Vec3{}},
		{"negative uniform scale", Vec3{1, 2, 3}, QuatRotate(0.3, axis), Vec3{-2, -2, -2}, Vec3{}},
		{"mirror x+rotation", Vec3{-1, 0, 2}, QuatRotate(1.1, axis), Vec3{-1, 1, 1}, Vec3{}},
		{"two negative scales", Vec3{}, QuatRotate(2.0, axis), Vec3{-3, -0.5, 2}, Vec3{}},
		{"shear", Vec3{}, QuatIdent(), Vec3{1, 1, 1}, Vec3{0.5, -0.25, 0.75}},
		{"rotation+scale+shear", Vec3{}, QuatRotate(0.9, axis), Vec3{1.5, 2.5, 0.5}, Vec3{0.3, 0.1, -0.4}},
		{"full", Vec3{7, -8, 9}, QuatRotate(-2.4, axis), Vec3{-2, 3, 0.75}, Vec3{-0.2, 0.6, 0.1}},
	}

	vertices := []Vec3{
		{0, 0, 0},
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
		{1.5, -2.5, 3.5},
		{-4, 0.25, 7},
	}

	for _, c := range cases {
		m := ComposeTransform(c.tr, c.rot, c.scale, c.shear)

		tr, rot, scale, shear, ok := m.Decompose()
		if !ok {
			t.Errorf("%s: Decompose reported failure", c.name)
			continue
		}

		recomposed := ComposeTransform(tr, rot, scale, shear)
		if !recomposed.ApproxEqualThreshold(m, decomposeTolerance) {
			t.Errorf("%s: recomposition mismatch\noriginal:    %v\nrecomposed:  %v", c.name, m, recomposed)
		}

		// The decomposed and recomposed transforms must move vertices the
		// same way the original matrix does.
		for _, v := range vertices {
			want := m.Mul4x1(v.Vec4(1)).Vec3()
			got := recomposed.Mul4x1(v.Vec4(1)).Vec3()
			if !got.ApproxEqualThreshold(want, decomposeTolerance) {
				t.Errorf("%s: vertex %v transformed to %v, want %v", c.name, v, got, want)
			}
		}

		// The rotation must be a unit quaternion describing a proper
		// rotation (determinant +1).
		if !FloatEqualThreshold(rot.Len(), 1, decomposeTolerance) {
			t.Errorf("%s: rotation quaternion is not unit length: %v", c.name, rot.Len())
		}
		if det := rot.Mat4().Mat3().Det(); !FloatEqualThreshold(det, 1, decomposeTolerance) {
			t.Errorf("%s: rotation has determinant %v, want 1", c.name, det)
		}

		// With all-positive scales the components come back as supplied.
		// (Pairs of negative scales can be absorbed into the rotation as a
		// 180-degree flip, and a reflection is reported by negating all
		// three scales, so in those cases only the round trip is checked.)
		if c.scale[0] > 0 && c.scale[1] > 0 && c.scale[2] > 0 {
			if !vec3AbsApprox(tr, c.tr, decomposeTolerance) {
				t.Errorf("%s: translation %v, want %v", c.name, tr, c.tr)
			}
			if !rot.OrientationEqualThreshold(c.rot, decomposeTolerance) {
				t.Errorf("%s: rotation %v, want %v", c.name, rot, c.rot)
			}
			if !vec3AbsApprox(scale, c.scale, decomposeTolerance) {
				t.Errorf("%s: scale %v, want %v", c.name, scale, c.scale)
			}
			if !vec3AbsApprox(shear, c.shear, decomposeTolerance) {
				t.Errorf("%s: shear %v, want %v", c.name, shear, c.shear)
			}
		}
	}
}

func TestDecomposeReflection(t *testing.T) {
	// A mirror across the YZ plane composed with a rotation has
	// determinant < 0. The reflection sign is ambiguous, so the
	// decomposition reports it by negating all three scale factors; the
	// round trip must still reproduce the original matrix.
	m := QuatRotate(0.6, Vec3{0, 0, 1}.Normalize()).Mat4().Mul4(Scale3D(-1, 1, 1))

	tr, rot, scale, shear, ok := m.Decompose()
	if !ok {
		t.Fatal("Decompose of mirrored matrix failed")
	}
	if !vec3AbsApprox(tr, Vec3{}, decomposeTolerance) {
		t.Errorf("translation %v, want zero", tr)
	}
	if !vec3AbsApprox(shear, Vec3{}, decomposeTolerance) {
		t.Errorf("shear %v, want zero", shear)
	}
	if scale[0] >= 0 || scale[1] >= 0 || scale[2] >= 0 {
		t.Errorf("reflection convention: all scale factors should be negative, got %v", scale)
	}
	if det := rot.Mat4().Mat3().Det(); !FloatEqualThreshold(det, 1, decomposeTolerance) {
		t.Errorf("rotation must stay proper (det 1), got %v", det)
	}
	if recomposed := ComposeTransform(tr, rot, scale, shear); !recomposed.ApproxEqualThreshold(m, decomposeTolerance) {
		t.Errorf("mirror round trip mismatch\noriginal:   %v\nrecomposed: %v", m, recomposed)
	}
}

func TestDecomposePerspective(t *testing.T) {
	// A perspective projection matrix is not affine and must not be
	// decomposed as if it were.
	if _, _, _, _, ok := Perspective(1.0, 1.5, 0.1, 100).Decompose(); ok {
		t.Error("perspective projection matrix wrongly accepted as affine")
	}

	// Bottom row differing from (0, 0, 0, 1) in any element is rejected.
	for _, idx := range []int{3, 7, 11, 15} {
		m := Ident4()
		m[idx] = 0.5
		if _, _, _, _, ok := m.Decompose(); ok {
			t.Errorf("matrix with bottom-row element %d = 0.5 wrongly accepted as affine", idx)
		}
	}
}

func TestDecomposeSingular(t *testing.T) {
	// A zero scale factor makes rotation and shear undetermined.
	if _, _, _, _, ok := Scale3D(0, 1, 1).Decompose(); ok {
		t.Error("zero scale should not decompose")
	}
	if _, _, _, _, ok := Scale3D(1, 0, 1).Decompose(); ok {
		t.Error("zero scale should not decompose")
	}
	if _, _, _, _, ok := Scale3D(1, 1, 0).Decompose(); ok {
		t.Error("zero scale should not decompose")
	}

	// A near-zero scale is rejected with the default epsilon but can be
	// forced through with a smaller, caller-configured one.
	near := Scale3D(1e-9, 1, 1)
	if _, _, _, _, ok := near.DecomposeEps(1e-6); ok {
		t.Error("near-singular matrix should fail with eps 1e-6")
	}
	tr, rot, scale, shear, ok := near.DecomposeEps(1e-12)
	if !ok {
		t.Error("near-singular matrix should decompose with eps 1e-12")
	} else if recomposed := ComposeTransform(tr, rot, scale, shear); !recomposed.ApproxEqualThreshold(near, decomposeTolerance) {
		t.Errorf("near-singular round trip mismatch: %v vs %v", near, recomposed)
	}
}

func TestDecomposeNonFinite(t *testing.T) {
	for _, v := range []float32{float32(math.NaN()), float32(math.Inf(1)), float32(math.Inf(-1))} {
		m := Ident4()
		m[0] = v
		if _, _, _, _, ok := m.Decompose(); ok {
			t.Errorf("matrix with non-finite element %v should not decompose", v)
		}
		m = Ident4()
		m[12] = v
		if _, _, _, _, ok := m.Decompose(); ok {
			t.Errorf("matrix with non-finite translation %v should not decompose", v)
		}
	}
}

func TestDecomposeExistingMatrices(t *testing.T) {
	// Matrices produced by the existing constructors must decompose and
	// recompose to themselves.
	mats := []Mat4{
		Ident4(),
		Translate3D(3, -4, 5),
		HomogRotate3D(0.8, Vec3{1, -1, 2}.Normalize()),
		Scale3D(2, 3, 4),
		Shear3D(0.4, -0.2, 0.9),
		Translate3D(1, 2, 3).Mul4(HomogRotate3DY(1.1)).Mul4(Scale3D(0.5, 2, 1.5)).Mul4(Shear3D(0.1, 0.2, 0.3)),
	}
	for i, m := range mats {
		tr, rot, scale, shear, ok := m.Decompose()
		if !ok {
			t.Errorf("matrix %d failed to decompose", i)
			continue
		}
		if recomposed := ComposeTransform(tr, rot, scale, shear); !recomposed.ApproxEqualThreshold(m, decomposeTolerance) {
			t.Errorf("matrix %d round trip mismatch\noriginal:   %v\nrecomposed: %v", i, m, recomposed)
		}
	}
}
