// This file is generated from mgl32/frustum_test.go; DO NOT EDIT

// Copyright 2014 The go-gl Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package mgl64

import (
	"math"
	"testing"
)

// approxPlane reports whether two planes describe the same oriented plane.
func approxPlane(a, b Plane, threshold float64) bool {
	return a.Normal.ApproxEqualThreshold(b.Normal, threshold) && FloatEqualThreshold(a.Offset, b.Offset, threshold)
}

func approxFrustum(a, b ViewFrustum, threshold float64) bool {
	for i := range a.Planes {
		if !approxPlane(a.Planes[i], b.Planes[i], threshold) {
			return false
		}
	}
	return true
}

func mustViewFrustum(t *testing.T, vp Mat4) ViewFrustum {
	t.Helper()
	f, err := ViewFrustumFromMatrix(vp)
	if err != nil {
		t.Fatalf("ViewFrustumFromMatrix returned error: %v", err)
	}
	return f
}

// The view-projection matrices below are the ones specified by the OpenGL
// specification for the standard frustum; the planes they imply are computed
// by hand from those definitions, not by the code under test.
const planeThreshold = 1e-5

func TestViewFrustumFromMatrixIdentity(t *testing.T) {
	t.Parallel()

	// Identity: the visible volume is the cube [-1,1]^3. Its faces are the
	// six axis-aligned planes, with normals pointing inwards.
	f := mustViewFrustum(t, Ident4())

	expected := ViewFrustum{[6]Plane{
		{Normal: Vec3{1, 0, 0}, Offset: 1},  // left:   x >= -1
		{Normal: Vec3{-1, 0, 0}, Offset: 1}, // right:  x <= 1
		{Normal: Vec3{0, 1, 0}, Offset: 1},  // bottom: y >= -1
		{Normal: Vec3{0, -1, 0}, Offset: 1}, // top:    y <= 1
		{Normal: Vec3{0, 0, 1}, Offset: 1},  // near:   z >= -1
		{Normal: Vec3{0, 0, -1}, Offset: 1}, // far:    z <= 1
	}}

	if !approxFrustum(f, expected, planeThreshold) {
		t.Fatalf("planes from identity = %v, want %v", f.Planes, expected.Planes)
	}

	// The six planes must be usable through the PlaneIndex constants.
	if got := f.Planes[PlaneLeft].Normal; got != (Vec3{1, 0, 0}) {
		t.Errorf("PlaneLeft normal = %v, want (1,0,0)", got)
	}
	if got := f.Planes[PlaneFar].Normal; got != (Vec3{0, 0, -1}) {
		t.Errorf("PlaneFar normal = %v, want (0,0,-1)", got)
	}
}

func TestViewFrustumPlaneDistances(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name   string
		vp     Mat4
		inside Vec3
		// distance is the distance from inside to each plane, in the order
		// left, right, bottom, top, near, far.
		distance [6]float64
		// behind is a point in front of the near plane; behindAll reports
		// whether it is also on all of the side planes (the perspective apex).
		behind    Vec3
		behindAll bool
	}{
		{
			// 90 degree vertical FOV, square aspect, near 1, far 3. At z = -2
			// the view volume is the square [-2,2]^2. The left plane is
			// x - z = 0 and the near plane is z = -1.
			name:   "perspective",
			vp:     Perspective(DegToRad(90), 1, 1, 3),
			inside: Vec3{0, 0, -2},
			// The side planes pass through (-2, 0, -2): 2/sqrt(2) = sqrt(2)
			// from the axis. The near plane is at z = -1 and the far at z = -3.
			distance: [6]float64{float64(math.Sqrt2), float64(math.Sqrt2), float64(math.Sqrt2), float64(math.Sqrt2), 1, 1},
			// A point just outside the near plane but within the side planes.
			behind:    Vec3{0, 0, -0.5},
			behindAll: false,
		},
		{
			// Orthographic projection to the box [-2,2] x [-3,3] x [-1,-5].
			name:   "orthographic",
			vp:     Ortho(-2, 2, -3, 3, 1, 5),
			inside: Vec3{0, 0, -3},
			// Side planes are at x = ±2 and y = ±3; near is z = -1, far z = -5.
			distance: [6]float64{2, 2, 3, 3, 2, 2},
			// With an orthographic projection a point in front of the near plane
			// is still on the inside of the side planes.
			behind:    Vec3{0, 0, 0},
			behindAll: false,
		},
		{
			// With a perspective projection moving straight to the eye (the
			// apex) puts the point on every side plane at once.
			name:      "perspective at the eye",
			vp:        Perspective(DegToRad(90), 1, 1, 3),
			inside:    Vec3{0, 0, -2},
			distance:  [6]float64{float64(math.Sqrt2), float64(math.Sqrt2), float64(math.Sqrt2), float64(math.Sqrt2), 1, 1},
			behind:    Vec3{0, 0, 0},
			behindAll: true,
		},
	}

	for _, c := range tests {
		c := c
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()

			f := mustViewFrustum(t, c.vp)

			for i, plane := range f.Planes {
				if l := plane.Normal.Len(); !FloatEqualThreshold(l, 1, 1e-5) {
					t.Errorf("plane %d (%v) normal length = %v, want 1", i, plane.Normal, l)
				}

				if d := plane.SignedDistance(c.inside); !FloatEqualThreshold(d, c.distance[i], 1e-4) {
					t.Errorf("plane %d distance from %v = %v, want %v", i, c.inside, d, c.distance[i])
				}
			}

			// Moving the point outside through the near plane makes exactly the
			// near plane's distance negative and leaves the other planes
			// strictly positive, except at the perspective apex, where the four
			// side planes meet and the far plane is still ahead.
			if d := f.Planes[PlaneNear].SignedDistance(c.behind); !(d < 0) {
				t.Errorf("distance from a point in front of the near plane = %v, want < 0", d)
			}
			for i, plane := range f.Planes {
				switch {
				case PlaneIndex(i) == PlaneNear:
					continue
				case c.behindAll && PlaneIndex(i) == PlaneFar:
					if d := plane.SignedDistance(c.behind); !(d > 0) {
						t.Errorf("plane %d: distance at the eye toward the far plane = %v, want > 0", i, d)
					}
				case c.behindAll:
					if d := plane.SignedDistance(c.behind); d != 0 {
						t.Errorf("plane %d: distance at the eye = %v, want 0", i, d)
					}
				default:
					if d := plane.SignedDistance(c.behind); !(d > 0) {
						t.Errorf("plane %d: distance from a point in front of the near plane = %v, want > 0", i, d)
					}
				}
			}
		})
	}
}

func TestViewFrustumPerspectiveTangentPlanes(t *testing.T) {
	t.Parallel()

	// The six planes of the 90 degree FOV / square aspect / near 1 / far 3
	// projection, expressed by their plane equations and normalized.
	s2 := float64(math.Sqrt2)
	f := mustViewFrustum(t, Perspective(DegToRad(90), 1, 1, 3))

	expected := [6]Plane{
		{Normal: Vec3{1 / s2, 0, -1 / s2}, Offset: 0},  // x - z = 0
		{Normal: Vec3{-1 / s2, 0, -1 / s2}, Offset: 0}, // -x - z = 0
		{Normal: Vec3{0, 1 / s2, -1 / s2}, Offset: 0},  // y - z = 0
		{Normal: Vec3{0, -1 / s2, -1 / s2}, Offset: 0}, // -y - z = 0
		{Normal: Vec3{0, 0, -1}, Offset: -1},           // -z - 1 = 0, i.e. z = -1
		{Normal: Vec3{0, 0, 1}, Offset: 3},             // z + 3 = 0, i.e. z = -3
	}

	if !approxFrustum(f, ViewFrustum{expected}, 1e-4) {
		t.Fatalf("planes = %v, want %v", f.Planes, expected)
	}
}

func TestViewFrustumCameraTransform(t *testing.T) {
	t.Parallel()

	// A camera positioned at (5, -2, 3) looking at the origin with an
	// orthographic projection. The view-projection matrix is
	// projection.Mul4(view), the same composition Project and UnProject use;
	// the frustum must follow it exactly.
	eye := Vec3{5, -2, 3}
	center := Vec3{0, 0, 0}
	up := Vec3{0, 1, 0}

	view := LookAtV(eye, center, up)
	proj := Ortho(-2, 2, -2, 2, 1, 10)
	f := mustViewFrustum(t, proj.Mul4(view))

	// A point at the center of the view volume is the projection center,
	// 5.5 units in front of the eye (midway between near and far).
	forward := center.Sub(eye).Normalize()
	mid := eye.Add(forward.Mul(5.5))
	// The side planes are 2 units from the axis; the near and far planes are
	// 4.5 units from the midpoint (the volume is 9 units deep).
	expectedDistance := [6]float64{2, 2, 2, 2, 4.5, 4.5}
	for i, plane := range f.Planes {
		if d := plane.SignedDistance(mid); !FloatEqualThreshold(d, expectedDistance[i], 1e-4) {
			t.Errorf("plane %d distance from view center = %v, want %v", i, d, expectedDistance[i])
		}
	}

	// The eye itself is outside (in front of the near plane).
	if f.ContainsPoint(eye) {
		t.Error("frustum contains the camera eye, which is in front of the near plane")
	}

	// A point on the axis just past the far plane is outside.
	if f.ContainsPoint(eye.Add(forward.Mul(11))) {
		t.Error("frustum contains a point past the far plane")
	}

	// A point 1.5 units in front of the eye is inside (near is 1, far is 10).
	if !f.ContainsPoint(eye.Add(forward.Mul(1.5))) {
		t.Error("frustum does not contain a point just inside the near plane")
	}

	// Compare against the same frustum built from the equivalent rotated but
	// un-translated camera: translating a camera translates its frustum, so
	// the normals are unchanged and each plane offset becomes offset - n . eye.
	viewAtOrigin := LookAtV(Vec3{0, 0, 0}, forward, up)
	fOrigin := mustViewFrustum(t, proj.Mul4(viewAtOrigin))

	for i := range f.Planes {
		want := fOrigin.Planes[i].Offset - fOrigin.Planes[i].Normal.Dot(eye)
		if !FloatEqualThreshold(f.Planes[i].Offset, want, 1e-4) {
			t.Errorf("plane %d offset = %v, want %v after translating the camera by %v", i, f.Planes[i].Offset, want, eye)
		}
		if !f.Planes[i].Normal.ApproxEqualThreshold(fOrigin.Planes[i].Normal, 1e-5) {
			t.Errorf("plane %d normal = %v, want %v", i, f.Planes[i].Normal, fOrigin.Planes[i].Normal)
		}
	}
}

func TestViewFrustumCullingPoints(t *testing.T) {
	t.Parallel()

	// Camera at the origin looking down -Z with a 90 degree FOV, near 1,
	// far 3. The view volume is the pyramid between z = -1 and z = -3.
	f := mustViewFrustum(t, Perspective(DegToRad(90), 1, 1, 3))

	tests := []struct {
		name  string
		p     Vec3
		point bool
		box   bool // an infinitesimally small box at p has the same answer
	}{
		{"origin behind near plane", Vec3{0, 0, 0}, false, false},
		{"between eye and near plane", Vec3{0, 0, -0.5}, false, false},
		{"in front of far plane", Vec3{0, 0, -4}, false, false},
		{"inside at center", Vec3{0, 0, -2}, true, true},
		{"inside near far plane", Vec3{0, 0, -2.9}, true, true},
		{"inside near near plane", Vec3{0, 0, -1.1}, true, true},
		{"outside left", Vec3{-2.5, 0, -2}, false, false},
		{"outside right", Vec3{2.5, 0, -2}, false, false},
		{"outside top", Vec3{0, 2.5, -2}, false, false},
		{"outside bottom", Vec3{0, -2.5, -2}, false, false},
		{"diagonally outside", Vec3{2.5, 2.5, -2}, false, false},
		// Points exactly on the boundary count as visible.
		{"on near plane", Vec3{0, 0, -1}, true, true},
		{"on far plane", Vec3{0, 0, -3}, true, true},
		{"on left plane", Vec3{-2, 0, -2}, true, true},
		{"on right plane", Vec3{2, 0, -2}, true, true},
		{"on top plane", Vec3{0, 2, -2}, true, true},
	}

	for _, c := range tests {
		c := c
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()

			if got := f.ContainsPoint(c.p); got != c.point {
				t.Errorf("ContainsPoint(%v) = %v, want %v", c.p, got, c.point)
			}

			box := AABB{Min: c.p, Max: c.p}
			if got := f.IntersectsBox(box); got != c.box {
				t.Errorf("IntersectsBox(%v) = %v, want %v", box, got, c.box)
			}
			if got := f.ContainsBox(box); got != c.box {
				t.Errorf("ContainsBox(%v) = %v, want %v", box, got, c.box)
			}
		})
	}
}

func TestViewFrustumSpheres(t *testing.T) {
	t.Parallel()

	f := mustViewFrustum(t, Perspective(DegToRad(90), 1, 1, 3))
	s2 := float64(math.Sqrt2)

	tests := []struct {
		name       string
		center     Vec3
		radius     float64
		intersects bool
		contains   bool
	}{
		{"well inside", Vec3{0, 0, -2}, 0.5, true, true},
		{"large sphere containing frustum", Vec3{0, 0, -2}, 10, true, false},
		{"outside left, far", Vec3{-10, 0, -2}, 1, false, false},
		{"outside left, overlapping", Vec3{-3, 0, -2}, 1, true, false},
		{"outside left, clear", Vec3{-10, 0, -2}, 1, false, false},
		{"behind near plane, touching", Vec3{0, 0, 0}, 1, true, false},
		{"behind near plane, clear", Vec3{0, 0, 0}, 0.5, false, false},
		{"past far plane, touching", Vec3{0, 0, -4}, 1, true, false},
		{"past far plane, clear", Vec3{0, 0, -4}, 0.5, false, false},
		// Straddling the near plane: intersects but is not contained.
		{"straddling near plane", Vec3{0, 0, -0.5}, 1, true, false},
		// The left plane is x - z = 0, so the distance from (x, 0, z) is
		// (x - z) / sqrt(2). At z = -2 the centre x = -2 + 0.5*sqrt(2) is
		// exactly 0.5 from the plane: the sphere is inside and tangent.
		{"touching side from inside", Vec3{-2 + 0.5*s2, 0, -2}, 0.5, true, true},
		// The same centre with a slightly larger radius sticks out of the left
		// plane: it still intersects the frustum but is not contained.
		{"straddling side plane", Vec3{-2 + 0.5*s2, 0, -2}, 0.6, true, false},
		// A sphere well outside the left plane must still be rejected: the
		// boundary tolerance is tiny (1e-6 relative), not a cull-everything
		// fudge.
		{"clearly outside left", Vec3{-2.75, 0, -2}, 0.5, false, false},
	}

	for _, c := range tests {
		c := c
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()

			if got := f.IntersectsSphere(c.center, c.radius); got != c.intersects {
				t.Errorf("IntersectsSphere(%v, %v) = %v, want %v", c.center, c.radius, got, c.intersects)
			}
			if got := f.ContainsSphere(c.center, c.radius); got != c.contains {
				t.Errorf("ContainsSphere(%v, %v) = %v, want %v", c.center, c.radius, got, c.contains)
			}
		})
	}
}

func TestViewFrustumBoxes(t *testing.T) {
	t.Parallel()

	f := mustViewFrustum(t, Perspective(DegToRad(90), 1, 1, 3))

	tests := []struct {
		name       string
		box        AABB
		intersects bool
		contains   bool
	}{
		{"fully inside", AABB{Vec3{-0.5, -0.5, -2.5}, Vec3{0.5, 0.5, -1.5}}, true, true},
		{"containing frustum", AABB{Vec3{-10, -10, -10}, Vec3{10, 10, 10}}, true, false},
		{"outside left", AABB{Vec3{-10, -1, -2.5}, Vec3{-9, 1, -1.5}}, false, false},
		{"outside top", AABB{Vec3{-1, 10, -2.5}, Vec3{1, 11, -1.5}}, false, false},
		{"behind camera", AABB{Vec3{-1, -1, -0.6}, Vec3{1, 1, -0.5}}, false, false},
		{"past far plane", AABB{Vec3{-1, -1, -10}, Vec3{1, 1, -9}}, false, false},
		// The box only reaches the left plane at the far end (its nearest point
		// to the plane is on that plane), so it counts as intersecting but not
		// contained.
		{"touching left plane", AABB{Vec3{-3, -1, -3}, Vec3{-2, 1, -1}}, true, false},
		// The box straddles the near plane.
		{"straddling near plane", AABB{Vec3{-1, -1, -2}, Vec3{1, 1, 0}}, true, false},
		// Diagonal corners: the box is conservative and still accepted even
		// though it is entirely beyond the frustum's corner in the strict
		// sense; the plane test must not reject it.
		{"diagonal outside of side corner", AABB{Vec3{1.9, 1.9, -2.1}, Vec3{2.1, 2.1, -1.9}}, true, false},
	}

	for _, c := range tests {
		c := c
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()

			if got := f.IntersectsBox(c.box); got != c.intersects {
				t.Errorf("IntersectsBox(%v) = %v, want %v", c.box, got, c.intersects)
			}
			if got := f.ContainsBox(c.box); got != c.contains {
				t.Errorf("ContainsBox(%v) = %v, want %v", c.box, got, c.contains)
			}
		})
	}
}

func TestViewFrustumBoxesCarveFrustum(t *testing.T) {
	t.Parallel()

	// A box that exactly bounds the whole frustum, the AABB of the far plane
	// square at z = -3: [-3,3] x [-3,3] x [-3,-1]. Every point of the frustum
	// lies in the box, so the frustum's planes must accept it.
	f := mustViewFrustum(t, Perspective(DegToRad(90), 1, 1, 3))
	bounds := AABB{Min: Vec3{-3, -3, -3}, Max: Vec3{3, 3, -1}}

	if !f.IntersectsBox(bounds) {
		t.Error("IntersectsBox rejected a box bounding the whole frustum")
	}
	if f.ContainsBox(bounds) {
		t.Error("ContainsBox accepted a box that extends outside the frustum")
	}

	// Every vertex of the far plane square is on the boundary.
	for _, x := range []float64{-3, 3} {
		for _, y := range []float64{-3, 3} {
			if !f.ContainsPoint(Vec3{x, y, -3}) {
				t.Errorf("frustum does not contain its own corner (%v, %v, -3)", x, y)
			}
		}
	}
}

func TestViewFrustumCullingMatchesProjection(t *testing.T) {
	t.Parallel()

	// Verify the plane tests against the original matrix: a point is inside
	// the frustum exactly when its clip coordinates satisfy the OpenGL
	// -w <= x, y, z <= w inequalities, including the OpenGL convention that
	// the near and far planes both lie at negative eye-space z.
	eye := Vec3{2, 3, 4}
	center := Vec3{-1, 0, 2}
	up := Vec3{0, 1, 0}
	vp := Perspective(DegToRad(60), 4.0/3.0, 0.5, 50).Mul4(LookAtV(eye, center, up))

	f := mustViewFrustum(t, vp)

	insideClip := func(p Vec3) bool {
		c := vp.Mul4x1(p.Vec4(1))
		return -c[3] <= c[0] && c[0] <= c[3] &&
			-c[3] <= c[1] && c[1] <= c[3] &&
			-c[3] <= c[2] && c[2] <= c[3]
	}

	// Sample a grid of points around the frustum, including points on and
	// around every boundary. Points within the rounding band of a boundary
	// are not compared: the frustum predicates deliberately accept those.
	checked := 0
	for xi := -16; xi <= 16; xi++ {
		for yi := -16; yi <= 16; yi++ {
			for zi := -16; zi <= 16; zi++ {
				p := Vec3{float64(xi) / 2, float64(yi) / 2, float64(zi) / 2}
				ambiguous := false
				for _, plane := range f.Planes {
					if Abs(plane.SignedDistance(p)) < 1e-4 {
						ambiguous = true
						break
					}
				}
				if ambiguous {
					continue
				}

				checked++
				want := insideClip(p)
				if got := f.ContainsPoint(p); got != want {
					t.Fatalf("ContainsPoint(%v) = %v, clip test = %v", p, got, want)
				}
			}
		}
	}

	if checked < 10000 {
		t.Fatalf("only %d points were compared; the sample grid is too sparse", checked)
	}
}

func TestViewFrustumCameraRotation(t *testing.T) {
	t.Parallel()

	// Rotating the camera must rotate the planes with it: a frustum built
	// from a camera looking down -Z, rotated by yaw and pitch, has plane
	// normals equal to the original normals rotated by the same rotation.
	proj := Perspective(DegToRad(40), 16.0/9.0, 0.1, 100)
	rotation := Rotate3DY(DegToRad(35)).Mul3(Rotate3DX(DegToRad(-20)))

	rotation4 := rotation.Mat4()
	vp := proj.Mul4(rotation4.Inv())
	f := mustViewFrustum(t, vp)
	fBase := mustViewFrustum(t, proj)

	for i := range f.Planes {
		wantNormal := rotation.Mul3x1(fBase.Planes[i].Normal)
		// Use an absolute tolerance: some components of the expected normal
		// are exactly zero, which makes a relative comparison meaningless.
		if diff := f.Planes[i].Normal.Sub(wantNormal).Len(); diff > 1e-4 {
			t.Errorf("plane %d normal = %v, want %v (differs by %v)", i, f.Planes[i].Normal, wantNormal, diff)
		}
		// A pure rotation about the eye leaves the distance from the eye
		// unchanged: the offsets are identical up to the precision of the
		// inverse used to build the view matrix, which scales with the far
		// plane distance.
		if diff := Abs(f.Planes[i].Offset - fBase.Planes[i].Offset); diff > 1e-4*Abs(fBase.Planes[i].Offset) {
			t.Errorf("plane %d offset = %v, want %v (differs by %v)", i, f.Planes[i].Offset, fBase.Planes[i].Offset, diff)
		}
	}

	// Concretely: the camera now looks 35 degrees to the right and 20 degrees
	// down, so a point 10 units along the new forward axis is inside while the
	// point 10 units down the original -Z axis is outside it.
	forward := rotation.Mul3x1(Vec3{0, 0, -1})
	if !f.ContainsPoint(forward.Mul(10)) {
		t.Errorf("frustum does not contain %v, 10 units along the camera forward axis", forward.Mul(10))
	}
	if f.ContainsPoint(Vec3{0, 0, -10}) {
		t.Error("frustum still contains the old forward axis after rotating the camera")
	}

	// The distance of the rotated axis point from each plane equals the
	// distance of the unrotated axis point from the corresponding unrotated
	// plane.
	for i, plane := range f.Planes {
		want := fBase.Planes[i].SignedDistance(Vec3{0, 0, -10})
		if d := plane.SignedDistance(forward.Mul(10)); !FloatEqualThreshold(d, want, 1e-4) {
			t.Errorf("plane %d distance along the rotated axis = %v, want %v", i, d, want)
		}
	}
}

func TestViewFrustumErrors(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name string
		vp   Mat4
		want error
	}{
		{"zero matrix", Mat4{}, ErrNonInvertibleMatrix},
		{"singular", Mat4{1, 2, 3, 4, 2, 4, 6, 8, 0, 0, 1, 0, 0, 0, 0, 1}, ErrNonInvertibleMatrix},
		{"nan", Mat4{1, 0, 0, 0, 0, NaN, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, ErrNonFiniteMatrix},
		{"inf", Mat4{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, InfPos, 1}, ErrNonFiniteMatrix},
		// A projection of rank 3: the bottom row is a multiple of the top row,
		// so right - left cancels to a plane with a zero normal even though the
		// matrix has a well-defined (zero) determinant.
		{"degenerate right plane", Mat4{0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, ErrNonInvertibleMatrix},
		// This matrix is invertible, but its top row negates the bottom row in
		// x, y and z, so the left plane's normal (bottom + top row) cancels to
		// zero while only its constant term remains.
		{"zero normal with invertible matrix", Mat4{-1, 0, 0, 1, -1, 1, 0, 1, -1, 0, 1, 1, 0, 0, 0, 1}, ErrDegeneratePlane},
	}

	for _, c := range tests {
		c := c
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()

			f, err := ViewFrustumFromMatrix(c.vp)
			if err != c.want {
				t.Errorf("ViewFrustumFromMatrix(%v) error = %v, want %v", c.vp, err, c.want)
			}
			if f != (ViewFrustum{}) {
				t.Errorf("ViewFrustumFromMatrix returned a frustum with an error: %v", f)
			}
		})
	}
}

func TestViewFrustumInvalidInput(t *testing.T) {
	t.Parallel()

	f := mustViewFrustum(t, Ident4())

	nan := Vec3{NaN, 0, 0}
	inf := Vec3{InfPos, 0, 0}

	// Non-finite points and radii are unclassifiable; culling errs on the side
	// of drawing.
	if !f.ContainsPoint(nan) || !f.ContainsPoint(inf) {
		t.Error("ContainsPoint rejected a non-finite point")
	}
	if !f.IntersectsSphere(nan, 1) || !f.IntersectsSphere(Vec3{}, NaN) || !f.IntersectsSphere(Vec3{}, InfPos) {
		t.Error("IntersectsSphere rejected non-finite input")
	}
	if f.ContainsSphere(nan, 1) || f.ContainsSphere(Vec3{}, NaN) || f.ContainsSphere(Vec3{}, InfPos) {
		t.Error("ContainsSphere accepted non-finite input")
	}
	if !f.IntersectsBox(AABB{nan, Vec3{1, 1, 1}}) || !f.IntersectsBox(AABB{Vec3{}, inf}) {
		t.Error("IntersectsBox rejected a non-finite box")
	}
	if f.ContainsBox(AABB{nan, Vec3{1, 1, 1}}) || f.ContainsBox(AABB{Vec3{}, inf}) {
		t.Error("ContainsBox accepted a non-finite box")
	}

	// Negative radii are treated as zero, not as an error and not as an
	// inverted sphere.
	for _, r := range []float64{-1, -1e9} {
		if got, want := f.IntersectsSphere(Vec3{}, r), f.ContainsPoint(Vec3{}); got != want {
			t.Errorf("IntersectsSphere(origin, %v) = %v, want %v", r, got, want)
		}
		if got, want := f.ContainsSphere(Vec3{}, r), f.ContainsPoint(Vec3{}); got != want {
			t.Errorf("ContainsSphere(origin, %v) = %v, want %v", r, got, want)
		}
	}

	// Inverted boxes are empty: never intersecting and never contained.
	inverted := AABB{Min: Vec3{1, 0, 0}, Max: Vec3{-1, 0, 0}}
	if inverted.Valid() {
		t.Error("Valid reported an inverted box as valid")
	}
	if f.IntersectsBox(inverted) {
		t.Error("IntersectsBox accepted an inverted box")
	}
	if f.ContainsBox(inverted) {
		t.Error("ContainsBox accepted an inverted box")
	}
	if !(AABB{Min: Vec3{-1, -1, -1}, Max: Vec3{1, 1, 1}}).Valid() {
		t.Error("Valid rejected a well-formed box")
	}
	if (AABB{Min: Vec3{-1, -1, -1}, Max: Vec3{NaN, 1, 1}}).Valid() {
		t.Error("Valid accepted a box with a NaN corner")
	}

	// Zero-size boxes on the boundary are visible.
	onBoundary := AABB{Min: Vec3{1, 1, 1}, Max: Vec3{1, 1, 1}}
	if !f.IntersectsBox(onBoundary) || !f.ContainsBox(onBoundary) {
		t.Error("a degenerate box on the boundary should be visible")
	}
}

func TestViewFrustumExtremeCoordinates(t *testing.T) {
	t.Parallel()

	f := mustViewFrustum(t, Ident4())

	// Very large but finite coordinates must not produce NaN or panic; they are
	// simply far outside the unit cube.
	huge := float64(3e38)
	if f.ContainsPoint(Vec3{0, 0, -huge}) {
		t.Error("ContainsPoint accepted a huge point")
	}
	if f.IntersectsBox(AABB{Min: Vec3{0, 0, -huge}, Max: Vec3{0, 0, -huge}}) {
		t.Error("IntersectsBox accepted a huge box")
	}

	// A box that starts beyond the right plane and extends far away: the
	// support point against the right plane is its nearest corner, so the box
	// is rejected even though its far corner is enormous.
	if f.IntersectsBox(AABB{Min: Vec3{2, 0, 0}, Max: Vec3{huge, 1, 1}}) {
		t.Error("IntersectsBox accepted a box extending far outside")
	}

	// A box centred on the frustum but much larger than it, but whose extent
	// still leaves it entirely on the inside of every plane, is accepted.
	nearby := AABB{Min: Vec3{-2, -2, -2}, Max: Vec3{2, 2, 2}}
	if !f.IntersectsBox(nearby) {
		t.Error("IntersectsBox rejected a box covering the frustum")
	}
	if f.ContainsBox(nearby) {
		t.Error("ContainsBox accepted a box larger than the frustum")
	}

	// Extreme coordinates that overflow the dot product must not be accepted
	// as contained; the conservative answer for intersection is "yes", since
	// the box covers the whole unit cube.
	big := AABB{Min: Vec3{-huge, -huge, -huge}, Max: Vec3{huge, huge, huge}}
	if !f.IntersectsBox(big) {
		t.Error("IntersectsBox rejected a huge box covering the frustum")
	}
	if f.ContainsBox(big) {
		t.Error("ContainsBox accepted a huge box")
	}
}
