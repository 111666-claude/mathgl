// This file is generated from mgl32/frustum.go; DO NOT EDIT

// Copyright 2014 The go-gl Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package mgl64

import (
	"errors"
	"math"
)

// Errors returned by ViewFrustumFromMatrix.
var (
	// ErrNonFiniteMatrix is returned when the view-projection matrix contains
	// a NaN or an infinite element.
	ErrNonFiniteMatrix = errors.New("view-projection matrix contains non-finite values")

	// ErrNonInvertibleMatrix is returned when the view-projection matrix is
	// singular (its determinant is zero or non-finite) and therefore does not
	// describe a bounded view volume.
	ErrNonInvertibleMatrix = errors.New("view-projection matrix is not invertible")

	// ErrDegeneratePlane is returned when the view-projection matrix yields a
	// frustum plane whose normal is zero or non-finite.
	ErrDegeneratePlane = errors.New("view-projection matrix yields a degenerate frustum plane")
)

// PlaneIndex identifies one of the six planes of a ViewFrustum.
type PlaneIndex int

// The six planes of a frustum. The order matches the order returned by
// ViewFrustumFromMatrix and may be used to index ViewFrustum.Planes.
const (
	PlaneLeft PlaneIndex = iota
	PlaneRight
	PlaneBottom
	PlaneTop
	PlaneNear
	PlaneFar
)

// Plane is an oriented plane. Normal is a unit-length vector and Offset is the
// signed distance from the origin to the plane along Normal, so the plane is
// the set of points p for which:
//
//	Normal.Dot(p) + Offset == 0
//
// A point is on the "inside" (the side the normal points to) when
// SignedDistance is positive. Frustum planes therefore satisfy
// SignedDistance(p) >= 0 for every point p inside the frustum; points exactly
// on a plane are on the boundary and are considered inside.
type Plane struct {
	Normal Vec3
	Offset float64
}

// SignedDistance returns the signed distance from v to the plane. It is
// positive on the side the plane normal points to and negative on the other
// side. The normal of planes obtained from ViewFrustumFromMatrix is unit-length,
// so the result is a true distance.
//
// For extremely large (but finite) coordinates the dot product may overflow;
// in that case the returned value may be ±Inf or NaN.
func (p Plane) SignedDistance(v Vec3) float64 {
	return p.Normal[0]*v[0] + p.Normal[1]*v[1] + p.Normal[2]*v[2] + p.Offset
}

// distance returns the signed distance from v to the plane along with a bound
// on the rounding error of that value. The bound scales with the magnitude of
// the terms that were summed, so it grows when the dot product and the offset
// nearly cancel, which is exactly when the computed distance is least
// trustworthy. The ViewFrustum predicates use this bound to keep objects that
// lie exactly on a plane visible despite floating point error.
func (p Plane) distance(v Vec3) (d, tolerance float64) {
	dot := p.Normal[0]*v[0] + p.Normal[1]*v[1] + p.Normal[2]*v[2]
	return dot + p.Offset, frustumEpsilon * (Abs(dot) + Abs(p.Offset))
}

// ViewFrustum is the view volume described by six oriented planes. The zero
// value is not a valid frustum; construct one with ViewFrustumFromMatrix.
//
// Planes are ordered according to the PlaneIndex constants and their normals
// point to the inside of the frustum. The predicates (ContainsPoint,
// IntersectsSphere, ContainsSphere, IntersectsBox and ContainsBox) are all
// conservative near a plane: a value that differs from the exact distance only
// by rounding error is treated as inside, so an object exactly on the boundary
// stays visible.
type ViewFrustum struct {
	Planes [6]Plane
}

// ViewFrustumFromMatrix extracts the six oriented, normalized planes of the view
// volume defined by a combined view-projection matrix.
//
// Pass the product of the projection and the view (modelview) matrix in the
// order used by Project and UnProject:
//
//	vp := projection.Mul4(modelview)
//	frustum, err := ViewFrustumFromMatrix(vp)
//
// The matrix may contain any affine view (rotation, translation, scale, ...)
// combined with a perspective, frustum or orthographic projection; the
// resulting planes are expressed in world space and follow the camera.
//
// The matrix is assumed to use the OpenGL clip-space convention, where a point
// with clip coordinates (x, y, z, w) is visible when
//
//	-w <= x <= w,  -w <= y <= w,  -w <= z <= w
//
// i.e. normalized device coordinates range from -1 to 1 on all three axes.
// This matches the matrices produced by Perspective, Frustum and Ortho in this
// package and correctly handles their negative (OpenGL) depth range. Projections
// using a [0, 1] depth range (Direct3D style), a reversed depth range or an
// infinite far plane are not supported.
//
// The plane normals point into the view volume. Points exactly on a plane are
// on the boundary and are treated as inside by all of the ViewFrustum
// predicates.
//
// ViewFrustumFromMatrix returns ErrNonFiniteMatrix if any element of vp is NaN or
// infinite, ErrNonInvertibleMatrix if vp has a zero or non-finite determinant,
// and ErrDegeneratePlane if a plane's normal is zero or non-finite. A matrix
// with an infinite far plane is rejected by one of these checks because such a
// projection does not bound the view volume.
func ViewFrustumFromMatrix(vp Mat4) (ViewFrustum, error) {
	for _, e := range vp {
		if !isFinite(e) {
			return ViewFrustum{}, ErrNonFiniteMatrix
		}
	}

	if det := vp.Det(); det == 0 || !isFinite(det) {
		return ViewFrustum{}, ErrNonInvertibleMatrix
	}

	// Clip-space coordinates are row * p. With the OpenGL convention the six
	// planes are the rows of w + x, w - x, w + y, w - y, w + z and w - z.
	r0, r1, r2, r3 := vp.Rows()
	coefficients := [6]Vec4{
		r3.Add(r0), // left:   x + w >= 0
		r3.Sub(r0), // right:  w - x >= 0
		r3.Add(r1), // bottom: y + w >= 0
		r3.Sub(r1), // top:    w - y >= 0
		r3.Add(r2), // near:   z + w >= 0
		r3.Sub(r2), // far:    w - z >= 0
	}

	var f ViewFrustum
	for i, c := range coefficients {
		plane, err := planeFromCoefficients(c)
		if err != nil {
			return ViewFrustum{}, err
		}
		f.Planes[i] = plane
	}

	return f, nil
}

// planeFromCoefficients builds a normalized plane from the coefficients
// (a, b, c, d) of the equation a*x + b*y + c*z + d = 0.
func planeFromCoefficients(c Vec4) (Plane, error) {
	n := Vec3{c[0], c[1], c[2]}

	// Normalize in float64 even for the 32-bit package: the coefficients are
	// already sums of matrix rows, and computing the length at higher
	// precision keeps the plane normals accurate to machine epsilon.
	// math.Hypot also avoids the overflow and underflow that Vec3.Len could
	// hit for very large or very small coefficients.
	length := math.Hypot(float64(n[0]), math.Hypot(float64(n[1]), float64(n[2])))
	if length == 0 || math.IsInf(length, 0) || math.IsNaN(length) {
		return Plane{}, ErrDegeneratePlane
	}

	inv := float64(1 / length)
	p := Plane{Normal: n.Mul(inv), Offset: c[3] * inv}
	if !isFiniteVec3(p.Normal) || !isFinite(p.Offset) {
		return Plane{}, ErrDegeneratePlane
	}

	return p, nil
}

// frustumEpsilon is the relative tolerance used by the ViewFrustum predicates
// when deciding which side of a plane a point lies on. It is deliberately
// larger than the machine epsilon of float32 so that an object exactly on a
// plane is not rejected because of the rounding error introduced while
// extracting and normalizing the planes. All of the predicates are
// conservative: a value within this relative band of a plane counts as inside,
// so culling may keep a sliver of invisible geometry but never discards
// visible geometry.
const frustumEpsilon = 1e-6

// ContainsPoint reports whether p lies inside or on the boundary of the
// frustum.
//
// A point whose coordinates are NaN or infinite cannot be classified and is
// reported as contained, so that callers which use this as a culling test err
// on the side of drawing the object.
func (f ViewFrustum) ContainsPoint(p Vec3) bool {
	if !isFiniteVec3(p) {
		return true
	}

	for _, plane := range f.Planes {
		d, tolerance := plane.distance(p)
		if !isNaN(d) && d < -tolerance {
			return false
		}
	}

	return true
}

// IntersectsSphere reports whether the sphere centered at center with the given
// radius intersects the frustum. A sphere that only touches a frustum plane is
// considered to intersect, so objects exactly on the boundary are visible.
//
// A negative radius is treated as zero, which reduces the test to
// ContainsPoint. If center or radius is NaN or infinite the result is true so
// that callers err on the side of drawing the object.
func (f ViewFrustum) IntersectsSphere(center Vec3, radius float64) bool {
	if !isFiniteVec3(center) || !isFinite(radius) {
		return true
	}

	if radius < 0 {
		radius = 0
	}

	for _, plane := range f.Planes {
		d, tolerance := plane.distance(center)
		if !isNaN(d) && d < -radius-tolerance {
			return false
		}
	}

	return true
}

// ContainsSphere reports whether the sphere centered at center with the given
// radius is entirely inside the frustum. A sphere that touches a frustum plane
// is considered contained.
//
// A negative radius is treated as zero, which reduces the test to
// ContainsPoint. If center or radius is NaN or infinite the result is false,
// because containment cannot be established.
func (f ViewFrustum) ContainsSphere(center Vec3, radius float64) bool {
	if !isFiniteVec3(center) || !isFinite(radius) {
		return false
	}

	if radius < 0 {
		radius = 0
	}

	for _, plane := range f.Planes {
		d, tolerance := plane.distance(center)
		if isNaN(d) || d < radius-tolerance {
			return false
		}
	}

	return true
}

// AABB is an axis-aligned bounding box defined by its minimum and maximum
// corners, both of which are inclusive.
//
// A box is valid when all components are finite and Min is component-wise less
// than or equal to Max; use Valid to check. An inverted box (a component of
// Min greater than the corresponding component of Max) denotes an empty
// volume and is never reported as intersecting or contained.
type AABB struct {
	Min Vec3
	Max Vec3
}

// Valid reports whether the box is well formed: all components are finite and
// Min is component-wise less than or equal to Max.
func (b AABB) Valid() bool {
	if !isFiniteVec3(b.Min) || !isFiniteVec3(b.Max) {
		return false
	}

	return b.Min[0] <= b.Max[0] && b.Min[1] <= b.Max[1] && b.Min[2] <= b.Max[2]
}

// IntersectsBox reports whether the axis-aligned box intersects the frustum.
// A box that only touches a frustum plane is considered to intersect.
//
// The test is performed against the six frustum planes. It is conservative:
// it may report a box that lies just outside an edge or corner of the frustum
// as intersecting, but it never rejects a box that is actually visible.
//
// If a component of the box is NaN or infinite the result is true, so callers
// err on the side of drawing the object. An inverted box is empty and gives
// false. For extremely large (but finite) coordinates an overflow may make the
// result true; an underflow or an ordinary large coordinate outside the volume
// gives false.
func (f ViewFrustum) IntersectsBox(box AABB) bool {
	if !isFiniteVec3(box.Min) || !isFiniteVec3(box.Max) {
		return true
	}

	if box.Min[0] > box.Max[0] || box.Min[1] > box.Max[1] || box.Min[2] > box.Max[2] {
		return false
	}

	for _, plane := range f.Planes {
		// The point of the box farthest along the plane normal is the one that
		// determines whether the box is completely outside this plane.
		d, tolerance := plane.distance(boxSupport(box, plane.Normal))
		if !isNaN(d) && d < -tolerance {
			return false
		}
	}

	return true
}

// ContainsBox reports whether the axis-aligned box is entirely inside the
// frustum. A box that touches a frustum plane is considered contained.
//
// If a component of the box is NaN or infinite the result is false, because
// containment cannot be established. An inverted box is empty and is never
// reported as contained. For extremely large (but finite) coordinates an
// overflow makes the result false.
func (f ViewFrustum) ContainsBox(box AABB) bool {
	if !isFiniteVec3(box.Min) || !isFiniteVec3(box.Max) {
		return false
	}

	if box.Min[0] > box.Max[0] || box.Min[1] > box.Max[1] || box.Min[2] > box.Max[2] {
		return false
	}

	for _, plane := range f.Planes {
		// The point of the box nearest along the plane normal determines
		// whether any part of the box is outside this plane.
		d, tolerance := plane.distance(boxSupport(box, plane.Normal.Mul(-1)))
		if isNaN(d) || d < -tolerance {
			return false
		}
	}

	return true
}

// boxSupport returns the corner of box that is farthest in the direction of
// dir, i.e. the corner maximizing dir.Dot(corner).
func boxSupport(box AABB, dir Vec3) Vec3 {
	var p Vec3
	for i := 0; i < 3; i++ {
		if dir[i] >= 0 {
			p[i] = box.Max[i]
		} else {
			p[i] = box.Min[i]
		}
	}
	return p
}

// isFinite reports whether v is neither NaN nor infinite.
func isFinite(v float64) bool {
	return !math.IsNaN(float64(v)) && !math.IsInf(float64(v), 0)
}

// isNaN reports whether v is NaN.
func isNaN(v float64) bool {
	return math.IsNaN(float64(v))
}

// isFiniteVec3 reports whether all components of v are finite.
func isFiniteVec3(v Vec3) bool {
	return isFinite(v[0]) && isFinite(v[1]) && isFinite(v[2])
}
