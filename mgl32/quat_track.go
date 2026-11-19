// Copyright 2014 The go-gl/mathgl Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package mgl32

import (
	"errors"
	"math"
	"sort"
)

// QuatTrackEndRule selects how an open (non-looping) QuatTrack determines the
// tangent at its first and last key.
type QuatTrackEndRule int

const (
	// QuatTrackEndClamped makes the trajectory come to rest at the first and
	// last key: the one-sided angular velocity is zero at both ends. This is
	// the zero value, and therefore the default.
	QuatTrackEndClamped QuatTrackEndRule = iota

	// QuatTrackEndOneSided gives the first and last key the angular velocity of
	// the squad segment adjacent to it, as if that segment were continued. The
	// trajectory does not stop at the ends.
	QuatTrackEndOneSided
)

// Errors returned by NewQuatTrack. They are exported so callers can tell the
// various invalid inputs apart.
var (
	// ErrQuatTrackLength is returned when the times and keys slices differ in
	// length.
	ErrQuatTrackLength = errors.New("quat track: times and keys must have the same length")

	// ErrQuatTrackEmpty is returned when no keys were given.
	ErrQuatTrackEmpty = errors.New("quat track: at least one key is required")

	// ErrQuatTrackTime is returned when a time is not finite, or when two
	// consecutive times are too far apart to compute a finite duration.
	ErrQuatTrackTime = errors.New("quat track: times must be finite")

	// ErrQuatTrackTimeOrder is returned when the times are not strictly
	// increasing. Duplicate and reversed times are rejected.
	ErrQuatTrackTimeOrder = errors.New("quat track: times must be strictly increasing")

	// ErrQuatTrackNonFinite is returned when a quaternion key has a NaN or
	// infinite component.
	ErrQuatTrackNonFinite = errors.New("quat track: quaternion components must be finite")

	// ErrQuatTrackZeroQuat is returned when a quaternion key has zero length
	// and can not be normalized.
	ErrQuatTrackZeroQuat = errors.New("quat track: quaternion must have non-zero length")

	// ErrQuatTrackLoopSeam is returned when a looping track's first and last
	// key do not represent the same orientation.
	ErrQuatTrackLoopSeam = errors.New("quat track: loop first and last keys must represent the same orientation")

	// ErrQuatTrackEndRule is returned for an unrecognized QuatTrackEndRule.
	ErrQuatTrackEndRule = errors.New("quat track: unknown end rule")
)

// quatTrackLoopEpsilon is the largest deviation from 1 allowed for the
// orientation dot product of a looping track's first and last key. Two unit
// quaternions with a dot of 1-epsilon are separated by roughly sqrt(2*epsilon)
// radians.
const quatTrackLoopEpsilon = 1e-6

// QuatTrackConfig configures a QuatTrack built by NewQuatTrack. The zero value
// is an open track whose trajectory comes to rest at the first and last key.
type QuatTrackConfig struct {
	// Loop closes the keyframes into a cycle. In this mode keys[0] and
	// keys[len(keys)-1] must represent the same orientation (they may be
	// antipodal, q and -q describe the same rotation); the last key is snapped
	// exactly onto the first. The track's period is the time span between the
	// first and last key, and queries outside that span wrap around.
	//
	// EndRule is ignored when Loop is true.
	Loop bool

	// EndRule selects the tangent rule at the ends of an open track. It has no
	// effect on a looping track.
	EndRule QuatTrackEndRule
}

// QuatTrack is a smooth, time-parameterized quaternion trajectory through an
// ordered set of keyframes. It uses squad (spherical quadrangle) interpolation:
// each segment is a spherical blend of two slerps, and the control points are
// chosen so that the angular velocity is continuous across every interior key.
// A QuatTrack therefore passes exactly through every key while leaving the
// interpolation C1 continuous.
//
// All keys are normalized and their signs are aligned so that consecutive keys
// lie on the same hemisphere; a key and its negation describe the same
// orientation and produce the same trajectory. Every value returned by At is a
// unit quaternion.
//
// A track with a single key is valid and returns that key for every query.
//
// Use NewQuatTrack to build a track.
type QuatTrack struct {
	times []float32
	keys  []Quat
	ctrl  []Quat
	loop  bool
}

// NewQuatTrack validates times and keys and builds a squad trajectory through
// the keyframes. times must be finite and strictly increasing, and every key
// must have finite components and a non-zero length. The slices are copied;
// later changes to them do not affect the track.
//
// For a looping track (cfg.Loop) the first and last key must describe the same
// orientation to within a small tolerance; otherwise ErrQuatTrackLoopSeam is
// returned. The last key is snapped exactly onto the first, and queries wrap
// around with a period of times[len(times)-1]-times[0].
//
// The returned track is safe for concurrent reads.
func NewQuatTrack(times []float32, keys []Quat, cfg QuatTrackConfig) (*QuatTrack, error) {
	if !cfg.Loop {
		switch cfg.EndRule {
		case QuatTrackEndClamped, QuatTrackEndOneSided:
		default:
			return nil, ErrQuatTrackEndRule
		}
	}

	if len(times) != len(keys) {
		return nil, ErrQuatTrackLength
	}
	if len(keys) == 0 {
		return nil, ErrQuatTrackEmpty
	}

	track := &QuatTrack{
		times: make([]float32, len(times)),
		keys:  make([]Quat, len(keys)),
		loop:  cfg.Loop,
	}
	copy(track.times, times)

	for i, tm := range track.times {
		if !quatTrackFinite(tm) {
			return nil, ErrQuatTrackTime
		}
		if i > 0 {
			d := tm - track.times[i-1]
			if !(d > 0) {
				return nil, ErrQuatTrackTimeOrder
			}
			if !quatTrackFinite(d) {
				return nil, ErrQuatTrackTime
			}
		}
	}

	for i, q := range keys {
		if !quatTrackFiniteQuat(q) {
			return nil, ErrQuatTrackNonFinite
		}
		norm, ok := quatTrackNormalize(q)
		if !ok {
			return nil, ErrQuatTrackZeroQuat
		}
		track.keys[i] = norm
	}

	// Align antipodal neighbours so every segment takes the short way around
	// and the relative quaternions have a non-negative scalar part.
	for i := 1; i < len(track.keys); i++ {
		if track.keys[i-1].Dot(track.keys[i]) < 0 {
			track.keys[i] = track.keys[i].Scale(-1)
		}
	}

	n := len(track.keys)
	if track.loop && n > 1 {
		if !quatTrackFinite(track.times[n-1] - track.times[0]) {
			return nil, ErrQuatTrackTime
		}
		first, last := track.keys[0], track.keys[n-1]
		if math.Abs(float64(first.Dot(last))) < 1-quatTrackLoopEpsilon {
			return nil, ErrQuatTrackLoopSeam
		}
		// Snap the closing key so the cycle is exact and the control points can
		// be built cyclically.
		track.keys[n-1] = first
	}

	track.ctrl = make([]Quat, n)
	if n == 1 {
		track.ctrl[0] = track.keys[0]
		return track, nil
	}

	m := n - 1 // number of segments
	dur := make([]float32, m)
	rel := make([]Vec3, m)
	for i := 0; i < m; i++ {
		dur[i] = track.times[i+1] - track.times[i]
		rel[i] = quatTrackLog(track.keys[i].Conjugate().Mul(track.keys[i+1]))
	}

	if track.loop {
		for i := 0; i < m; i++ {
			prev := (i - 1 + m) % m
			ctrl := quatTrackControl(rel[prev], dur[prev], rel[i], dur[i])
			track.ctrl[i] = track.keys[i].Mul(quatTrackExp(ctrl)).Normalize()
		}
		track.ctrl[m] = track.ctrl[0]
		return track, nil
	}

	switch cfg.EndRule {
	case QuatTrackEndClamped:
		// Pull the control points a half tangent towards the interior so the
		// one-sided angular velocity vanishes at both ends.
		track.ctrl[0] = track.keys[0].Mul(quatTrackExp(rel[0].Mul(-0.5))).Normalize()
		track.ctrl[n-1] = track.keys[n-1].Mul(quatTrackExp(rel[m-1].Mul(0.5))).Normalize()
	case QuatTrackEndOneSided:
		// A control point equal to the key leaves the adjacent segment's
		// angular velocity untouched.
		track.ctrl[0] = track.keys[0]
		track.ctrl[n-1] = track.keys[n-1]
	}

	for i := 1; i < n-1; i++ {
		ctrl := quatTrackControl(rel[i-1], dur[i-1], rel[i], dur[i])
		track.ctrl[i] = track.keys[i].Mul(quatTrackExp(ctrl)).Normalize()
	}

	return track, nil
}

// Len returns the number of keyframes in the track.
func (track *QuatTrack) Len() int {
	return len(track.keys)
}

// Loop reports whether the track is closed into a cycle.
func (track *QuatTrack) Loop() bool {
	return track.loop
}

// Start returns the time of the first keyframe. Queries before this time clamp
// to it, or wrap around for a looping track.
func (track *QuatTrack) Start() float32 {
	return track.times[0]
}

// End returns the time of the last keyframe. For a looping track this is the
// end of the period; the track continues at Start afterwards.
func (track *QuatTrack) End() float32 {
	return track.times[len(track.times)-1]
}

// At returns the unit quaternion at time t.
//
// For an open track, times outside the keyframe span clamp to the first or last
// key. For a looping track the time wraps around with the track's period, so
// At(Start()+k*period) always returns the first key. A NaN time returns the
// first key.
func (track *QuatTrack) At(t float32) Quat {
	n := len(track.keys)
	if n == 1 {
		return track.keys[0]
	}

	if math.IsNaN(float64(t)) {
		t = track.times[0]
	}

	start, end := track.times[0], track.times[n-1]
	if track.loop {
		period := float64(end) - float64(start)
		if math.IsInf(float64(t), 0) {
			t = start
		}
		// Reduce in float64: t-start can exceed the float32 range for extreme
		// but finite times.
		rel := math.Mod(float64(t)-float64(start), period)
		if rel < 0 {
			rel += period
		}
		t = float32(float64(start) + rel)
	} else {
		if t < start {
			t = start
		} else if t > end {
			t = end
		}
	}

	// Find the segment i with times[i] <= t < times[i+1]. Clamp the last
	// segment so t == end lands on the final key instead of running past it.
	m := n - 1
	i := sort.Search(m, func(j int) bool { return track.times[j+1] > t })
	if i >= m {
		i = m - 1
	}

	u := (t - track.times[i]) / (track.times[i+1] - track.times[i])
	if u < 0 {
		u = 0
	} else if u > 1 {
		u = 1
	}

	a := QuatSlerp(track.keys[i], track.keys[i+1], u)
	b := QuatSlerp(track.ctrl[i], track.ctrl[i+1], u)
	return QuatSlerp(a, b, 2*u*(1-u)).Normalize()
}

// quatTrackControl returns the rotation vector (in the body frame of key i)
// that places key i's squad control point: key_i * exp(result).
//
// relPrev is the log of key[i-1]^-1*key[i] over durPrev, and relNext is the log
// of key[i]^-1*key[i+1] over durNext. With uniform spacing this reduces to
// (relPrev-relNext)/4, the classic squad control point.
func quatTrackControl(relPrev Vec3, durPrev float32, relNext Vec3, durNext float32) Vec3 {
	// Scale the durations by the larger one before multiplying: both ratios
	// are then in (0,1], their sum is in (1,2], and durations close to the
	// float maximum can not overflow.
	longest := durPrev
	if durNext > longest {
		longest = durNext
	}
	prev := durPrev / longest
	next := durNext / longest
	scale := 1 / (2 * (prev + next))
	return relPrev.Mul(next * scale).Sub(relNext.Mul(prev * scale))
}

// quatTrackNormalize returns q scaled to unit length. It returns false if q is
// the zero quaternion. Scaling by the largest component first keeps the
// computation finite for keys whose components are near the float maximum.
func quatTrackNormalize(q Quat) (Quat, bool) {
	largest := Abs(q.W)
	for _, c := range []float32{q.V[0], q.V[1], q.V[2]} {
		if a := Abs(c); a > largest {
			largest = a
		}
	}
	if largest == 0 {
		return Quat{}, false
	}
	q = Quat{q.W / largest, Vec3{q.V[0] / largest, q.V[1] / largest, q.V[2] / largest}}
	return q.Normalize(), true
}

// quatTrackLog returns the principal logarithm of a unit quaternion: the
// rotation vector v such that Quat{0,v} has the same direction as q's vector
// part and angle in [0, pi/2].
func quatTrackLog(q Quat) Vec3 {
	if q.W < 0 {
		q = q.Scale(-1)
	}
	v := q.V
	s := v.Len()
	if s == 0 {
		return Vec3{}
	}
	return v.Mul(float32(math.Atan2(float64(s), float64(q.W))) / s)
}

// quatTrackExp returns the unit quaternion exp(Quat{0, v}).
func quatTrackExp(v Vec3) Quat {
	theta := v.Len()
	if theta == 0 {
		return QuatIdent()
	}
	sn, cs := math.Sincos(float64(theta))
	return Quat{float32(cs), v.Mul(float32(sn) / theta)}
}

// quatTrackFinite reports whether f is neither NaN nor infinite.
func quatTrackFinite(f float32) bool {
	return !math.IsNaN(float64(f)) && !math.IsInf(float64(f), 0)
}

// quatTrackFiniteQuat reports whether every component of q is finite.
func quatTrackFiniteQuat(q Quat) bool {
	return quatTrackFinite(q.W) && quatTrackFinite(q.V[0]) &&
		quatTrackFinite(q.V[1]) && quatTrackFinite(q.V[2])
}
