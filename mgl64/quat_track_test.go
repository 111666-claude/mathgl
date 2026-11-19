// This file is generated from mgl32/quat_track_test.go; DO NOT EDIT

// Copyright 2014 The go-gl/mathgl Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package mgl64

import (
	"math"
	"math/rand"
	"testing"
)

// trackCameras returns four camera orientations looking in different
// directions, as a rig would sweep through them.
func trackCameras() []Quat {
	return []Quat{
		QuatLookAtV(Vec3{0, 0, 0}, Vec3{0, 0, -1}, Vec3{0, 1, 0}), // forward
		QuatLookAtV(Vec3{0, 0, 0}, Vec3{1, 0, 0}, Vec3{0, 1, 0}),  // right
		QuatLookAtV(Vec3{0, 0, 0}, Vec3{0, 0, 1}, Vec3{0, 1, 0}),  // behind
		QuatLookAtV(Vec3{0, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, -1}), // up
	}
}

// trackBodyOmega returns the body-frame angular velocity of track between two
// times, in radians per unit time, using the finite difference t1-t0.
func trackBodyOmega(q0, q1 Quat, dt float64) Vec3 {
	return quatTrackLog(q0.Conjugate().Mul(q1)).Mul(2 / dt)
}

// trackOmegaDiff returns the length of the difference between two angular
// velocities. An absolute comparison is used because angular velocity
// components near zero would fail a relative comparison.
func trackOmegaDiff(a, b Vec3) float64 {
	return a.Sub(b).Len()
}

// trackSideVelocities returns the one-sided rotation-vector rates at key i:
// the rate arriving along the segment before the key (u -> 1) and the rate
// leaving along the segment after it (u -> 0), both expressed in the key's own
// frame. Continuous angular velocity means the two vectors are equal.
//
// It works from the stored segment data rather than sampling At, because the
// derivative of squad has a jump in acceleration at a key: a finite difference
// straddling the key carries an O(h) truncation error. With the control point
// c_i = key_i*exp(M), squad's derivative ends the previous segment at
// (relPrev - 2M)/durPrev and starts the next segment at (relNext + 2M)/durNext,
// where relPrev = log(key[i-1]^-1 key[i]) and relNext = log(key[i]^-1 key[i+1]).
func trackSideVelocities(trk *QuatTrack, i int) (before, after Vec3) {
	key := trk.keys[i]
	rel := func(a, b Quat) Vec3 { return quatTrackLog(a.Conjugate().Mul(b)) }
	m := quatTrackLog(key.Conjugate().Mul(trk.ctrl[i]))

	if i > 0 {
		before = rel(trk.keys[i-1], key).Sub(m.Mul(2)).Mul(1 / (trk.times[i] - trk.times[i-1]))
	}
	if i < len(trk.keys)-1 {
		after = rel(key, trk.keys[i+1]).Add(m.Mul(2)).Mul(1 / (trk.times[i+1] - trk.times[i]))
	}
	return before, after
}

// trackLoopSeamVelocities returns the one-sided rotation-vector rates at the
// seam of a looping track, expressed in the frame of the first key: the rate
// arriving along the closing segment and the rate leaving along the first
// segment. Continuous angular velocity means the two are equal.
func trackLoopSeamVelocities(trk *QuatTrack) (before, after Vec3) {
	keys := trk.keys
	m := len(keys) - 1
	rel := func(a, b Quat) Vec3 { return quatTrackLog(a.Conjugate().Mul(b)) }
	// The closing segment duplicates the first key as keys[m]; its control
	// point is ctrl[m] = ctrl[0].
	m0 := quatTrackLog(keys[0].Conjugate().Mul(trk.ctrl[0]))
	before = rel(keys[m-1], keys[0]).Sub(m0.Mul(2)).Mul(1 / (trk.times[m] - trk.times[m-1]))
	after = rel(keys[0], keys[1]).Add(m0.Mul(2)).Mul(1 / (trk.times[1] - trk.times[0]))
	return before, after
}

func TestNewQuatTrackErrors(t *testing.T) {
	keys := trackCameras()
	good := []float64{0, 1, 2, 3}
	nan := float64(math.NaN())
	inf := float64(math.Inf(1))

	tests := []struct {
		name  string
		times []float64
		keys  []Quat
		cfg   QuatTrackConfig
		want  error
	}{
		{"length mismatch", []float64{0, 1}, keys, QuatTrackConfig{}, ErrQuatTrackLength},
		{"empty", nil, nil, QuatTrackConfig{}, ErrQuatTrackEmpty},
		{"duplicate time", []float64{0, 1, 1, 3}, keys, QuatTrackConfig{}, ErrQuatTrackTimeOrder},
		{"reversed time", []float64{0, 2, 1, 3}, keys, QuatTrackConfig{}, ErrQuatTrackTimeOrder},
		{"nan time", []float64{0, 1, nan, 3}, keys, QuatTrackConfig{}, ErrQuatTrackTime},
		{"inf time", []float64{0, 1, inf, 3}, keys, QuatTrackConfig{}, ErrQuatTrackTime},
		{
			"nan component",
			good,
			[]Quat{keys[0], keys[1], {nan, Vec3{}}, keys[3]},
			QuatTrackConfig{},
			ErrQuatTrackNonFinite,
		},
		{
			"inf component",
			good,
			[]Quat{keys[0], keys[1], {1, Vec3{inf, 0, 0}}, keys[3]},
			QuatTrackConfig{},
			ErrQuatTrackNonFinite,
		},
		{
			"zero quaternion",
			good,
			[]Quat{keys[0], keys[1], {}, keys[3]},
			QuatTrackConfig{},
			ErrQuatTrackZeroQuat,
		},
		{
			"loop seam discontinuity",
			[]float64{0, 1, 2, 3},
			keys, // forward -> right -> behind -> up does not close
			QuatTrackConfig{Loop: true},
			ErrQuatTrackLoopSeam,
		},
		{
			"unknown end rule",
			good,
			keys,
			QuatTrackConfig{EndRule: QuatTrackEndRule(42)},
			ErrQuatTrackEndRule,
		},
	}

	for _, c := range tests {
		trk, err := NewQuatTrack(c.times, c.keys, c.cfg)
		if err != c.want {
			t.Errorf("%s: NewQuatTrack error = %v, want %v", c.name, err, c.want)
		}
		if trk != nil {
			t.Errorf("%s: NewQuatTrack returned a track alongside error %v", c.name, err)
		}
	}
}

func TestQuatTrackClampsOpenEnds(t *testing.T) {
	cams := trackCameras()
	times := []float64{1, 0.5 + 1, 1.7 + 1, 4 + 1}
	trk, err := NewQuatTrack(times, cams, QuatTrackConfig{})
	if err != nil {
		t.Fatalf("NewQuatTrack returned error: %v", err)
	}
	first, last := trk.At(times[0]), trk.At(times[len(times)-1])
	for _, at := range []float64{-1e9, times[0] - 1e-3, times[0]} {
		if got := trk.At(at); !got.ApproxEqualThreshold(first, 1e-6) {
			t.Errorf("At(%v) = %v, want the first key %v", at, got, first)
		}
	}
	for _, at := range []float64{times[len(times)-1], times[len(times)-1] + 1e-3, 1e9} {
		if got := trk.At(at); !got.ApproxEqualThreshold(last, 1e-6) {
			t.Errorf("At(%v) = %v, want the last key %v", at, got, last)
		}
	}
}

func TestQuatTrackNonFiniteQuery(t *testing.T) {
	cams := trackCameras()
	times := []float64{0, 0.5, 1.7, 4}
	trk, err := NewQuatTrack(times, cams, QuatTrackConfig{})
	if err != nil {
		t.Fatalf("NewQuatTrack returned error: %v", err)
	}
	if got := trk.At(float64(math.NaN())); !got.ApproxEqualThreshold(trk.At(times[0]), 1e-6) {
		t.Errorf("At(NaN) = %v, want the first key %v", got, trk.At(times[0]))
	}

	// For a looping track, an infinite query wraps to the start as well.
	looped := []Quat{cams[0], cams[1], cams[0]}
	loopTimes := []float64{0, 1, 2}
	loopTrk, err := NewQuatTrack(loopTimes, looped, QuatTrackConfig{Loop: true})
	if err != nil {
		t.Fatalf("NewQuatTrack returned error: %v", err)
	}
	if got := loopTrk.At(float64(math.Inf(1))); !got.OrientationEqualThreshold(loopTrk.At(loopTimes[0]), 1e-6) {
		t.Errorf("At(+Inf) = %v, want the first key %v", got, loopTrk.At(loopTimes[0]))
	}
}

func TestQuatTrackExtremeFiniteKeys(t *testing.T) {
	// Components near the float maximum must not overflow during
	// normalization.
	big := float64(math.MaxFloat64 / 2)
	keys := []Quat{
		{big, Vec3{big, 0, 0}},
		{0, Vec3{big, big, 0}},
		{big, Vec3{0, 0, -big}},
	}
	trk, err := NewQuatTrack([]float64{0, 1, 2}, keys, QuatTrackConfig{})
	if err != nil {
		t.Fatalf("NewQuatTrack returned error: %v", err)
	}
	for i := 0; i <= 30; i++ {
		at := float64(i) / 15
		if got := trk.At(at); !FloatEqualThreshold(got.Len(), 1, 1e-5) {
			t.Fatalf("At(%v) has length %v, want 1", at, got.Len())
		}
	}

	// Times spanning most of the float range must not overflow their
	// durations when computing the control points.
	hugeTimes := []float64{0, math.MaxFloat64 / 2, math.MaxFloat64}
	if _, err := NewQuatTrack(hugeTimes, keys, QuatTrackConfig{}); err != nil {
		t.Fatalf("NewQuatTrack with huge times returned error: %v", err)
	}
	for i := 0; i <= 30; i++ {
		at := hugeTimes[0] + (hugeTimes[2]-hugeTimes[0])*float64(i)/30
		if got := trk.At(at); !FloatEqualThreshold(got.Len(), 1, 1e-5) {
			t.Fatalf("At(%v) has length %v, want 1", at, got.Len())
		}
	}
}

func TestQuatTrackRepeatedKeys(t *testing.T) {
	// Consecutive keys with the same orientation span a zero rotation; the
	// track must still be finite and unit length, and must pass through those
	// keys at their times.
	q := QuatRotate(0.4, Vec3{1, 2, 3})
	other := QuatRotate(-1.2, Vec3{0, 1, 0})
	keys := []Quat{q, q, other, other}
	times := []float64{0, 1, 2, 3}

	trk, err := NewQuatTrack(times, keys, QuatTrackConfig{})
	if err != nil {
		t.Fatalf("NewQuatTrack returned error: %v", err)
	}
	for i := 0; i <= 60; i++ {
		at := float64(i) / 20
		got := trk.At(at)
		if math.IsNaN(float64(got.W)) || !FloatEqualThreshold(got.Len(), 1, 1e-5) {
			t.Fatalf("At(%v) = %v, want a unit quaternion", at, got)
		}
	}
	for i, at := range times {
		if got := trk.At(at); !got.OrientationEqualThreshold(keys[i], 1e-5) {
			t.Errorf("At(%v) = %v, want orientation of key %d %v", at, got, i, keys[i])
		}
	}
}

func TestQuatTrackTwoKeys(t *testing.T) {
	a := QuatRotate(0.3, Vec3{0, 1, 0})
	b := QuatRotate(2.1, Vec3{1, 0, 0})

	// With the one-sided end rule both control points equal their keys, so the
	// squad blend collapses to plain slerp for a two-key track.
	oneSided, err := NewQuatTrack([]float64{0, 1}, []Quat{a, b}, QuatTrackConfig{EndRule: QuatTrackEndOneSided})
	if err != nil {
		t.Fatalf("NewQuatTrack returned error: %v", err)
	}
	for i := 0; i <= 20; i++ {
		u := float64(i) / 20
		want := QuatSlerp(a, b, u)
		if got := oneSided.At(u); !got.OrientationEqualThreshold(want, 1e-5) {
			t.Errorf("two-key one-sided At(%v) = %v, want slerp %v", u, got, want)
		}
	}

	// The default clamped rule instead eases in and out, so the path bows off
	// the slerp arc between the keys but still hits both ends and has unit
	// length throughout.
	clamped, err := NewQuatTrack([]float64{0, 1}, []Quat{a, b}, QuatTrackConfig{})
	if err != nil {
		t.Fatalf("NewQuatTrack returned error: %v", err)
	}
	if !clamped.At(0).OrientationEqualThreshold(a, 1e-5) || !clamped.At(1).OrientationEqualThreshold(b, 1e-5) {
		t.Errorf("two-key track does not reach its endpoints: %v, %v", clamped.At(0), clamped.At(1))
	}
	for i := 0; i <= 20; i++ {
		u := float64(i) / 20
		if got := clamped.At(u); !FloatEqualThreshold(got.Len(), 1, 1e-5) {
			t.Errorf("two-key clamped At(%v) has length %v, want 1", u, got.Len())
		}
	}
	// The rest at the start distinguishes it from slerp.
	if clamped.At(0.05).OrientationEqualThreshold(QuatSlerp(a, b, 0.05), 1e-5) {
		t.Errorf("clamped two-key track did not ease in at the start")
	}
}

func TestQuatTrackRandomizedProperties(t *testing.T) {
	rng := rand.New(rand.NewSource(1))
	randomQuat := func() Quat {
		axis := Vec3{
			float64(rng.NormFloat64()),
			float64(rng.NormFloat64()),
			float64(rng.NormFloat64()),
		}
		if axis.Len() == 0 {
			axis = Vec3{0, 1, 0}
		}
		return QuatRotate(float64(rng.Float64()*2*math.Pi), axis).Normalize()
	}
	randomTimes := func(n int) []float64 {
		times := make([]float64, n)
		for i := 1; i < n; i++ {
			// Deliberately non-uniform, and sometimes very short.
			times[i] = times[i-1] + 0.01 + float64(rng.Float64()*5)
		}
		return times
	}

	for trial := 0; trial < 25; trial++ {
		n := 2 + rng.Intn(6)
		times := randomTimes(n)
		keys := make([]Quat, n)
		for i := range keys {
			keys[i] = randomQuat()
		}

		for _, loop := range []bool{false, true} {
			ks := append([]Quat(nil), keys...)
			tms := append([]float64(nil), times...)
			if loop {
				// Close the cycle, sometimes with the antipodal representation.
				ks = append(ks, ks[0])
				if rng.Intn(2) == 0 {
					ks[len(ks)-1] = ks[0].Scale(-1)
				}
				tms = append(tms, tms[len(tms)-1]+0.5)
			}

			trk, err := NewQuatTrack(tms, ks, QuatTrackConfig{Loop: loop})
			if err != nil {
				t.Fatalf("trial %d loop=%v: NewQuatTrack returned error: %v", trial, loop, err)
			}
			if trk.Len() != len(ks) {
				t.Fatalf("trial %d loop=%v: Len() = %d, want %d", trial, loop, trk.Len(), len(ks))
			}

			// Every key is reached and every sample is a unit quaternion.
			for i, at := range tms {
				got := trk.At(at)
				if !FloatEqualThreshold(got.Len(), 1, 1e-4) {
					t.Fatalf("trial %d loop=%v: At(%v) length %v", trial, loop, at, got.Len())
				}
				if !got.OrientationEqualThreshold(ks[i], 1e-4) {
					t.Fatalf("trial %d loop=%v: At(%v) misses key %d", trial, loop, at, i)
				}
			}

			// Exact one-sided velocities agree at every interior key.
			for i := 1; i < trk.Len()-1; i++ {
				before, after := trackSideVelocities(trk, i)
				if d := trackOmegaDiff(before, after); d > 1e-3 {
					t.Fatalf("trial %d loop=%v: key %d angular velocity jump %v", trial, loop, i, d)
				}
			}

			if loop {
				before, after := trackLoopSeamVelocities(trk)
				if d := trackOmegaDiff(before, after); d > 1e-3 {
					t.Fatalf("trial %d: loop seam angular velocity jump %v", trial, d)
				}
			}
		}
	}
}

func TestQuatTrackSingleKey(t *testing.T) {
	q := QuatRotate(0.7, Vec3{0, 1, 0})
	trk, err := NewQuatTrack([]float64{5}, []Quat{q}, QuatTrackConfig{})
	if err != nil {
		t.Fatalf("NewQuatTrack returned error: %v", err)
	}
	if trk.Len() != 1 || trk.Start() != 5 || trk.End() != 5 || trk.Loop() {
		t.Fatalf("unexpected single-key track metadata: len=%d start=%v end=%v loop=%v",
			trk.Len(), trk.Start(), trk.End(), trk.Loop())
	}
	for _, at := range []float64{-100, 5, 5.5, 1e9} {
		if got := trk.At(at); !got.ApproxEqualThreshold(q, 1e-6) {
			t.Errorf("At(%v) = %v, want %v", at, got, q)
		}
	}
}

func TestQuatTrackThroughKeys(t *testing.T) {
	cams := trackCameras()
	times := []float64{0, 0.5, 1.7, 4}
	trk, err := NewQuatTrack(times, cams, QuatTrackConfig{})
	if err != nil {
		t.Fatalf("NewQuatTrack returned error: %v", err)
	}

	for i, at := range times {
		got := trk.At(at)
		if !got.OrientationEqualThreshold(cams[i], 1e-5) {
			t.Errorf("At(%v) = %v, want orientation of key %d (%v)", at, got, i, cams[i])
		}
		if !FloatEqualThreshold(got.Len(), 1, 1e-5) {
			t.Errorf("At(%v) is not a unit quaternion: len = %v", at, got.Len())
		}
	}

	// Sampling densely keeps every value a unit quaternion.
	for i := 0; i <= 200; i++ {
		at := times[0] + (times[3]-times[0])*float64(i)/200
		if got := trk.At(at); !FloatEqualThreshold(got.Len(), 1, 1e-5) {
			t.Fatalf("At(%v) is not a unit quaternion: len = %v", at, got.Len())
		}
	}
}

func TestQuatTrackAngularVelocityContinuous(t *testing.T) {
	cams := trackCameras()
	times := []float64{0, 0.5, 1.7, 4}
	trk, err := NewQuatTrack(times, cams, QuatTrackConfig{})
	if err != nil {
		t.Fatalf("NewQuatTrack returned error: %v", err)
	}

	// The exact one-sided velocities at each interior key agree; this holds
	// for non-uniform time spacing too.
	for i := 1; i < len(times)-1; i++ {
		before, after := trackSideVelocities(trk, i)
		if d := trackOmegaDiff(before, after); d > 1e-4 {
			t.Errorf("key %d (t=%v): angular velocity jumps by %v: before %v, after %v", i, times[i], d, before, after)
		}
	}

	// Sampled on either side of a key, the angular velocity therefore matches
	// to within the O(h) truncation error of the one-sided difference and the
	// float32 round-off.
	const h = 1e-4
	for i := 1; i < len(times)-1; i++ {
		at := times[i]
		left := trackBodyOmega(trk.At(at-h), trk.At(at), h)
		right := trackBodyOmega(trk.At(at), trk.At(at+h), h)
		if d := trackOmegaDiff(left, right); d > 5e-2 {
			t.Errorf("key %d (t=%v): sampled angular velocities differ by %v: left %v, right %v", i, at, d, left, right)
		}
	}

	// The angular velocity is not constant within a segment, which is what
	// distinguishes a smooth trajectory from piecewise slerp: at the midpoint
	// of a segment it differs from the value at the key.
	_, atKey := trackSideVelocities(trk, 1)
	mid := (times[1] + times[2]) / 2
	atMid := trackBodyOmega(trk.At(mid-h), trk.At(mid+h), 2*h)
	if trackOmegaDiff(atKey, atMid) < 1e-2 {
		t.Errorf("angular velocity did not vary along segment 1: key %v, mid %v", atKey, atMid)
	}

	// Contrast with plain piecewise slerp, which changes angular velocity at
	// the interior keys.
	piecewise := func(at float64) Quat {
		i := 0
		for i < len(times)-2 && at > times[i+1] {
			i++
		}
		u := Clamp((at-times[i])/(times[i+1]-times[i]), 0, 1)
		return QuatSlerp(cams[i], cams[i+1], u)
	}
	left := trackBodyOmega(piecewise(times[1]-h), piecewise(times[1]), h)
	right := trackBodyOmega(piecewise(times[1]), piecewise(times[1]+h), h)
	if trackOmegaDiff(left, right) < 1e-1 {
		t.Errorf("piecewise slerp unexpectedly has continuous angular velocity at t=%v: %v vs %v", times[1], left, right)
	}
}

func TestQuatTrackEndRules(t *testing.T) {
	cams := trackCameras()
	times := []float64{0, 0.5, 1.7, 4}

	const h = 1e-3

	clamped, err := NewQuatTrack(times, cams, QuatTrackConfig{EndRule: QuatTrackEndClamped})
	if err != nil {
		t.Fatalf("NewQuatTrack returned error: %v", err)
	}
	// The exact one-sided velocity at the ends is zero.
	if before, _ := trackSideVelocities(clamped, 0); before.Len() != 0 {
		t.Errorf("clamped start has angular velocity %v, want 0", before)
	}
	last := len(times) - 1
	if _, after := trackSideVelocities(clamped, last); after.Len() != 0 {
		t.Errorf("clamped end has angular velocity %v, want 0", after)
	}
	// A small step out from each end confirms the same through At.
	if omega := trackBodyOmega(clamped.At(times[0]), clamped.At(times[0]+h), h); omega.Len() > 1e-1 {
		t.Errorf("clamped start has angular velocity %v, want ~0", omega)
	}
	if omega := trackBodyOmega(clamped.At(times[last]-h), clamped.At(times[last]), h); omega.Len() > 1e-1 {
		t.Errorf("clamped end has angular velocity %v, want ~0", omega)
	}

	oneSided, err := NewQuatTrack(times, cams, QuatTrackConfig{EndRule: QuatTrackEndOneSided})
	if err != nil {
		t.Fatalf("NewQuatTrack returned error: %v", err)
	}
	// The one-sided start matches the first segment's velocity: the trajectory
	// does not come to rest.
	if _, after := trackSideVelocities(oneSided, 0); after.Len() == 0 {
		t.Errorf("one-sided start has zero angular velocity, want the first segment's velocity")
	}
	startOmega := trackBodyOmega(oneSided.At(times[0]), oneSided.At(times[0]+h), h)
	if startOmega.Len() < 1e-1 {
		t.Errorf("one-sided start has angular velocity %v, want the first segment's velocity", startOmega)
	}
	// Away from the ends both rules describe the same trajectory, since the
	// end rule only changes the outermost control points' influence.
	for i := 0; i < 50; i++ {
		at := times[1] + (times[2]-times[1])*float64(i)/50
		if !clamped.At(at).OrientationEqualThreshold(oneSided.At(at), 1e-5) {
			t.Fatalf("end rules disagree at interior t=%v: %v vs %v", at, clamped.At(at), oneSided.At(at))
		}
	}
}

func TestQuatTrackLoop(t *testing.T) {
	// A full yaw sweep: forward -> right -> behind -> left -> forward.
	cams := []Quat{
		QuatLookAtV(Vec3{}, Vec3{0, 0, -1}, Vec3{0, 1, 0}),
		QuatLookAtV(Vec3{}, Vec3{1, 0, 0}, Vec3{0, 1, 0}),
		QuatLookAtV(Vec3{}, Vec3{0, 0, 1}, Vec3{0, 1, 0}),
		QuatLookAtV(Vec3{}, Vec3{-1, 0, 0}, Vec3{0, 1, 0}),
	}
	keys := make([]Quat, len(cams)+1)
	copy(keys, cams)
	keys[len(cams)] = cams[0]
	times := []float64{0, 0.7, 1.9, 2.2, 3}

	trk, err := NewQuatTrack(times, keys, QuatTrackConfig{Loop: true})
	if err != nil {
		t.Fatalf("NewQuatTrack returned error: %v", err)
	}
	if !trk.Loop() || trk.Len() != len(keys) {
		t.Fatalf("unexpected loop track metadata: len=%d loop=%v", trk.Len(), trk.Loop())
	}

	// The period wraps exactly.
	first := trk.At(times[0])
	for _, k := range []float64{-1, 0, 1, 2} {
		if got := trk.At(times[0] + k*(times[4]-times[0])); !got.OrientationEqualThreshold(first, 1e-5) {
			t.Errorf("At(start + %v*period) = %v, want %v", k, got, first)
		}
	}

	// The seam is as smooth as an interior key: the exact one-sided angular
	// velocities agree, and a sampled check on either side of it passes within
	// the finite-difference error.
	if before, after := trackLoopSeamVelocities(trk); trackOmegaDiff(before, after) > 1e-4 {
		t.Errorf("loop seam angular velocities jump by %v: before %v, after %v",
			trackOmegaDiff(before, after), before, after)
	}

	const h = 1e-4
	const tol = 5e-2
	period := times[len(times)-1] - times[0]
	left := trackBodyOmega(trk.At(times[0]-h), trk.At(times[0]), h)
	// At(times[0]-h) wrapped to the end of the period.
	right := trackBodyOmega(trk.At(times[0]), trk.At(times[0]+h), h)
	if d := trackOmegaDiff(left, right); d > tol {
		t.Errorf("loop seam angular velocities differ by %v: left %v, right %v", d, left, right)
	}
	if got := trk.At(times[0] - h); !got.OrientationEqualThreshold(trk.At(times[0]+period-h), 1e-5) {
		t.Errorf("loop wrap is not periodic: %v vs %v", got, trk.At(times[0]+period-h))
	}

	for i := 1; i < len(times)-1; i++ {
		before, after := trackSideVelocities(trk, i)
		if d := trackOmegaDiff(before, after); d > 1e-4 {
			t.Errorf("loop key %d (t=%v): angular velocity jumps by %v: before %v, after %v", i, times[i], d, before, after)
		}
	}

	// A closing key given as the antipode of the first still closes the loop.
	antipodes := append([]Quat(nil), keys...)
	antipodes[len(antipodes)-1] = keys[0].Scale(-1)
	trkAntipodal, err := NewQuatTrack(times, antipodes, QuatTrackConfig{Loop: true})
	if err != nil {
		t.Fatalf("NewQuatTrack with antipodal closing key returned error: %v", err)
	}
	for i := 0; i <= 100; i++ {
		at := times[0] + (times[4]-times[0])*float64(i)/100
		if !trk.At(at).OrientationEqualThreshold(trkAntipodal.At(at), 1e-5) {
			t.Fatalf("antipodal closing key changed the loop at t=%v: %v vs %v", at, trk.At(at), trkAntipodal.At(at))
		}
	}
}

func TestQuatTrackSignAlignment(t *testing.T) {
	cams := trackCameras()
	times := []float64{0, 0.5, 1.7, 4}

	flipped := make([]Quat, len(cams))
	for i, q := range cams {
		flipped[i] = q
		if i%2 == 1 {
			flipped[i] = q.Scale(-1)
		}
	}

	a, err := NewQuatTrack(times, cams, QuatTrackConfig{})
	if err != nil {
		t.Fatalf("NewQuatTrack returned error: %v", err)
	}
	b, err := NewQuatTrack(times, flipped, QuatTrackConfig{})
	if err != nil {
		t.Fatalf("NewQuatTrack returned error: %v", err)
	}

	for i := 0; i <= 100; i++ {
		at := times[0] + (times[3]-times[0])*float64(i)/100
		if !a.At(at).OrientationEqualThreshold(b.At(at), 1e-5) {
			t.Fatalf("antipodal keys changed the trajectory at t=%v: %v vs %v", at, a.At(at), b.At(at))
		}
	}
}

func TestQuatTrackCopiesInput(t *testing.T) {
	cams := trackCameras()
	times := []float64{0, 0.5, 1.7, 4}
	trk, err := NewQuatTrack(times, cams, QuatTrackConfig{})
	if err != nil {
		t.Fatalf("NewQuatTrack returned error: %v", err)
	}
	want := trk.At(1.1)

	times[0] = 100
	cams[0] = QuatIdent().Scale(3)
	if got := trk.At(1.1); !got.ApproxEqualThreshold(want, 1e-6) {
		t.Errorf("track changed after caller mutated its inputs: %v vs %v", got, want)
	}
}
