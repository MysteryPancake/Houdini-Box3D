// Box3DMath.h
//
// Houdini-free, header-only vector and quaternion helpers with no other
// dependencies: unit constants, finite checks, small fixed-size copies,
// quaternion handling. Quaternions are x, y, z, w; matrices are row-vector (p'
// = [p 1] * M, translation in row 3).
#pragma once

#include <algorithm>
#include <cmath>

namespace b3dop
{

// Every component of a 3-vector (float or double) is finite.
template <class T> inline bool finite3( const T* v )
{
	return std::isfinite( v[0] ) && std::isfinite( v[1] ) && std::isfinite( v[2] );
}

// Finite and >= 0, else `fallback` (a negative value is replaced, not clamped).
inline float nonNegOr( float v, float fallback = 0.0f )
{
	return ( std::isfinite( v ) && v >= 0.0f ) ? v : fallback;
}

// Negative clamped to 0, non-finite replaced by `fallback` (unlike nonNegOr,
// which replaces a negative value too).
inline float clampNonNegOr( float v, float fallback )
{
	if ( !std::isfinite( v ) )
		return fallback;
	return v > 0.0f ? v : 0.0f;
}

// Equal, or both NaN: a setting that is NaN and has not changed is not a
// change.
inline bool sameFloat( float a, float b )
{
	return a == b || ( a != a && b != b );
}

// Dot product, cross product (out must not alias an input) and length of
// 3-vectors (float or double).
template <class T> inline T dot3( const T* a, const T* b )
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

template <class T> inline void cross3( const T* a, const T* b, T* out )
{
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}

template <class T> inline T length3( const T* v )
{
	return std::sqrt( dot3( v, v ) );
}

// dst[0..2] = src[0..2] / dst[0..3] = src[0..3]. The element types may differ
// (double -> float).
template <class D, class S> inline void copy3( D* dst, const S* src )
{
	for ( int i = 0; i < 3; ++i )
		dst[i] = static_cast<D>( src[i] );
}

template <class D, class S> inline void copy4( D* dst, const S* src )
{
	for ( int i = 0; i < 4; ++i )
		dst[i] = static_cast<D>( src[i] );
}

// ---------------------------------------------------------------------------
// How values change on their way into and out of the geometry. Shared by the
// read path and the write-back's prediction, so the solver can tell its own
// writes from other edits.
// ---------------------------------------------------------------------------

constexpr float kPi = 3.14159265358979f;
constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kRadToDeg = 57.29577951308232f;

// Two rotations differ beyond float noise (q and -q are the same rotation):
// components compared after aligning signs. A dot-product test would only see
// rotations above ~0.16 degrees.
inline bool quatsDiffer( const float a[4], const float b[4], double eps = 1e-5 )
{
	double dot = 0.0;
	for ( int i = 0; i < 4; ++i )
		dot += static_cast<double>( a[i] ) * static_cast<double>( b[i] );
	const double sign = dot < 0.0 ? -1.0 : 1.0;
	for ( int i = 0; i < 4; ++i )
		if ( std::fabs( static_cast<double>( a[i] ) - sign * static_cast<double>( b[i] ) ) > eps )
			return true;
	return false;
}

// The inverse inertia of a solid box of `mass` whose full side lengths are
// `extents`, about its centre, as xx yy zz xy xz yz (the layout of
// BodyProps::inverseInertia). False, `out` untouched, if the mass or a side is
// not positive and finite.
inline bool boundsInverseInertia( const float extents[3], float mass, float out[6] )
{
	if ( !( mass > 0.0f ) || !std::isfinite( mass ) )
		return false;
	double sq[3];
	for ( int i = 0; i < 3; ++i )
	{
		if ( !( extents[i] > 0.0f ) || !std::isfinite( extents[i] ) )
			return false;
		sq[i] = static_cast<double>( extents[i] ) * extents[i];
	}
	const double m = mass / 12.0;
	const double inertia[3] = { m * ( sq[1] + sq[2] ), m * ( sq[0] + sq[2] ), m * ( sq[0] + sq[1] ) };
	float inv[3];
	for ( int i = 0; i < 3; ++i )
	{
		inv[i] = static_cast<float>( 1.0 / inertia[i] );
		if ( !std::isfinite( inv[i] ) )
			return false;
	}
	out[0] = inv[0];
	out[1] = inv[1];
	out[2] = inv[2];
	out[3] = out[4] = out[5] = 0.0f;
	return true;
}

// What Bullet adds to a hull's bounding box before taking the inertia of the
// box (the collision margin, counted on both sides and again by the inertia
// itself; 0.08 for its 0.02 default, as measured: bullet_hull_inertia).
constexpr float kBulletInertiaPad = 0.08f;

// boundsInverseInertia for a box whose sides lie along a frame turned by `rot`
// (x, y, z, w) from the body's: the frame the extents are measured in is the
// rest frame, and `rot` takes body-space vectors into it. The tensor comes out
// in the body's axes (symmetric: xx yy zz xy xz yz). Null `rot`: the same axes.
inline bool boundsInverseInertiaIn( const float extents[3], float mass, const float* rot, float out[6] )
{
	float diag[6];
	if ( !boundsInverseInertia( extents, mass, diag ) )
		return false;
	if ( !rot )
	{
		std::copy( diag, diag + 6, out );
		return true;
	}
	// Rows of the rotation matrix: the rest axes in body coordinates.
	const double x = rot[0], y = rot[1], z = rot[2], w = rot[3];
	const double r[3][3] = { { 1 - 2 * ( y * y + z * z ), 2 * ( x * y - z * w ), 2 * ( x * z + y * w ) },
							 { 2 * ( x * y + z * w ), 1 - 2 * ( x * x + z * z ), 2 * ( y * z - x * w ) },
							 { 2 * ( x * z - y * w ), 2 * ( y * z + x * w ), 1 - 2 * ( x * x + y * y ) } };
	const int pair[6][2] = { { 0, 0 }, { 1, 1 }, { 2, 2 }, { 0, 1 }, { 0, 2 }, { 1, 2 } };
	for ( int n = 0; n < 6; ++n )
	{
		double v = 0.0;
		for ( int k = 0; k < 3; ++k )
			v += diag[k] * r[k][pair[n][0]] * r[k][pair[n][1]];
		out[n] = static_cast<float>( v );
		if ( !std::isfinite( out[n] ) )
			return false;
	}
	return true;
}

// Inverse of a symmetric 3x3 matrix stored as xx, yy, zz, xy, xz, yz (the
// layout of BodyProps::inverseInertia). False, `out` untouched, if the matrix
// is not finite or not positive definite (an inertia tensor and its inverse
// always are).
inline bool invertSymmetric3( const float in[6], float out[6] )
{
	for ( int i = 0; i < 6; ++i )
		if ( !std::isfinite( in[i] ) )
			return false;
	const double a = in[0], b = in[1], c = in[2];
	const double d = in[3], e = in[4], f = in[5];
	// Leading principal minors, all positive for a positive definite matrix.
	const double m2 = a * b - d * d;
	const double det = a * ( b * c - f * f ) - d * ( d * c - f * e ) + e * ( d * f - b * e );
	if ( !( a > 0.0 ) || !( m2 > 0.0 ) || !( det > 0.0 ) || !std::isfinite( det ) )
		return false;
	const double inv[6] = { ( b * c - f * f ) / det, ( a * c - e * e ) / det, ( a * b - d * d ) / det,
							( e * f - d * c ) / det, ( d * f - b * e ) / det, ( d * e - a * f ) / det };
	for ( const double v : inv )
		if ( !std::isfinite( v ) )
			return false;
	for ( int i = 0; i < 6; ++i )
		out[i] = static_cast<float>( inv[i] );
	return true;
}

// A point through a row-vector 4x4 (Houdini layout: p' = [p 1] * M, translation
// in row 3).
inline void xformPoint( const double m[16], const double p[3], double out[3] )
{
	for ( int c = 0; c < 3; ++c )
		out[c] = p[0] * m[c] + p[1] * m[4 + c] + p[2] * m[8 + c] + m[12 + c];
}

// Box3D asserts on non-unit quaternions; attributes may hold zeros or off-unit
// values.
inline void sanitizeQuat( float q[4] )
{
	double len2 = 0.0;
	for ( int i = 0; i < 4; ++i )
		len2 += static_cast<double>( q[i] ) * static_cast<double>( q[i] );
	if ( !( len2 > 1e-12 ) || !std::isfinite( len2 ) )
	{
		q[0] = q[1] = q[2] = 0.0f;
		q[3] = 1.0f;
		return;
	}
	const float inv = static_cast<float>( 1.0 / std::sqrt( len2 ) );
	for ( int i = 0; i < 4; ++i )
		q[i] *= inv;
}

// The unit quaternion of the rotation whose matrix has columns x, y, z (an
// orthonormal right-handed frame: it turns the unit axes into x, y, z).
inline void quatFromAxes( const float x[3], const float y[3], const float z[3], float q[4] )
{
	// m[r][c]: column c is the image of unit axis c.
	const double m00 = x[0], m10 = x[1], m20 = x[2];
	const double m01 = y[0], m11 = y[1], m21 = y[2];
	const double m02 = z[0], m12 = z[1], m22 = z[2];
	const double trace = m00 + m11 + m22;
	double qx, qy, qz, qw;
	if ( trace > 0.0 )
	{
		const double s = 0.5 / std::sqrt( trace + 1.0 );
		qw = 0.25 / s;
		qx = ( m21 - m12 ) * s;
		qy = ( m02 - m20 ) * s;
		qz = ( m10 - m01 ) * s;
	}
	else if ( m00 > m11 && m00 > m22 )
	{
		const double s = 2.0 * std::sqrt( 1.0 + m00 - m11 - m22 );
		qw = ( m21 - m12 ) / s;
		qx = 0.25 * s;
		qy = ( m01 + m10 ) / s;
		qz = ( m02 + m20 ) / s;
	}
	else if ( m11 > m22 )
	{
		const double s = 2.0 * std::sqrt( 1.0 + m11 - m00 - m22 );
		qw = ( m02 - m20 ) / s;
		qx = ( m01 + m10 ) / s;
		qy = 0.25 * s;
		qz = ( m12 + m21 ) / s;
	}
	else
	{
		const double s = 2.0 * std::sqrt( 1.0 + m22 - m00 - m11 );
		qw = ( m10 - m01 ) / s;
		qx = ( m02 + m20 ) / s;
		qy = ( m12 + m21 ) / s;
		qz = 0.25 * s;
	}
	q[0] = static_cast<float>( qx );
	q[1] = static_cast<float>( qy );
	q[2] = static_cast<float>( qz );
	q[3] = static_cast<float>( qw );
	sanitizeQuat( q );
}

// out = a * b (b applied first, then a). `out` may be a or b.
inline void quatMul( const float a[4], const float b[4], float out[4] )
{
	const float x = a[3] * b[0] + b[3] * a[0] + a[1] * b[2] - a[2] * b[1];
	const float y = a[3] * b[1] + b[3] * a[1] + a[2] * b[0] - a[0] * b[2];
	const float z = a[3] * b[2] + b[3] * a[2] + a[0] * b[1] - a[1] * b[0];
	const float w = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
	out[0] = x;
	out[1] = y;
	out[2] = z;
	out[3] = w;
}

// The conjugate: the inverse of a unit quaternion.
inline void quatConj( const float q[4], float out[4] )
{
	out[0] = -q[0];
	out[1] = -q[1];
	out[2] = -q[2];
	out[3] = q[3];
}

// Rotate v by the unit quaternion q. T is float or double: the maths runs in T.
template <class T> inline void quatRotate( const float q[4], const T v[3], T out[3] )
{
	const T qx = q[0], qy = q[1], qz = q[2], qw = q[3];
	// t = 2 * cross(q.xyz, v);  v' = v + w * t + cross(q.xyz, t)
	const T two = static_cast<T>( 2 );
	const T tx = two * ( qy * v[2] - qz * v[1] );
	const T ty = two * ( qz * v[0] - qx * v[2] );
	const T tz = two * ( qx * v[1] - qy * v[0] );
	out[0] = v[0] + qw * tx + ( qy * tz - qz * ty );
	out[1] = v[1] + qw * ty + ( qz * tx - qx * tz );
	out[2] = v[2] + qw * tz + ( qx * ty - qy * tx );
}

// A body's position and rotation.
struct BodyPose
{
	double p[3] = { 0, 0, 0 };
	float q[4] = { 0, 0, 0, 1 };
};

// What P holds after `in` is written to it: the double itself for a 64-bit P
// (`exact`), else double -> float -> double. False if not finite.
inline bool storedPosition( const double in[3], double out[3], bool exact = false )
{
	for ( int i = 0; i < 3; ++i )
		out[i] = exact ? in[i] : static_cast<double>( static_cast<float>( in[i] ) );
	return std::isfinite( out[0] ) && std::isfinite( out[1] ) && std::isfinite( out[2] );
}

} // namespace b3dop
