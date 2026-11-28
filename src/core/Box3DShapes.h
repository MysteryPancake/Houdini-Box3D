// Box3DShapes.h
//
// Houdini-free, header-only helpers: shape parameters from point attributes,
// packed-prim matrix math, hashing, collision-group patterns. (Plain vector /
// quaternion math: Box3DMath.h.) No Box3D or HDK includes, so
// tests/core_physics.cpp covers them without Houdini.
#pragma once

#include "Box3DMath.h" // finite3, copy3 / copy4, kDegToRad, quaternion helpers

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

namespace b3dop
{

enum class ShapeType : int
{
	Box = 0,
	Sphere = 1,
	Convex = 2, // one convex hull of the packed geometry
	// several convex hulls on ONE body (packed geometry pieces, grouped by
	// `name`)
	Concave = 3,
	Capsule = 4,  // axis along the body's Y
	None = 5,	  // no collision shape: a static body that nothing collides with
	Plane = 6,	  // a ground slab: its top is the plane through the body origin,
				  // facing the body's +Y (groundSpec); static bodies only
	Cylinder = 7, // axis along the body's Y; a polygon prism hull (Box3DWorld)
};

// Sides of a cylinder's polygon (b3CreateCylinder accepts 3 to 32).
constexpr int kCylinderSides = 32;

// A box, sphere, capsule or cylinder in body-local space. `center` is its
// offset from the body origin (non-zero e.g. when fitted to packed geometry not
// centred on the pivot). Convex / concave shapes are hulls instead
// (Box3DWorld::createHull).
struct ShapeSpec
{
	ShapeType type = ShapeType::Box;
	float halfExtents[3] = { 0.5f, 0.5f, 0.5f }; // Box
	float radius = 0.5f;						 // Sphere, Capsule, Cylinder
	float halfLength = 0.5f;					 // Capsule: half the straight section, along Y;
												 // Cylinder: half its height
	float center[3] = { 0.0f, 0.0f, 0.0f };		 // every primitive
	float rot[4] = { 0.0f, 0.0f, 0.0f,
					 // x, y, z, w: orientation in body space. Box and capsule; a
					 // sphere ignores it
					 1.0f };
};

// A ground (a Ground Plane DOP, a static `plane` piece): a wide, deep box whose
// top face is the plane y = 0 of its body. Wide and long, so nothing walks off
// its edge; deep, so a fast body cannot get through. The defaults are well
// inside Box3D's world bounds (B3_HUGE) and small enough that float
// coordinates stay fine near the body; the user can change them (the solver's
// Ground Size).
struct GroundSize
{
	// Along x, below the surface (y), along z.
	float size[3] = { 1000.0f, 10.0f, 1000.0f };
};
constexpr float kMinGroundSize = 0.1f;
constexpr float kMaxGroundSize[3] = { 20000.0f, 5000.0f, 20000.0f }; // below B3_HUGE

// The ground as a ShapeSpec of type Plane: halfExtents are half the size on
// each axis, so a spec carries its size (and a change of size is a change of
// shape). Each size is kept within [kMinGroundSize, kMaxGroundSize]; a
// non-finite one becomes the default.
inline ShapeSpec groundSpec( const GroundSize& g = GroundSize() )
{
	const GroundSize d;
	ShapeSpec s;
	s.type = ShapeType::Plane;
	for ( int i = 0; i < 3; ++i )
	{
		const float size =
			std::isfinite( g.size[i] ) ? std::clamp( g.size[i], kMinGroundSize, kMaxGroundSize[i] ) : d.size[i];
		s.halfExtents[i] = 0.5f * size;
	}
	return s;
}

// `bullet_georep` -> ShapeType, Houdini's lowercase names, exact match.
// `compound` and `concave` are both Concave. False (out untouched) for null,
// empty or unknown values.
inline bool parseShapeType( const char* name, ShapeType& out )
{
	if ( !name || !*name )
		return false;
	struct Entry
	{
		const char* name;
		ShapeType type;
	};
	static const Entry kEntries[] = {
		{ "convexhull", ShapeType::Convex }, { "concave", ShapeType::Concave },	 { "box", ShapeType::Box },
		{ "capsule", ShapeType::Capsule },	 { "compound", ShapeType::Concave }, { "sphere", ShapeType::Sphere },
		{ "none", ShapeType::None },		 { "plane", ShapeType::Plane },		 { "cylinder", ShapeType::Cylinder },
	};
	for ( const Entry& e : kEntries )
		if ( std::strcmp( name, e.name ) == 0 )
		{
			out = e.type;
			return true;
		}
	return false;
}

// Smallest box half-size component / sphere / capsule size a shape may have.
constexpr float kMinExtent = 1e-4f;

// |v|, at least kMinExtent; non-finite -> kMinExtent.
inline float clampExtent( float v )
{
	v = std::fabs( v );
	return ( std::isfinite( v ) && v > kMinExtent ) ? v : kMinExtent;
}

// The shape fills a box of size pscale * scale (Copy to Points on a unit
// primitive). Sphere: diameter = largest component. Capsule along Y: radius =
// half the larger of x / z, total height = y. Components are made positive and
// at least 1e-4.
inline ShapeSpec makeShapeSpec( ShapeType type, float pscale, const float scale[3] )
{
	const float sx = clampExtent( pscale * scale[0] );
	const float sy = clampExtent( pscale * scale[1] );
	const float sz = clampExtent( pscale * scale[2] );

	ShapeSpec s;
	// Convex / concave: the fallback box when there is no geometry to hull.
	s.type = ( type == ShapeType::Sphere || type == ShapeType::Capsule || type == ShapeType::Cylinder )
				 ? type
				 : ShapeType::Box;
	if ( type == ShapeType::Sphere )
		s.radius = 0.5f * std::max( { sx, sy, sz } );
	else if ( type == ShapeType::Cylinder )
	{
		s.radius = 0.5f * std::max( sx, sz );
		s.halfLength = 0.5f * sy;
	}
	else if ( type == ShapeType::Capsule )
	{
		s.radius = 0.5f * std::max( sx, sz );
		s.halfLength = std::max( 0.5f * sy - s.radius, 0.0f );
	}
	else
	{
		s.halfExtents[0] = 0.5f * sx;
		s.halfExtents[1] = 0.5f * sy;
		s.halfExtents[2] = 0.5f * sz;
	}
	return s;
}

// ---------------------------------------------------------------------------
// Explicit primitives (Bullet Data with AutoFit off). Each is optional:
//   bullet_primS   box HALF extents (Bullet Data documents Box Size as a
//   half-size) bullet_radius  sphere / capsule radius bullet_length  capsule
//   straight section along Y, caps not included bullet_primT   offset from the
//   body origin bullet_primR   Euler degrees, order XYZ (the solver builds the
//   quaternion)
// ---------------------------------------------------------------------------
// The values are Box3D Data's parameter defaults: SIM_Box3DData counts a value as
// set when it differs from them.
struct PrimOverrides
{
	bool hasSize = false, hasRot = false, hasPos = false, hasRadius = false, hasLength = false;
	float size[3] = { 1.0f, 1.0f, 1.0f };
	float rot[4] = { 0.0f, 0.0f, 0.0f, 1.0f }; // unit quaternion x, y, z, w (the solver builds it with the HDK)
	float pos[3] = { 0.0f, 0.0f, 0.0f };
	float radius = 1.0f;
	float length = 1.0f;

	bool any() const
	{
		return hasSize || hasRot || hasPos || hasRadius || hasLength;
	}
};

// Non-finite values are ignored; sizes are clamped like makeShapeSpec.
inline void applyPrimOverrides( ShapeSpec& s, const PrimOverrides& o )
{
	if ( o.hasSize && s.type == ShapeType::Box && finite3( o.size ) )
		for ( int i = 0; i < 3; ++i )
			s.halfExtents[i] = clampExtent( o.size[i] );

	if ( o.hasRadius && std::isfinite( o.radius ) &&
		 ( s.type == ShapeType::Sphere || s.type == ShapeType::Capsule || s.type == ShapeType::Cylinder ) )
		s.radius = clampExtent( o.radius );

	if ( o.hasLength && std::isfinite( o.length ) && ( s.type == ShapeType::Capsule || s.type == ShapeType::Cylinder ) )
		s.halfLength = std::max( 0.5f * o.length, 0.0f );

	if ( o.hasPos && finite3( o.pos ) )
		copy3( s.center, o.pos );

	if ( o.hasRot && finite3( o.rot ) && std::isfinite( o.rot[3] ) )
		copy4( s.rot, o.rot );
}

inline float shapeVolume( const ShapeSpec& s )
{
	const float pi = kPi;
	const float r = s.radius;
	switch ( s.type )
	{
		case ShapeType::Sphere:
			return ( 4.0f / 3.0f ) * pi * r * r * r;
		case ShapeType::Capsule:
			return pi * r * r * ( 2.0f * s.halfLength ) + ( 4.0f / 3.0f ) * pi * r * r * r;
		case ShapeType::Cylinder:
			return pi * r * r * ( 2.0f * s.halfLength );
		case ShapeType::Plane:
			return 0.0f; // static: no mass
		default:
			return 8.0f * s.halfExtents[0] * s.halfExtents[1] * s.halfExtents[2];
	}
}

// Object id in the high 32 bits, the point's `id` (else its number) in the
// low 32.
inline uint64_t makeBodyKey( int objectId, int64_t pointId )
{
	return ( static_cast<uint64_t>( static_cast<uint32_t>( objectId ) ) << 32 ) |
		   static_cast<uint64_t>( static_cast<uint32_t>( pointId ) );
}

// ---------------------------------------------------------------------------
// Packed primitives and hull point preparation
//
// Houdini matrices are row-vector (p' = p * M, translation in row 3);
// everything here uses that layout (m3[row*3+col], m4[row*4+col]). Quaternions
// are x, y, z, w.
// ---------------------------------------------------------------------------

// The upper-left 3x3 of a 4x4 (both row-major).
inline void upper3x3( const double m4[16], double m3[9] )
{
	for ( int r = 0; r < 3; ++r )
		for ( int c = 0; c < 3; ++c )
			m3[r * 3 + c] = m4[r * 4 + c];
}

// A 3x3's row lengths (the per-row scale of M = S * R). False if a row is zero
// or non-finite. The plugin splits rotations with the HDK
// (UT_Matrix3D::extractScales); this Houdini-free version is only for scale
// keys.
inline bool matrixRowLengths( const double m[9], double out[3] )
{
	out[0] = out[1] = out[2] = 1.0;
	double l[3];
	for ( int r = 0; r < 3; ++r )
	{
		l[r] = std::sqrt( m[r * 3] * m[r * 3] + m[r * 3 + 1] * m[r * 3 + 1] + m[r * 3 + 2] * m[r * 3 + 2] );
		if ( !( l[r] > 1e-12 ) || !std::isfinite( l[r] ) )
			return false;
	}
	out[0] = l[0];
	out[1] = l[1];
	out[2] = l[2];
	return true;
}

// A 3x3's determinant: negative when the matrix mirrors.
inline double matrixDeterminant( const double m[9] )
{
	return m[0] * ( m[4] * m[8] - m[5] * m[7] ) - m[1] * ( m[3] * m[8] - m[5] * m[6] ) +
		   m[2] * ( m[3] * m[7] - m[4] * m[6] );
}

// A mirrored packed 3x3 is read as its rotation and scale with the third row
// negated (splitPackedMatrix), which leaves a local z flip in front of them:
// M = diag(1, 1, -1) * S * R. This negates `m`'s third row when `before`
// (the 3x3 the last sync read) mirrored, so a 3x3 composed from a new rotation
// keeps the mirror.
inline void keepMirror( bool mirrored, double m[9] )
{
	if ( !mirrored )
		return;
	for ( int c = 0; c < 3; ++c )
		m[6 + c] = -m[6 + c];
}
inline void keepMirror( const double before[9], double m[9] )
{
	keepMirror( matrixDeterminant( before ) < 0.0, m );
}

// The scale packedPointsToBodyLocal bakes in: m4's row lengths, rounded to 5
// significant digits so instances differing by float noise share a hull. A
// mirroring 3x3 has a negative third entry (the local z flip), so the points
// come out mirrored and the hull key differs from the unmirrored piece's.
// False if degenerate.
inline bool packedScale( const double m4[16], double S[3] )
{
	double m3[9];
	upper3x3( m4, m3 );
	if ( !matrixRowLengths( m3, S ) )
		return false;
	// Round to 5 significant digits.
	for ( int j = 0; j < 3; ++j )
	{
		const double v = S[j];
		if ( !( std::fabs( v ) > 0.0 ) || !std::isfinite( v ) )
			continue;
		const double step = std::pow( 10.0, std::floor( std::log10( std::fabs( v ) ) ) - 4.0 );
		S[j] = std::round( v / step ) * step;
	}
	if ( matrixDeterminant( m3 ) < 0.0 )
		S[2] = -S[2];
	return true;
}

// Hashing for hull keys and signatures: 8 bytes per round (a multiply and a
// shift), the tail byte by byte. Keys only live for the process, so the exact
// function may change freely.
constexpr uint64_t kHashSeed = 1469598103934665603ull;
constexpr uint64_t kHashMul = 0xff51afd7ed558ccdull;

// Mixes one 64-bit value in (by value, so the same on any byte order). The
// value is spread first (an odd multiply, so no two values meet) and then xored
// in: without that the result depends on `h ^ v` alone, and two raw, structured
// values (small ids) with the same xor collide, as mixHash(a, b) and
// mixHash(c, d) do whenever a ^ b == c ^ d.
constexpr uint64_t kHashSpread = 0x9e3779b97f4a7c15ull;
inline uint64_t mixHash( uint64_t h, uint64_t v )
{
	h = ( h ^ ( v * kHashSpread ) ) * kHashMul;
	return h ^ ( h >> 32 );
}

inline uint64_t hashBytes( uint64_t h, const void* data, size_t byteCount )
{
	const unsigned char* b = static_cast<const unsigned char*>( data );
	size_t i = 0;
	for ( ; i + 8 <= byteCount; i += 8 )
	{
		uint64_t w;
		std::memcpy( &w, b + i, 8 );
		h = mixHash( h, w );
	}
	uint64_t tail = 0;
	for ( size_t k = 0; i < byteCount; ++i, ++k )
		tail |= static_cast<uint64_t>( b[i] ) << ( 8 * k );
	return mixHash( h, tail ^ ( static_cast<uint64_t>( byteCount ) << 56 ) );
}

// The raw bytes of `floatCount` floats, with the count mixed in.
inline uint64_t hashFloats( const float* xyz, size_t floatCount )
{
	return hashBytes( kHashSeed ^ static_cast<uint64_t>( floatCount ), xyz, floatCount * sizeof( float ) );
}

// A double's bits; -0 and +0 hash alike.
inline uint64_t mixDouble( uint64_t h, double d )
{
	d += 0.0;
	uint64_t u;
	std::memcpy( &u, &d, sizeof u );
	return mixHash( h, u );
}

// Every field of a shape spec: equal specs hash alike.
inline uint64_t hashShapeSpec( const ShapeSpec& sp )
{
	// 0.0f + x turns -0 into +0, so equal specs hash alike.
	auto z = []( float v ) { return v + 0.0f; };
	const float f[] = { static_cast<float>( static_cast<int>( sp.type ) ),
						z( sp.halfExtents[0] ),
						z( sp.halfExtents[1] ),
						z( sp.halfExtents[2] ),
						z( sp.radius ),
						z( sp.halfLength ),
						z( sp.center[0] ),
						z( sp.center[1] ),
						z( sp.center[2] ),
						z( sp.rot[0] ),
						z( sp.rot[1] ),
						z( sp.rot[2] ),
						z( sp.rot[3] ) };
	return hashFloats( f, sizeof( f ) / sizeof( f[0] ) );
}

// A packed hull's cache key: the source's point hash plus what differs per
// instance (pivot, scale). `kind`: 1 merged hull, 2 per piece. Same inputs as
// packedPointsToBodyLocal, so a hit means the same body-local points. Never 0.
inline uint64_t packedHullKey( uint64_t sourceHash, const double pivot[3], const double S[3], uint32_t kind )
{
	uint64_t h = mixHash( sourceHash, kind );
	for ( int j = 0; j < 3; ++j )
		h = mixDouble( mixDouble( h, pivot[j] ), S[j] );
	return h ? h : 1ull;
}

// A packed AutoFit's cache key (Box3DWorld::findFit): the source's pieces hash,
// pivot and scale (the inputs of packedPointsToBodyLocal) and the shape type,
// so instances of one source share the fit without their points being moved
// into body space and hashed. Never 0.
inline uint64_t packedFitKey( uint64_t sourceHash, const double pivot[3], const double S[3], ShapeType type )
{
	constexpr uint32_t kFitKindBase = 16; // above packedHullKey's hull kinds
	return packedHullKey( sourceHash, pivot, S, kFitKindBase + static_cast<uint32_t>( type ) );
}

// Connected sets of primitives (sharing points, directly or not).
// primPoints[i]: points of primitive i. Returns each primitive's set, numbered
// in order of first appearance.
template <class PointId> std::vector<int> connectedSets( const std::vector<std::vector<PointId>>& primPoints )
{
	const size_t n = primPoints.size();
	std::vector<size_t> parent( n );
	for ( size_t i = 0; i < n; ++i )
		parent[i] = i;
	auto root = [&]( size_t i ) {
		while ( parent[i] != i )
			i = parent[i] = parent[parent[i]];
		return i;
	};

	std::unordered_map<PointId, size_t> firstUser; // point -> first primitive that uses it
	for ( size_t i = 0; i < n; ++i )
		for ( const PointId& pt : primPoints[i] )
		{
			const auto ins = firstUser.emplace( pt, i );
			if ( ins.second )
				continue;
			const size_t a = root( ins.first->second ), b = root( i );
			if ( a != b )
				parent[a > b ? a : b] = a > b ? b : a;
		}

	std::vector<int> label( n, -1 ), sets( n );
	int next = 0;
	for ( size_t i = 0; i < n; ++i )
	{
		const size_t r = root( i );
		if ( label[r] < 0 )
			label[r] = next++;
		sets[i] = label[r];
	}
	return sets;
}

// A packed primitive's inner points in body-local space: (p - pivot) * S, with
// m4 the full transform (only its 3x3 is read, M = S * R) and pivot
// GEO_PrimPacked::getPivot (the inner point at the packed point's P, i.e. the
// body origin and centre of mass). S is rounded (packedScale) so rotated
// instances produce identical points. Degenerate m4: unchanged.
inline void packedPointsToBodyLocal( const float* xyz, int count, const double m4[16], const double pivotIn[3],
									 std::vector<float>& out )
{
	out.assign( xyz, xyz + static_cast<size_t>( count ) * 3 );
	if ( count <= 0 )
		return;

	double S[3];
	if ( !packedScale( m4, S ) )
		return;

	for ( int i = 0; i < count; ++i )
		for ( int j = 0; j < 3; ++j )
			out[static_cast<size_t>( 3 * i + j )] =
				static_cast<float>( ( static_cast<double>( xyz[3 * i + j] ) - pivotIn[j] ) * S[j] );
}

// The body axes as a fitting frame (rows: the frame's axes in body space).
inline const float ( &identityFrame() )[3][3]
{
	static const float kIdentity[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
	return kIdentity;
}

// A box / sphere / capsule fitted to the points along `axes` (three
// orthonormal, right-handed rows in body space), as Bullet's AutoFit does
// (measured against it, test autofit_bullet). Box: the points' bounds along
// the axes, turned with them. Sphere: centred on those bounds, just enclosing
// every point. Capsule: along the second axis (the one closest to the body's
// Y after canonicalFrame), centred on the bounds, radius just enclosing every
// point around that axis, ending where the bounds end along it (measured:
// Bullet's capsule on a standing 0.4 x 1.2 x 0.4 piece tops out at 0.6).
// Extents are clamped like makeShapeSpec. False for no points or a non-finite
// one.
inline bool fitShapeInFrame( ShapeType type, const float* xyz, int count, const float axes[3][3], ShapeSpec& out )
{
	if ( count <= 0 )
		return false;
	float lo[3], hi[3];
	for ( int i = 0; i < count; ++i )
	{
		const float* p = xyz + 3 * i;
		if ( !finite3( p ) )
			return false;
		for ( int k = 0; k < 3; ++k )
		{
			const float d = p[0] * axes[k][0] + p[1] * axes[k][1] + p[2] * axes[k][2];
			if ( i == 0 || d < lo[k] )
				lo[k] = d;
			if ( i == 0 || d > hi[k] )
				hi[k] = d;
		}
	}

	float half[3], mid[3];
	for ( int k = 0; k < 3; ++k )
	{
		mid[k] = 0.5f * ( lo[k] + hi[k] );
		half[k] = 0.5f * clampExtent( hi[k] - lo[k] );
	}

	out = ShapeSpec();
	for ( int j = 0; j < 3; ++j )
		out.center[j] = mid[0] * axes[0][j] + mid[1] * axes[1][j] + mid[2] * axes[2][j];
	const bool turned = !( axes[0][0] == 1.0f && axes[1][1] == 1.0f && axes[2][2] == 1.0f ); // keeps an exact identity

	// Per point, relative to the centre: along the second axis, and the squared
	// distance from that axis (in doubles: r^2 - d^2 cancels at the rim).
	auto relative = [&]( int i, double& along, double& dist2 ) {
		double v[3];
		for ( int j = 0; j < 3; ++j )
			v[j] = static_cast<double>( xyz[3 * i + j] ) - out.center[j];
		along = v[0] * axes[1][0] + v[1] * axes[1][1] + v[2] * axes[1][2];
		dist2 = std::max( 0.0, v[0] * v[0] + v[1] * v[1] + v[2] * v[2] - along * along );
	};

	if ( type == ShapeType::Sphere )
	{
		double r2 = 0.0;
		for ( int i = 0; i < count; ++i )
		{
			double along, dist2;
			relative( i, along, dist2 );
			r2 = std::max( r2, dist2 + along * along );
		}
		out.type = ShapeType::Sphere;
		out.radius = clampExtent( static_cast<float>( std::sqrt( r2 ) ) );
	}
	else if ( type == ShapeType::Capsule || type == ShapeType::Cylinder )
	{
		double r2 = 0.0;
		for ( int i = 0; i < count; ++i )
		{
			double along, dist2;
			relative( i, along, dist2 );
			r2 = std::max( r2, dist2 );
		}
		out.type = type;
		out.radius = clampExtent( static_cast<float>( std::sqrt( r2 ) ) );
		// Bullet keeps the whole capsule within the bounds along the axis: the
		// caps end where the piece does (a sphere when the radius is larger). A
		// cylinder's flat ends sit on the bounds.
		out.halfLength = type == ShapeType::Cylinder ? half[1] : std::max( half[1] - out.radius, 0.0f );
		if ( turned )
			quatFromAxes( axes[0], axes[1], axes[2], out.rot );
	}
	else
	{
		out.type = ShapeType::Box;
		copy3( out.halfExtents, half );
		if ( turned )
			quatFromAxes( axes[0], axes[1], axes[2], out.rot );
	}
	return true;
}

// A box / sphere / capsule fitted to the points' bounds along the body axes:
// the fallback when points cannot be hulled (fitShapeOriented in Box3DWorld.h
// is AutoFit, which also tries turned frames).
inline bool fitShapeToPoints( ShapeType type, const float* xyz, int count, ShapeSpec& out )
{
	return fitShapeInFrame( type, xyz, count, identityFrame(), out );
}

// Turns an orthonormal frame (rows) into the closest one to the body axes that
// spans the same box: each body axis in turn takes the remaining frame axis
// most parallel to it, pointing its way; the third is their cross product, so
// the result is right-handed. A frame already along the body axes becomes the
// exact identity.
inline void canonicalFrame( float axes[3][3] )
{
	float in[3][3];
	std::memcpy( in, axes, sizeof in );
	bool used[3] = { false, false, false };
	for ( int j = 0; j < 2; ++j )
	{
		int best = -1;
		for ( int k = 0; k < 3; ++k )
			if ( !used[k] && ( best < 0 || std::fabs( in[k][j] ) > std::fabs( in[best][j] ) ) )
				best = k;
		used[best] = true;
		const float s = in[best][j] < 0.0f ? -1.0f : 1.0f;
		for ( int c = 0; c < 3; ++c )
			axes[j][c] = s * in[best][c];
	}
	axes[2][0] = axes[0][1] * axes[1][2] - axes[0][2] * axes[1][1];
	axes[2][1] = axes[0][2] * axes[1][0] - axes[0][0] * axes[1][2];
	axes[2][2] = axes[0][0] * axes[1][1] - axes[0][1] * axes[1][0];
	// Snap float noise so an unturned frame is exactly the identity.
	for ( int r = 0; r < 3; ++r )
		for ( int c = 0; c < 3; ++c )
		{
			const float want = ( r == c ) ? 1.0f : 0.0f;
			if ( std::fabs( axes[r][c] - want ) < 1e-6f )
				axes[r][c] = want;
		}
}

// symmetryAxisFrame: below this gap between one variance and the mean of the
// other two (relative to their mean), the variances do not tell which axis the
// points are round about.
constexpr double kSymmetryAxisGap = 0.1;

// The eigenvalues and unit eigenvectors (rows) of the symmetric 3x3 matrix
// `a` (xx, yy, zz, xy, xz, yz), by cyclic Jacobi rotations.
inline void symmetricEigen3( const double a[6], double values[3], double vectors[3][3] )
{
	double m[3][3] = { { a[0], a[3], a[4] }, { a[3], a[1], a[5] }, { a[4], a[5], a[2] } };
	double v[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } }; // columns: the eigenvectors
	for ( int sweep = 0; sweep < 32; ++sweep )
	{
		const double off = m[0][1] * m[0][1] + m[0][2] * m[0][2] + m[1][2] * m[1][2];
		const double diag = m[0][0] * m[0][0] + m[1][1] * m[1][1] + m[2][2] * m[2][2];
		if ( off <= 1e-30 * diag || off == 0.0 )
			break;
		for ( int p = 0; p < 2; ++p )
			for ( int q = p + 1; q < 3; ++q )
			{
				if ( m[p][q] == 0.0 )
					continue;
				const double theta = ( m[q][q] - m[p][p] ) / ( 2.0 * m[p][q] );
				const double t =
					( theta >= 0.0 ? 1.0 : -1.0 ) / ( std::fabs( theta ) + std::sqrt( theta * theta + 1.0 ) );
				const double c = 1.0 / std::sqrt( t * t + 1.0 ), sn = t * c;
				for ( int k = 0; k < 3; ++k ) // m = m J
				{
					const double mkp = m[k][p], mkq = m[k][q];
					m[k][p] = c * mkp - sn * mkq;
					m[k][q] = sn * mkp + c * mkq;
				}
				for ( int k = 0; k < 3; ++k ) // m = J^T m
				{
					const double mpk = m[p][k], mqk = m[q][k];
					m[p][k] = c * mpk - sn * mqk;
					m[q][k] = sn * mpk + c * mqk;
				}
				for ( int k = 0; k < 3; ++k )
				{
					const double vkp = v[k][p], vkq = v[k][q];
					v[k][p] = c * vkp - sn * vkq;
					v[k][q] = sn * vkp + c * vkq;
				}
			}
	}
	for ( int i = 0; i < 3; ++i )
	{
		values[i] = m[i][i];
		for ( int k = 0; k < 3; ++k )
			vectors[i][k] = v[k][i];
	}
}

// A fitting frame (rows, right-handed) whose second axis is the points'
// axis of symmetry, for a capsule or cylinder that runs along its piece (a
// piece's `piece_georep`; a body's AutoFit runs along the axis nearest Y
// instead, as Bullet's does): of the principal axes of the points' spread, the
// one whose variance differs most from the other two, which is the axis of a
// round rod or disc, and the long side of a box; when none differs enough
// (kSymmetryAxisGap), the one the points are round about. An
// axis within 1e-6 of a body axis becomes it exactly. The first axis is the
// body axis least along it, made perpendicular. False without points.
inline bool symmetryAxisFrame( const float* xyz, int count, float axes[3][3] )
{
	if ( !xyz || count <= 0 )
		return false;
	double mean[3] = { 0, 0, 0 };
	for ( int i = 0; i < count; ++i )
		for ( int k = 0; k < 3; ++k )
			mean[k] += static_cast<double>( xyz[3 * i + k] );
	for ( double& m : mean )
		m /= static_cast<double>( count );
	double cov[6] = { 0, 0, 0, 0, 0, 0 };
	for ( int i = 0; i < count; ++i )
	{
		double d[3];
		for ( int k = 0; k < 3; ++k )
			d[k] = static_cast<double>( xyz[3 * i + k] ) - mean[k];
		cov[0] += d[0] * d[0];
		cov[1] += d[1] * d[1];
		cov[2] += d[2] * d[2];
		cov[3] += d[0] * d[1];
		cov[4] += d[0] * d[2];
		cov[5] += d[1] * d[2];
	}
	double values[3], vectors[3][3];
	symmetricEigen3( cov, values, vectors );
	int pick = 0;
	double apart = -1.0;
	for ( int i = 0; i < 3; ++i )
	{
		const double d = std::fabs( values[i] - 0.5 * ( values[( i + 1 ) % 3] + values[( i + 2 ) % 3] ) );
		if ( d > apart )
		{
			apart = d;
			pick = i;
		}
	}
	// A round rod about 0.7 times as long as it is wide spreads as far along its
	// axis as across it, so the variances cannot tell: then the axis is the one
	// the points are round about (the least anisotropy of their spread in the
	// plane across it, to fourth order, which an n-gon with n other than 1, 2
	// or 4 has none of).
	const double mean3 = ( values[0] + values[1] + values[2] ) / 3.0;
	if ( apart < kSymmetryAxisGap * mean3 )
	{
		double best = -1.0;
		for ( int i = 0; i < 3; ++i )
		{
			const double* u = vectors[( i + 1 ) % 3];
			const double* w = vectors[( i + 2 ) % 3];
			double c2 = 0, s2 = 0, c4 = 0, s4 = 0, r2 = 0, r4 = 0;
			for ( int k = 0; k < count; ++k )
			{
				double d[3];
				for ( int j = 0; j < 3; ++j )
					d[j] = static_cast<double>( xyz[3 * k + j] ) - mean[j];
				const double x = d[0] * u[0] + d[1] * u[1] + d[2] * u[2];
				const double y = d[0] * w[0] + d[1] * w[1] + d[2] * w[2];
				const double re = x * x - y * y, im = 2.0 * x * y; // (x + iy)^2
				c2 += re;
				s2 += im;
				c4 += re * re - im * im;
				s4 += 2.0 * re * im;
				r2 += x * x + y * y;
				r4 += ( x * x + y * y ) * ( x * x + y * y );
			}
			const double score =
				( r2 > 0.0 ? std::hypot( c2, s2 ) / r2 : 0.0 ) + ( r4 > 0.0 ? std::hypot( c4, s4 ) / r4 : 0.0 );
			if ( best < 0.0 || score < best )
			{
				best = score;
				pick = i;
			}
		}
	}
	double a[3] = { vectors[pick][0], vectors[pick][1], vectors[pick][2] };
	int along = 0, across = 0;
	for ( int k = 1; k < 3; ++k )
	{
		if ( std::fabs( a[k] ) > std::fabs( a[along] ) )
			along = k;
		if ( std::fabs( a[k] ) < std::fabs( a[across] ) )
			across = k;
	}
	if ( std::fabs( a[( along + 1 ) % 3] ) < 1e-6 && std::fabs( a[( along + 2 ) % 3] ) < 1e-6 )
	{
		const double sign = a[along] < 0.0 ? -1.0 : 1.0;
		a[0] = a[1] = a[2] = 0.0;
		a[along] = sign;
		across = ( along + 1 ) % 3;
	}
	double n = std::sqrt( a[0] * a[0] + a[1] * a[1] + a[2] * a[2] );
	for ( double& c : a )
		c /= n;
	// The first axis: the body axis least along `a`, without its `a` part.
	double u[3] = { 0, 0, 0 };
	u[across] = 1.0;
	for ( int k = 0; k < 3; ++k )
		u[k] -= a[across] * a[k];
	n = std::sqrt( u[0] * u[0] + u[1] * u[1] + u[2] * u[2] );
	for ( double& c : u )
		c /= n;
	const double w[3] = { u[1] * a[2] - u[2] * a[1], u[2] * a[0] - u[0] * a[2], u[0] * a[1] - u[1] * a[0] };
	for ( int k = 0; k < 3; ++k )
	{
		axes[0][k] = static_cast<float>( u[k] );
		axes[1][k] = static_cast<float>( a[k] );
		axes[2][k] = static_cast<float>( w[k] );
	}
	return true;
}

} // namespace b3dop
