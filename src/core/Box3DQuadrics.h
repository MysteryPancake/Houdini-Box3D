// Box3DQuadrics.h
//
// Houdini-free, header-only: surface points for Houdini's quadric primitives
// (Sphere, Tube, Circle, Metaball), which have one point and a transform
// instead of polygons. The shape builders read points only, so each quadric is
// turned into points on its surface and the hull / box / sphere fit works on
// those. Sizes follow what the RBD Bullet Solver does with the same primitives
// (measured by tests/houdini/checks_bullet.py: bullet_primitive_types).
//
// The matrix is the primitive's local transform as a math matrix A (p' = A * p,
// row-major), which maps the unit primitive: a sphere of radius 1; a tube along
// Y from -0.5 to 0.5 with radius 1; a circle of radius 1 in the XY plane.
// (Houdini's UT_Matrix3 multiplies row vectors, so the caller passes its
// transpose.)
#pragma once

#include <cctype>
#include <cmath>
#include <cstring>
#include <vector>

namespace b3dop
{

enum class QuadricKind
{
	None,
	Sphere,
	Tube,
	Circle,
	Metaball
};

// Kind from the primitive's type name ("Sphere", "Tube", "Circle", "Metaball",
// any case); None otherwise.
inline bool quadricNameIs( const char* name, const char* want )
{
	for ( ; *name && *want; ++name, ++want )
		if ( std::tolower( static_cast<unsigned char>( *name ) ) !=
			 std::tolower( static_cast<unsigned char>( *want ) ) )
			return false;
	return !*name && !*want;
}

inline QuadricKind quadricKindFromName( const char* name )
{
	if ( !name )
		return QuadricKind::None;
	if ( quadricNameIs( name, "Sphere" ) )
		return QuadricKind::Sphere;
	if ( quadricNameIs( name, "Tube" ) )
		return QuadricKind::Tube;
	if ( quadricNameIs( name, "Circle" ) )
		return QuadricKind::Circle;
	if ( quadricNameIs( name, "Metaball" ) )
		return QuadricKind::Metaball;
	return QuadricKind::None;
}

// A metaball's field reaches farther than its radius; the RBD Bullet Solver's
// collision shape for a radius 0.5 metaball rests at 0.25, so the radius is
// halved.
constexpr double kMetaballRadiusScale = 0.5;
// A circle is flat; its points are given this much thickness either way along
// its normal so the hull has volume (the Bullet Solver's flat circle rests at
// about 0.02).
constexpr double kCircleHalfThickness = 0.02;
// The same thickness (half) for any flat piece that cannot be hulled.
constexpr float kFlatHalfThickness = static_cast<float>( kCircleHalfThickness );
// True if the Sphere's transform is a uniform scale (with any turn): its three
// columns have the same length and are at right angles, within 0.1%. Then it is
// an exact sphere of that radius, which Box3D has as a shape; otherwise (an
// ellipsoid, a shear) it is sampled and hulled.
inline bool quadricSphereRadius( const double a[9], double& radius )
{
	double col[3][3], len[3];
	for ( int c = 0; c < 3; ++c )
	{
		for ( int r = 0; r < 3; ++r )
			col[c][r] = a[r * 3 + c];
		len[c] = std::sqrt( col[c][0] * col[c][0] + col[c][1] * col[c][1] + col[c][2] * col[c][2] );
	}
	if ( !( len[0] > 1e-9 ) || !std::isfinite( len[0] + len[1] + len[2] ) )
		return false;
	const double tol = 1e-3 * len[0];
	for ( int c = 0; c < 3; ++c )
	{
		if ( std::fabs( len[c] - len[0] ) > tol )
			return false;
		const int d = ( c + 1 ) % 3;
		const double dot = col[c][0] * col[d][0] + col[c][1] * col[d][1] + col[c][2] * col[d][2];
		if ( std::fabs( dot ) > 1e-3 * len[0] * len[0] )
			return false;
	}
	radius = len[0];
	return true;
}

constexpr int kQuadricRings = 12;	 // latitude steps pole to pole
constexpr int kQuadricSegments = 24; // points around

// Appends points (x, y, z, ...) on the primitive's surface, in the space of
// `centre` (the primitive's point): the unit shape through `a` (3x3, p' = a *
// p, row-major), plus `centre`. A Tube's ring at y = 0.5 has radius `taper`
// (GEO_PrimTube::getTaper; the ring at -0.5 has radius 1).
inline void appendQuadricPoints( QuadricKind kind, const double a[9], const double centre[3], std::vector<float>& xyz,
								 double taper = 1.0 )
{
	auto put = [&]( double x, double y, double z ) {
		xyz.push_back( static_cast<float>( centre[0] + a[0] * x + a[1] * y + a[2] * z ) );
		xyz.push_back( static_cast<float>( centre[1] + a[3] * x + a[4] * y + a[5] * z ) );
		xyz.push_back( static_cast<float>( centre[2] + a[6] * x + a[7] * y + a[8] * z ) );
	};
	const double twoPi = 6.283185307179586;
	const double halfTurn = twoPi / 2.0;
	switch ( kind )
	{
		case QuadricKind::Sphere:
		case QuadricKind::Metaball:
		{
			const double s = kind == QuadricKind::Metaball ? kMetaballRadiusScale : 1.0;
			put( 0, s, 0 ); // poles exactly, so a sphere rests on its true radius
			put( 0, -s, 0 );
			for ( int i = 1; i < kQuadricRings; ++i )
			{
				const double phi = halfTurn * i / kQuadricRings;
				const double r = s * std::sin( phi ), y = s * std::cos( phi );
				for ( int j = 0; j < kQuadricSegments; ++j )
				{
					const double th = twoPi * j / kQuadricSegments;
					put( r * std::cos( th ), y, r * std::sin( th ) );
				}
			}
			break;
		}
		case QuadricKind::Tube:
			for ( int cap = 0; cap < 2; ++cap )
				for ( int j = 0; j < kQuadricSegments; ++j )
				{
					const double th = twoPi * j / kQuadricSegments;
					const double r = cap ? taper : 1.0;
					put( r * std::cos( th ), cap ? 0.5 : -0.5, r * std::sin( th ) );
				}
			break;
		case QuadricKind::Circle:
		{
			// Thickness along the transformed normal (the matrix's z column), not the
			// unit z.
			double nx = a[2], ny = a[5], nz = a[8];
			const double len = std::sqrt( nx * nx + ny * ny + nz * nz );
			if ( len > 1e-12 )
			{
				nx *= kCircleHalfThickness / len;
				ny *= kCircleHalfThickness / len;
				nz *= kCircleHalfThickness / len;
			}
			else
				nz = kCircleHalfThickness;
			for ( int side = 0; side < 2; ++side )
				for ( int j = 0; j < kQuadricSegments; ++j )
				{
					const double th = twoPi * j / kQuadricSegments;
					const double sgn = side ? 1.0 : -1.0;
					xyz.push_back(
						static_cast<float>( centre[0] + a[0] * std::cos( th ) + a[1] * std::sin( th ) + sgn * nx ) );
					xyz.push_back(
						static_cast<float>( centre[1] + a[3] * std::cos( th ) + a[4] * std::sin( th ) + sgn * ny ) );
					xyz.push_back(
						static_cast<float>( centre[2] + a[6] * std::cos( th ) + a[7] * std::sin( th ) + sgn * nz ) );
				}
			break;
		}
		case QuadricKind::None:
			return;
	}
	xyz.push_back( static_cast<float>( centre[0] ) );
	xyz.push_back( static_cast<float>( centre[1] ) );
	xyz.push_back( static_cast<float>( centre[2] ) );
}

} // namespace b3dop
