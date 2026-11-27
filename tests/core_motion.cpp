// The drag and speed limits of RBD Bullet Solver's POP-style attributes
// (core/Box3DMotion.h), and the world inertia the spin drag works on.
#include "TestUtil.h"
#include "WorldTestUtil.h"
#include "core/Box3DMotion.h"

#include <cmath>
#include <cstdio>

using namespace b3dop;

namespace
{

float len( const float v[3] )
{
	return std::sqrt( v[0] * v[0] + v[1] * v[1] + v[2] * v[2] );
}

} // namespace

int main()
{
	const float zero[3] = { 0, 0, 0 };

	// ---- implicit drag: the closed forms ----
	{
		// Quadratic: speed s decays as s / (1 + c dt s).
		float v[3] = { 4.0f, 0.0f, 0.0f };
		applyImplicitDrag( 2.0f, 0.5f, zero, 0.1f, 2.0f, v ); // c = 0.5 * 2
		CHECK( near( v[0], 4.0 / ( 1.0 + 1.0 * 0.1 * 4.0 ), 1e-5 ) && v[1] == 0.0f );
		// Linear: exp(-c dt).
		float w[3] = { 3.0f, 4.0f, 0.0f };
		applyImplicitDrag( 1.0f, 0.5f, zero, 0.2f, 1.0f, w );
		CHECK( near( w[0], 3.0 * std::exp( -0.1 ), 1e-5 ) && near( w[1], 4.0 * std::exp( -0.1 ), 1e-5 ) );
		// Between: slower than the quadratic case would be at a low speed, never
		// past rest, and continuous in the exponent.
		float a[3] = { 2.0f, 0.0f, 0.0f }, b[3] = { 2.0f, 0.0f, 0.0f }, c[3] = { 2.0f, 0.0f, 0.0f };
		applyImplicitDrag( 1.999f, 1.0f, zero, 0.05f, 1.0f, a );
		applyImplicitDrag( 2.0f, 1.0f, zero, 0.05f, 1.0f, b );
		applyImplicitDrag( 1.5f, 1.0f, zero, 0.05f, 1.0f, c );
		CHECK( near( a[0], b[0], 1e-3 ) && c[0] > 0.0f && c[0] < 2.0f );
		// Towards a target: the relative velocity decays, the target stays.
		float t[3] = { 5.0f, 0.0f, 0.0f };
		const float target[3] = { 1.0f, 2.0f, 0.0f };
		applyImplicitDrag( 1.0f, 10.0f, target, 100.0f, 1.0f, t ); // long: reaches it
		CHECK( near( t[0], 1.0, 1e-4 ) && near( t[1], 2.0, 1e-4 ) );
		// A strong drag does not overshoot past the target (explicit would).
		float s[3] = { 10.0f, 0.0f, 0.0f };
		applyImplicitDrag( 2.0f, 1e6f, zero, 0.1f, 1.0f, s );
		CHECK( s[0] >= 0.0f && s[0] < 1e-3 );
		// At rest relative to the target: nothing happens.
		float r[3] = { 1.0f, 2.0f, 0.0f };
		applyImplicitDrag( 1.7f, 3.0f, target, 0.1f, 1.0f, r );
		CHECK( near( r[0], 1.0, 1e-6 ) && near( r[1], 2.0, 1e-6 ) );
	}

	// ---- the exponent and the parameters ----
	{
		CHECK( clampDragExp( 0.2f ) == 1.0f && clampDragExp( 5.0f ) == 2.0f && clampDragExp( 1.5f ) == 1.5f );
		CHECK( clampDragExp( std::nanf( "" ) ) == 2.0f );
		MotionParams p;
		CHECK( !p.any() && !p.hasDrag() && !p.hasLimits() );
		p.maxSpin = 0.0f; // a limit of 0 is a limit
		CHECK( p.hasLimits() && p.any() );
		p = MotionParams();
		p.spinResist = 0.1f;
		CHECK( p.hasDrag() && !p.hasLimits() );
	}

	// ---- the clamps ----
	{
		MotionParams p;
		p.maxSpeed = 2.0f;
		float lin[3] = { 6.0f, 8.0f, 0.0f }, ang[3] = { 1.0f, 0.0f, 0.0f };
		CHECK( clampMotion( p, lin, ang ) && near( len( lin ), 2.0, 1e-5 ) && near( lin[0] / lin[1], 0.75, 1e-5 ) &&
			   ang[0] == 1.0f );
		// A minimum raises a slow body, never a resting one.
		p = MotionParams();
		p.minSpeed = 3.0f;
		float slow[3] = { 0.0f, 0.5f, 0.0f }, rest[3] = { 0, 0, 0 };
		CHECK( clampMotion( p, slow, ang ) && near( slow[1], 3.0, 1e-5 ) );
		CHECK( !clampMotion( p, rest, rest ) );
		// Spin limits are separate from speed limits.
		p = MotionParams();
		p.maxSpin = 1.0f;
		float l2[3] = { 9.0f, 0, 0 }, a2[3] = { 0.0f, 3.0f, 4.0f };
		CHECK( clampMotion( p, l2, a2 ) && near( len( a2 ), 1.0, 1e-5 ) && l2[0] == 9.0f );
		// Within the limits: unchanged.
		p = MotionParams();
		p.minSpeed = 1.0f;
		p.maxSpeed = 5.0f;
		float in[3] = { 3.0f, 0, 0 }, spin[3] = { 0, 0, 0 };
		CHECK( !clampMotion( p, in, spin ) && in[0] == 3.0f );
	}

	// ---- the limit on the constraints' change (accelmax / angaccelmax) ----
	{
		MotionParams p;
		CHECK( !p.hasAccelLimits() );
		p.maxAccel = 0.0f;
		CHECK( p.hasAccelLimits() && p.any() && !p.hasLimits() );
		// A landing: falling at 10, stopped by the ground, with gravity's -0.4 in
		// the step. The constraints made +9.6 of change; a limit of 24 per second
		// over 1/24 s lets 1 of it through.
		const float expected[3] = { 0.0f, -10.4f, 0.0f };
		float v[3] = { 0.0f, 0.0f, 0.0f };
		CHECK( clampConstraintChange( 24.0f, 1.0f / 24.0f, expected, v ) && near( v[1], -9.4, 1e-5 ) && v[0] == 0.0f );
		// Within the limit, or no limit: unchanged.
		float small[3] = { 0.0f, -10.0f, 0.0f };
		CHECK( !clampConstraintChange( 24.0f, 1.0f / 24.0f, expected, small ) && small[1] == -10.0f );
		float none[3] = { 0.0f, 0.0f, 0.0f };
		CHECK( !clampConstraintChange( -1.0f, 1.0f / 24.0f, expected, none ) && none[1] == 0.0f );
		// The velocity a step makes with no constraints: plain without damping,
		// and Box3D's substeps one by one with it.
		const float v0[3] = { 1.0f, 2.0f, 0.0f }, g[3] = { 0.0f, -10.0f, 0.0f };
		float free[3];
		undisturbedVelocity( v0, g, 0.0f, 0.1f, 4, free );
		CHECK( near( free[0], 1.0, 1e-6 ) && near( free[1], 1.0, 1e-6 ) );
		float stepped[3] = { v0[0], v0[1], v0[2] };
		for ( int i = 0; i < 4; ++i )
			for ( int k = 0; k < 3; ++k )
				stepped[k] = g[k] * 0.025f + stepped[k] / ( 1.0f + 0.025f * 2.0f );
		undisturbedVelocity( v0, g, 2.0f, 0.1f, 4, free );
		CHECK( near( free[0], stepped[0], 1e-5 ) && near( free[1], stepped[1], 1e-5 ) );
		// The change keeps its direction: shortened along it.
		const float e2[3] = { 0, 0, 0 };
		float diag[3] = { 3.0f, 4.0f, 0.0f };
		CHECK( clampConstraintChange( 1.0f, 1.0f, e2, diag ) && near( len( diag ), 1.0, 1e-5 ) &&
			   near( diag[0] / diag[1], 0.75, 1e-5 ) );
	}

	// ---- the world inertia the spin drag uses ----
	{
		WorldSettings ws;
		ws.gravity[0] = ws.gravity[1] = ws.gravity[2] = 0.0f;
		Box3DWorld w;
		CHECK( w.create( ws ) );
		// A 2 x 1 x 1 box turned 90 degrees about z: its long axis is world y.
		BodySettings b;
		const double p[3] = { 0, 5, 0 };
		const float q[4] = { 0, 0, 0.70710678f, 0.70710678f };
		const BodyHandle h = w.createBody( b, p, q );
		ShapeSpec s;
		s.type = ShapeType::Box;
		s.halfExtents[0] = 1.0f;
		s.halfExtents[1] = s.halfExtents[2] = 0.5f;
		w.createShape( h, b, s );
		float i[6], mass, centre[3], local[3];
		CHECK( w.inertiaWorldByHandle( h, i ) );
		CHECK( w.getMassDataByHandle( h, mass, centre, local ) );
		// About the long axis it is m / 12 (1 + 1), about the others m / 12 (4 +
		// 1).
		CHECK( near( i[1], mass / 6.0, 1e-3 * mass ) && near( i[0], 5.0 * mass / 12.0, 1e-3 * mass ) &&
			   near( i[2], 5.0 * mass / 12.0, 1e-3 * mass ) );
		CHECK( near( i[3], 0.0, 1e-3 * mass ) && near( i[4], 0.0, 1e-3 * mass ) && near( i[5], 0.0, 1e-3 * mass ) );
		CHECK( !w.inertiaWorldByHandle( 0, i ) );
	}

	// ---- the inertia of the box over a hull (Bullet's convex hulls) ----
	{
		float inv[6] = {}, turned[6] = {};
		const float ext[3] = { 2.0f, 1.0f, 1.0f };
		// About x: m (1 + 1) / 12 = 2, about y and z: m (4 + 1) / 12 = 5.
		CHECK( boundsInverseInertia( ext, 12.0f, inv ) );
		CHECK( near( inv[0], 0.5, 1e-6 ) && near( inv[1], 0.2, 1e-6 ) && near( inv[2], 0.2, 1e-6 ) );
		// The rest axes turned 90 degrees about z from the body's: the long axis is
		// the body's y.
		const float q90z[4] = { 0, 0, 0.70710678f, 0.70710678f };
		CHECK( boundsInverseInertiaIn( ext, 12.0f, q90z, turned ) );
		CHECK( near( turned[0], 0.2, 1e-5 ) && near( turned[1], 0.5, 1e-5 ) && near( turned[2], 0.2, 1e-5 ) &&
			   near( turned[3], 0.0, 1e-5 ) );
		// 45 degrees about z: the long axis is between x and y, so xy is
		// (0.5 - 0.2) / 2 in size.
		const float q45z[4] = { 0, 0, 0.38268343f, 0.92387953f };
		CHECK( boundsInverseInertiaIn( ext, 12.0f, q45z, turned ) );
		CHECK( near( std::fabs( turned[3] ), 0.15, 1e-4 ) && near( turned[0], 0.35, 1e-4 ) );
		CHECK( !boundsInverseInertiaIn( ext, 0.0f, nullptr, turned ) );

		WorldSettings ws;
		Box3DWorld w;
		CHECK( w.create( ws ) );
		const auto hullBody = [&]( const float* pts, int n, double y ) {
			BodySettings b;
			const double p[3] = { 0, y, 0 };
			const BodyHandle h = w.createBody( b, p, kQid );
			const HullId id = w.createHull( pts, n );
			CHECK( id != 0 && w.createHullShape( h, b, id ) );
			return h;
		};
		// A tetrahedron: a hull that is not a box.
		const float tetra[12] = { 1, 1, 1, 1, -1, -1, -1, 1, -1, -1, -1, 1 };
		const BodyHandle t = hullBody( tetra, 4, 5 );
		float e[3] = { 0, 0, 0 };
		CHECK( w.boundsInertiaExtentsByHandle( t, nullptr, 0.0f, e ) );
		CHECK( near( e[0], 2.0, 1e-3 ) && near( e[1], 2.0, 1e-3 ) && near( e[2], 2.0, 1e-3 ) );
		CHECK( w.boundsInertiaExtentsByHandle( t, nullptr, 0.08f, e ) );
		CHECK( near( e[0], 2.08, 1e-3 ) );
		// A box hull, upright and turned: exact already.
		const float cube[24] = { -1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1 };
		CHECK( !w.boundsInertiaExtentsByHandle( hullBody( cube, 8, 10 ), nullptr, 0.0f, e ) );
		const float c = 0.70710678f; // the cube turned 45 degrees about z
		const float turnedCube[24] = { -c, -c, -1, c, -c, -1, -c, c, -1, c, c, -1,
									   -c, -c, 1,  c, -c, 1,  -c, c, 1,	 c, c, 1 };
		CHECK( !w.boundsInertiaExtentsByHandle( hullBody( turnedCube, 8, 15 ), nullptr, 0.0f, e ) );
		// No shape, one sphere: not. Two spheres 4 apart: the box over them.
		BodySettings b;
		const double p0[3] = { 0, 20, 0 };
		const BodyHandle two = w.createBody( b, p0, kQid );
		CHECK( !w.boundsInertiaExtentsByHandle( two, nullptr, 0.0f, e ) );
		ShapeSpec ball;
		ball.type = ShapeType::Sphere;
		ball.radius = 0.5f;
		CHECK( w.createShape( two, b, ball ) );
		CHECK( !w.boundsInertiaExtentsByHandle( two, nullptr, 0.0f, e ) );
		ball.center[0] = 4.0f;
		CHECK( w.createShape( two, b, ball ) );
		CHECK( w.boundsInertiaExtentsByHandle( two, nullptr, 0.0f, e ) );
		CHECK( near( e[0], 5.0, 1e-3 ) && near( e[1], 1.0, 1e-3 ) );
		// The rest frame turned 90 degrees about z: the long side is y.
		CHECK( w.boundsInertiaExtentsByHandle( two, q90z, 0.0f, e ) );
		CHECK( near( e[0], 1.0, 1e-3 ) && near( e[1], 5.0, 1e-3 ) );
	}

	std::printf( "OK\n" );
	return fails ? 1 : 0;
}
