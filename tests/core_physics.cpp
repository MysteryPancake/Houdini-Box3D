// Exercises the real Box3D calls in the wrapper: bodies, shapes, hulls, forces,
// filtering, sleep and world settings.
//
// This file holds the case list (kTests below), main and the basic body cases.
// The rest are in physics_shapes.cpp and physics_bodies.cpp, declared in
// PhysicsCases.h. Run with no arguments to run all of them; pass case names to
// run only those, or --list to print the names. CMake registers each case as
// its own CTest test (b3dop_core_physics.<name>) by reading that list, so a
// failure says which area broke.
#include "KeyedWorld.h"
#include "PhysicsCases.h"
#include "TestUtil.h"
#include "core/Box3DShapes.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <vector>

// Basics: a box and a sphere fall onto a static ground box and come to rest;
// transforms and velocities round-trip through the keyed and handle APIs;
// removeBody; destroying the world.
void testBodyBasics()
{
	b3dop::KeyedWorld w;
	CHECK( w.create( b3dop::WorldSettings() ) );

	const double groundPos[3] = { 0, -0.5, 0 };
	const double boxPos[3] = { 0, 3.0, 0 };

	b3dop::BodySettings groundBody;
	groundBody.type = b3dop::BodySettings::Type::Static;
	b3dop::BodySettings dynamicBox; // dynamic

	CHECK( w.addBody( 1, groundBody, groundPos, qIdentity ) );
	const float groundHalf[3] = { 10, 0.5f, 10 };
	CHECK( w.addBoxShape( 1, groundHalf ) );
	CHECK( w.addBody( 2, dynamicBox, boxPos, qIdentity ) );
	const float boxHalf[3] = { 0.5f, 0.5f, 0.5f };
	CHECK( w.addBoxShape( 2, boxHalf ) );
	CHECK( w.addBody( 3, dynamicBox, boxPos, qIdentity ) );
	CHECK( w.addSphereShape( 3, 0.5f ) );
	CHECK( !w.addBody( 2, dynamicBox, boxPos, qIdentity ) ); // duplicate id rejected
	CHECK( w.bodyCount() == 3 );

	for ( int i = 0; i < 240; ++i )
		w.step( 1.0f / 60.0f );

	double outP[3];
	float outQ[4], outLin[3], outAng[3];
	CHECK( w.getTransform( 2, outP, outQ ) );
	CHECK( w.getVelocity( 2, outLin, outAng ) );
	std::printf( "box    rest y=%.4f  |v|=%.5f\n", outP[1], std::fabs( outLin[1] ) );
	CHECK( std::fabs( outP[1] - 0.5 ) < 0.05 ); // resting on ground top (y=0) + half extent
	CHECK( std::fabs( outLin[1] ) < 0.1 );
	CHECK( w.getTransform( 3, outP, outQ ) );
	std::printf( "sphere rest y=%.4f\n", outP[1] );
	CHECK( std::fabs( outP[1] - 0.5 ) < 0.05 );

	// push state in and read it back (what the solver does for external edits)
	const double newPos[3] = { 5, 7, -2 };
	CHECK( w.setTransform( 2, newPos, qIdentity ) );
	const float setLin[3] = { 1, 2, 3 }, setAng[3] = { 0.5f, 0, 0 };
	CHECK( w.setVelocity( 2, setLin, setAng ) );
	CHECK( w.getTransform( 2, outP, outQ ) );
	CHECK( std::fabs( outP[0] - 5 ) < 1e-5 && std::fabs( outP[1] - 7 ) < 1e-5 && std::fabs( outP[2] + 2 ) < 1e-5 );
	CHECK( w.getVelocity( 2, outLin, outAng ) );
	CHECK( std::fabs( outLin[0] - 1 ) < 1e-5 && std::fabs( outLin[2] - 3 ) < 1e-5 &&
		   std::fabs( outAng[0] - 0.5f ) < 1e-5 );

	// handle access gives the same answers as keyed access
	{
		const b3dop::BodyHandle h = w.bodyHandle( 2 );
		CHECK( h != 0 );
		CHECK( w.bodyHandle( 9999 ) == 0 );
		double hp[3];
		float hq[4] = {}, hlin[3] = {}, hang[3] = {};
		CHECK( w.getTransformByHandle( h, hp, hq ) );
		CHECK( w.getTransform( 2, outP, outQ ) );
		CHECK( hp[0] == outP[0] && hp[1] == outP[1] && hp[2] == outP[2] && hq[3] == outQ[3] );
		CHECK( w.getVelocityByHandle( h, hlin, hang ) );
		CHECK( hlin[0] == outLin[0] && hlin[2] == outLin[2] && hang[0] == outAng[0] );
		const double np2[3] = { -4, 8, 1 };
		const float v2[3] = { 0, 1, 0 }, a2[3] = { 0, 0, 2 };
		CHECK( w.setTransformByHandle( h, np2, qIdentity ) );
		CHECK( w.setVelocityByHandle( h, v2, a2 ) );
		CHECK( w.getTransform( 2, outP, outQ ) );
		CHECK( std::fabs( outP[0] + 4 ) < 1e-5 && std::fabs( outP[1] - 8 ) < 1e-5 );
		CHECK( w.getVelocity( 2, outLin, outAng ) );
		CHECK( std::fabs( outLin[1] - 1 ) < 1e-5 && std::fabs( outAng[2] - 2 ) < 1e-5 );
		CHECK( !w.getTransformByHandle( 0, hp, hq ) );
		// distinct bodies get distinct handles
		CHECK( w.bodyHandle( 3 ) != h );
	}

	w.removeBody( 2 );
	CHECK( !w.hasBody( 2 ) && w.bodyCount() == 2 );
	CHECK( !w.getTransform( 2, outP, outQ ) );
	w.destroy();
	CHECK( !w.isValid() );
}

// Point-attribute mapping (Box3DShapes.h) + addShape / setBodyType.
void testPointMapping()
{
	double outP[3] = {};
	float outQ[4] = {}, outLin[3] = {}, outAng[3] = {};

	using namespace b3dop;

	ShapeType t = ShapeType::Box;
	CHECK( parseShapeType( "sphere", t ) && t == ShapeType::Sphere );
	CHECK( parseShapeType( "box", t ) && t == ShapeType::Box );
	t = ShapeType::Sphere;
	CHECK( !parseShapeType( "Sphere", t ) && !parseShapeType( "BOX", t ) && t == ShapeType::Sphere ); // case-sensitive
	t = ShapeType::Sphere;
	CHECK( !parseShapeType( "proxy", t ) && t == ShapeType::Sphere ); // unknown: unchanged
	CHECK( !parseShapeType( "", t ) && !parseShapeType( nullptr, t ) );

	const float one[3] = { 1, 1, 1 }, aniso[3] = { 1, 2, 4 };
	const ShapeSpec b1 = makeShapeSpec( ShapeType::Box, 2.0f, one );
	CHECK( std::fabs( b1.halfExtents[0] - 1.0f ) < 1e-6f ); // unit box * pscale 2
	const ShapeSpec b2 = makeShapeSpec( ShapeType::Box, 1.0f, aniso );
	CHECK( std::fabs( b2.halfExtents[2] - 2.0f ) < 1e-6f );
	const ShapeSpec s1 = makeShapeSpec( ShapeType::Sphere, 1.0f, aniso );
	CHECK( std::fabs( s1.radius - 2.0f ) < 1e-6f ); // largest component
	const float zero[3] = { 0, 0, 0 };
	CHECK( makeShapeSpec( ShapeType::Box, 1.0f, zero ).halfExtents[0] > 0.0f ); // clamped

	CHECK( std::fabs( shapeVolume( b2 ) - 8.0f ) < 1e-4f ); // 1 x 2 x 4
	CHECK( makeBodyKey( 1, 5 ) != makeBodyKey( 2, 5 ) && makeBodyKey( 1, 5 ) != makeBodyKey( 1, 6 ) );

	// Two "points" with different mass, dropped from the same height, plus
	// a static ground point: mass -> density -> real Box3D mass.
	KeyedWorld pw;
	CHECK( pw.create( WorldSettings() ) );
	BodySettings stat;
	stat.type = BodySettings::Type::Static;
	const double gp[3] = { 0, -0.5, 0 };
	const float gsc[3] = { 20, 1, 20 };
	CHECK( pw.addBody( makeBodyKey( 1, 0 ), stat, gp, qIdentity ) );
	CHECK( pw.addShape( makeBodyKey( 1, 0 ), makeShapeSpec( ShapeType::Box, 1.0f, gsc ) ) );

	BodySettings dyn;
	const ShapeSpec heavy = makeShapeSpec( ShapeType::Box, 1.0f, one );
	dyn.props.density = 50.0f / shapeVolume( heavy );
	const double hp[3] = { -2, 3, 0 }, spherePos[3] = { 2, 3, 0 };
	CHECK( pw.addBody( makeBodyKey( 1, 1 ), dyn, hp, qIdentity ) );
	CHECK( pw.addShape( makeBodyKey( 1, 1 ), heavy ) );
	CHECK( pw.addBody( makeBodyKey( 1, 2 ), dyn, spherePos, qIdentity ) );
	CHECK( pw.addShape( makeBodyKey( 1, 2 ), makeShapeSpec( ShapeType::Sphere, 1.0f, one ) ) );
	const float v0[3] = { 1, 0, 0 }, w0[3] = { 0, 0, 0 };
	CHECK( pw.setVelocity( makeBodyKey( 1, 1 ), v0, w0 ) ); // initial `v`

	pw.step( 1.0f / 60.0f );
	CHECK( pw.getVelocity( makeBodyKey( 1, 1 ), outLin, outAng ) );
	CHECK( outLin[0] > 0.9f ); // v attribute honoured

	for ( int i = 0; i < 300; ++i )
		pw.step( 1.0f / 60.0f );
	CHECK( pw.getTransform( makeBodyKey( 1, 1 ), outP, outQ ) );
	CHECK( std::fabs( outP[1] - 0.5 ) < 0.05 ); // rests on ground
	CHECK( pw.getTransform( makeBodyKey( 1, 2 ), outP, outQ ) );
	CHECK( std::fabs( outP[1] - 0.5 ) < 0.05 );

	// active 1 -> 0: becomes static and stops moving.
	BodySettings::Type ty;
	CHECK( pw.getBodyType( makeBodyKey( 1, 2 ), ty ) && ty == BodySettings::Type::Dynamic );
	CHECK( pw.setBodyType( makeBodyKey( 1, 2 ), BodySettings::Type::Static ) );
	CHECK( pw.getBodyType( makeBodyKey( 1, 2 ), ty ) && ty == BodySettings::Type::Static );
	const double lift[3] = { 2, 5, 0 };
	CHECK( pw.setTransform( makeBodyKey( 1, 2 ), lift, qIdentity ) );
	pw.step( 1.0f / 60.0f );
	CHECK( pw.getTransform( makeBodyKey( 1, 2 ), outP, outQ ) );
	CHECK( std::fabs( outP[1] - 5.0 ) < 1e-4 ); // static: gravity ignored

	// Point removed -> body removed, others untouched.
	pw.removeBody( makeBodyKey( 1, 2 ) );
	CHECK( !pw.hasBody( makeBodyKey( 1, 2 ) ) && pw.hasBody( makeBodyKey( 1, 1 ) ) );

	// a capsule dropped upright on the ground (its top face is y = 0)
	// comes to rest at half its height.
	const float stand[3] = { 1, 3, 1 };
	const double capP[3] = { -6, 4, 0 };
	CHECK( pw.addBody( makeBodyKey( 1, 10 ), dyn, capP, qIdentity ) );
	CHECK( pw.addShape( makeBodyKey( 1, 10 ), makeShapeSpec( ShapeType::Capsule, 1.0f, stand ) ) );
	for ( int i = 0; i < 300; ++i )
		pw.step( 1.0f / 60.0f );
	CHECK( pw.getTransform( makeBodyKey( 1, 10 ), outP, outQ ) );
	CHECK( std::fabs( outP[1] - 1.5 ) < 0.06 ); // capsule: 3 tall
	CHECK( pw.bodyMass( makeBodyKey( 1, 10 ) ) > 0.0f );

	// a cylinder (radius 0.5, 3 tall) dropped upright rests at half its height
	{
		const double cylP[3] = { -7.5, 4, 0 };
		CHECK( pw.addBody( makeBodyKey( 1, 11 ), dyn, cylP, qIdentity ) );
		CHECK( pw.addShape( makeBodyKey( 1, 11 ), makeShapeSpec( ShapeType::Cylinder, 1.0f, stand ) ) );
		for ( int i = 0; i < 300; ++i )
			pw.step( 1.0f / 60.0f );
		CHECK( pw.getTransform( makeBodyKey( 1, 11 ), outP, outQ ) );
		CHECK( std::fabs( outP[1] - 1.5 ) < 0.06 );
		CHECK( pw.bodyMass( makeBodyKey( 1, 11 ) ) > 0.0f );
	}

	// a capsule placed off-centre via ShapeSpec::center moves the shape, not
	// the body
	{
		KeyedWorld w3;
		CHECK( w3.create( WorldSettings() ) );
		const double o[3] = { 0, 0, 0 };
		CHECK( w3.addBody( makeBodyKey( 1, 1 ), dyn, o, qIdentity ) );
		ShapeSpec sp = makeShapeSpec( ShapeType::Capsule, 1.0f, stand );
		sp.center[0] = 5.0f;
		CHECK( w3.addShape( makeBodyKey( 1, 1 ), sp ) );
		CHECK( w3.shapeCount( makeBodyKey( 1, 1 ) ) == 1 );
	}

	std::printf( "point-mapping checks done\n" );
}

// Move events: only awake bodies, with the transform, user data and sleep
// flag.
void testMoveEvents()
{
	b3dop::KeyedWorld ew;
	b3dop::WorldSettings ws;
	CHECK( ew.create( ws ) );
	const float q0[4] = { 0, 0, 0, 1 };
	b3dop::BodySettings st;
	st.type = b3dop::BodySettings::Type::Static;
	const double gp[3] = { 0, -0.5, 0 };
	const float gh[3] = { 20, 0.5f, 20 };
	CHECK( ew.addBody( 1, st, gp, q0 ) );
	CHECK( ew.addBoxShape( 1, gh ) );
	const double bp[3] = { 0, 0.5, 0 };
	CHECK( ew.addBody( 2, b3dop::BodySettings(), bp, q0 ) );
	const float bh[3] = { 0.5f, 0.5f, 0.5f };
	CHECK( ew.addBoxShape( 2, bh ) );
	int tag = 7;
	ew.setBodyUserData( ew.bodyHandle( 2 ), &tag );

	std::vector<b3dop::BodyMove> all;
	CHECK( ew.getMoveEvents( all ) == 0 && all.empty() ); // nothing stepped yet
	ew.step( 1.0f / 60.0f );
	// the static ground never moves
	CHECK( ew.getMoveEvents( all ) == 1 && all.size() == 1 );
	const b3dop::BodyMove mv = all[0];
	CHECK( mv.userData == &tag && mv.handle == ew.bodyHandle( 2 ) );
	double p[3];
	float q[4];
	CHECK( ew.getTransformByHandle( mv.handle, p, q ) );
	CHECK( mv.pos[0] == p[0] && mv.pos[1] == p[1] && mv.pos[2] == p[2] );
	CHECK( mv.rot[0] == q[0] && mv.rot[3] == q[3] );

	// releasing the build settings keeps type queries and changes working
	// (Box3D answers)
	b3dop::BodySettings::Type ty;
	CHECK( ew.getBodyType( 2, ty ) && ty == b3dop::BodySettings::Type::Dynamic );
	CHECK( ew.getBodyType( 2, ty ) && ty == b3dop::BodySettings::Type::Dynamic );
	CHECK( ew.setBodyType( 2, b3dop::BodySettings::Type::Kinematic ) );
	CHECK( ew.getBodyType( 2, ty ) && ty == b3dop::BodySettings::Type::Kinematic );
	CHECK( ew.setBodyType( 2, b3dop::BodySettings::Type::Dynamic ) );
	CHECK( !ew.getBodyType( 99, ty ) );

	// resting on the ground, it falls asleep: one last event with the flag,
	// then none
	bool sawSleep = false;
	int lastCount = 1;
	for ( int i = 0; i < 600 && lastCount > 0; ++i )
	{
		ew.step( 1.0f / 60.0f );
		lastCount = ew.getMoveEvents( all );
		if ( lastCount && all[0].fellAsleep )
			sawSleep = true;
	}
	CHECK( sawSleep );
	CHECK( ew.getMoveEvents( all ) == 0 );
	// a transform set by the caller is not a move event
	const double np[3] = { 5, 0.5, 0 };
	CHECK( ew.setTransformByHandle( ew.bodyHandle( 2 ), np, q0 ) );
	CHECK( ew.getMoveEvents( all ) == 0 );
	std::printf( "move-event checks done\n" );
}

// sanitizeQuat / storedPosition (the write-back's pose helpers).
void testStoredPose()
{
	// sanitizeQuat / storedPosition: the write-back predicts what the geometry
	// holds with these instead of reading it back, so their behaviour is pinned
	// here.
	float z[4] = { 0, 0, 0, 0 };
	b3dop::sanitizeQuat( z );
	CHECK( z[0] == 0 && z[1] == 0 && z[2] == 0 && z[3] == 1 ); // zero -> identity
	float nan4[4] = { NAN, 0, 0, 1 };
	b3dop::sanitizeQuat( nan4 );
	CHECK( nan4[3] == 1 && nan4[0] == 0 ); // non-finite -> identity
	float off[4] = { 0.1f, 0.2f, 0.3f, 1.7f };
	b3dop::sanitizeQuat( off );
	double l2 = 0;
	for ( float o : off )
		l2 += static_cast<double>( o ) * o;
	CHECK( std::fabs( l2 - 1.0 ) < 1e-6 ); // normalised
	float again[4] = { off[0], off[1], off[2], off[3] };
	b3dop::sanitizeQuat( again );
	double drift = 0;
	for ( int i = 0; i < 4; ++i )
		drift += std::fabs( static_cast<double>( again[i] ) - off[i] );
	CHECK( drift < 1e-6 ); // idempotent to float noise

	const double in[3] = { 0.1, -12345.678901234, 1e-9 };
	double out[3];
	CHECK( b3dop::storedPosition( in, out ) );
	for ( int i = 0; i < 3; ++i )
		CHECK( out[i] == static_cast<double>( static_cast<float>( in[i] ) ) );
	double out2[3];
	CHECK( b3dop::storedPosition( out, out2 ) ); // idempotent: no drift on re-store
	for ( int i = 0; i < 3; ++i )
		CHECK( out2[i] == out[i] );
	// volatile: decided at run time, so MSVC doesn't flag the float overflow
	// the check is about (C4756).
	volatile double hugeX = 1e300;
	const double huge[3] = { hugeX, 0, 0 };
	CHECK( !b3dop::storedPosition( huge, out ) ); // overflows float -> not finite
	std::printf( "stored-pose checks done\n" );
}

// Handle-based building: createBody / createShape / createHullShape /
// applyMassFromShapes.
void testHandleBuild()
{
	{
		using namespace b3dop;

		// parseShapeType: exact lowercase names, `out` untouched on a miss
		{
			const char* names[] = { "box",	"sphere",	"convexhull", "concave", "compound", "capsule",
									"none", "cylinder", "",			  nullptr,	 "Box" };
			const bool ok[] = { true, true, true, true, true, true, true, true, false, false, false };
			for ( size_t i = 0; i < sizeof( names ) / sizeof( names[0] ); ++i )
			{
				ShapeType a = ShapeType::Capsule; // sentinel: unchanged on failure
				CHECK( parseShapeType( names[i], a ) == ok[i] );
				if ( !ok[i] )
					CHECK( a == ShapeType::Capsule );
			}
		}
	}

	{
		using namespace b3dop;
		KeyedWorld hw;
		CHECK( hw.create( WorldSettings() ) );

		BodySettings stat;
		stat.type = BodySettings::Type::Static;
		const double gp[3] = { 0, -0.5, 0 };
		const float gh[3] = { 30, 0.5f, 30 };
		ShapeSpec groundBox;
		groundBox.type = ShapeType::Box;
		groundBox.halfExtents[0] = gh[0];
		groundBox.halfExtents[1] = gh[1];
		groundBox.halfExtents[2] = gh[2];
		const BodyHandle ground = hw.createBody( stat, gp, qIdentity );
		CHECK( ground != 0 && hw.createShape( ground, stat, groundBox ) );
		CHECK( hw.bodyCount() == 1 );

		// user data comes back in the move events (the solver maps an event to its
		// mirror entry with it)
		int tag = 7;
		BodySettings dyn;
		dyn.props.density = 2.0f;
		const double dp[3] = { 0, 3, 0 };
		const BodyHandle falling = hw.createBody( dyn, dp, qIdentity, &tag );
		ShapeSpec cubeSpec;
		cubeSpec.type = ShapeType::Box;
		cubeSpec.halfExtents[0] = cubeSpec.halfExtents[1] = cubeSpec.halfExtents[2] = 0.5f;
		CHECK( falling != 0 && hw.createShape( falling, dyn, cubeSpec ) );
		CHECK( hw.bodyCount() == 2 );
		const float lin[3] = { 1, 0, 0 }, ang[3] = { 0, 0, 0 };
		CHECK( hw.setVelocityByHandle( falling, lin, ang ) );
		hw.step( 1.0f / 60.0f );
		bool sawTag = false;
		std::vector<BodyMove> moves;
		hw.getMoveEvents( moves );
		for ( const BodyMove& mv : moves )
			if ( mv.handle == falling && mv.userData == &tag )
				sawTag = true;
		CHECK( sawTag );

		// a handle-made body is invisible to the keyed API (no maps are filled)
		CHECK( !hw.hasBody( 1 ) && hw.bodyHandle( 1 ) == 0 );

		// Mass is the same whether it is recomputed per shape (keyed API) or
		// applied once (handle API, updateMass = false): an L of two cubes at
		// density 5, then three pieces.
		auto cube = []( float cx, float cy, float cz, float h ) {
			std::vector<float> v;
			for ( int sx = -1; sx <= 1; sx += 2 )
				for ( int sy = -1; sy <= 1; sy += 2 )
					for ( int sz = -1; sz <= 1; sz += 2 )
					{
						v.push_back( cx + static_cast<float>( sx ) * h );
						v.push_back( cy + static_cast<float>( sy ) * h );
						v.push_back( cz + static_cast<float>( sz ) * h );
					}
			return v;
		};
		const std::vector<float> pA = cube( 0, 0, 0, 0.5f ), pB = cube( 1, 0, 0, 0.5f ), pC = cube( 0, 1, 0, 0.5f );
		float vA = 0, vB = 0, vC = 0;
		const HullId hA = hw.createHull( pA.data(), 8, &vA );
		const HullId hB = hw.createHull( pB.data(), 8, &vB );
		const HullId hC = hw.createHull( pC.data(), 8, &vC );
		CHECK( hA && hB && hC );
		BodySettings dense;
		dense.props.density = 5.0f;
		const double lp[3] = { 6, 3, 0 };

		CHECK( hw.addBody( 500, dense, lp, qIdentity ) && hw.addHullShape( 500, hA ) && hw.addHullShape( 500, hB ) &&
			   hw.addHullShape( 500, hC ) );
		const BodyHandle once = hw.createBody( dense, lp, qIdentity );
		CHECK( once != 0 );
		CHECK( hw.createHullShape( once, dense, hA, false ) && hw.createHullShape( once, dense, hB, false ) &&
			   hw.createHullShape( once, dense, hC, false ) );
		hw.applyMassFromShapes( once );
		CHECK( std::fabs( hw.bodyMass( 500 ) - 5.0f * ( vA + vB + vC ) ) < 1e-3f );

		// run both for a while: identical behaviour (they sit apart, on the ground)
		const double lp2[3] = { -6, 3, 0 };
		const BodyHandle once2 = hw.createBody( dense, lp2, qIdentity ); // same body, other place
		CHECK( hw.createHullShape( once2, dense, hA, false ) && hw.createHullShape( once2, dense, hB, false ) &&
			   hw.createHullShape( once2, dense, hC, false ) );
		hw.applyMassFromShapes( once2 );
		const double lp3[3] = { -6, 3, 20 };
		CHECK( hw.addBody( 501, dense, lp3, qIdentity ) && hw.addHullShape( 501, hA ) && hw.addHullShape( 501, hB ) &&
			   hw.addHullShape( 501, hC ) );
		for ( int i = 0; i < 240; ++i )
			hw.step( 1.0f / 60.0f );
		double p1[3], p2[3];
		float q1[4], q2[4];
		CHECK( hw.getTransformByHandle( once2, p1, q1 ) && hw.getTransform( 501, p2, q2 ) );
		CHECK( std::fabs( p1[1] - p2[1] ) < 1e-3 ); // same resting height
		for ( int i = 0; i < 4; ++i )
			CHECK( std::fabs( q1[i] - q2[i] ) < 1e-3f );

		// setTypeByHandle: a dynamic body made static stops falling
		const double fp[3] = { 20, 5, 0 };
		const BodyHandle hover = hw.createBody( dyn, fp, qIdentity );
		CHECK( hover != 0 && hw.createShape( hover, dyn, cubeSpec ) );
		CHECK( hw.setTypeByHandle( hover, BodySettings::Type::Static ) );
		for ( int i = 0; i < 30; ++i )
			hw.step( 1.0f / 60.0f );
		double hp[3];
		float hq[4];
		CHECK( hw.getTransformByHandle( hover, hp, hq ) && std::fabs( hp[1] - 5.0 ) < 1e-6 );

		// destroyBody releases the hulls the body used; the count drops by one per
		// body
		const int before = hw.bodyCount();
		hw.destroyBody( once );
		hw.destroyBody( once2 );
		CHECK( hw.bodyCount() == before - 2 );
		hw.removeBody( 500 );
		hw.removeBody( 501 );
		hw.pruneHulls();
		CHECK( hw.hullCacheSize() == 0 );

		// destroying a body twice (or handle 0) is harmless
		hw.destroyBody( once );
		hw.destroyBody( 0 );
		CHECK( hw.bodyCount() == before - 4 );
		// Shape outlines (the solver's guide geometry): a box hull and an offset
		// sphere on one body come back in body space.
		{
			BodySettings dyn2;
			const double at[3] = { 5, 5, 5 };
			const BodyHandle two = hw.createBody( dyn2, at, qIdentity );
			ShapeSpec box;
			box.type = ShapeType::Box;
			box.halfExtents[0] = 0.5f;
			box.halfExtents[1] = 0.25f;
			box.halfExtents[2] = 1.0f;
			ShapeSpec ball;
			ball.type = ShapeType::Sphere;
			ball.radius = 0.3f;
			ball.center[1] = 2.0f;
			CHECK( two != 0 && hw.createShape( two, dyn2, box ) && hw.createShape( two, dyn2, ball ) );
			std::vector<Box3DWorld::ShapeOutline> outlines;
			CHECK( hw.bodyShapeOutlines( two, outlines ) && outlines.size() == 2 );
			int hulls = 0, spheres = 0;
			for ( const Box3DWorld::ShapeOutline& o : outlines )
			{
				if ( o.kind == Box3DWorld::ShapeOutline::Hull )
				{
					++hulls;
					CHECK( o.verts.size() == 24 ); // 8 corners
					float maxZ = 0.0f;
					for ( size_t k = 2; k < o.verts.size(); k += 3 )
						maxZ = std::max( maxZ, std::fabs( o.verts[k] ) );
					CHECK( std::fabs( maxZ - 1.0f ) < 1e-5f ); // body space, not world
					int faces = 0;
					for ( size_t k = 0; k < o.loops.size(); k += 1 + static_cast<size_t>( o.loops[k] ) )
					{
						CHECK( o.loops[k] == 4 );
						++faces;
					}
					CHECK( faces == 6 );
				}
				else if ( o.kind == Box3DWorld::ShapeOutline::Sphere )
				{
					++spheres;
					CHECK( std::fabs( o.radius - 0.3f ) < 1e-6f && std::fabs( o.c1[1] - 2.0f ) < 1e-6f );
				}
			}
			CHECK( hulls == 1 && spheres == 1 );
			std::vector<Box3DWorld::ShapeOutline> none;
			CHECK( !hw.bodyShapeOutlines( 0, none ) && none.empty() );
			hw.destroyBody( two );
		}
		std::printf( "handle-build checks done\n" );
	}
}

// Mid-simulation changes (what the solver does when an attribute changes).
void testMidSimChanges()
{
	using namespace b3dop;
	KeyedWorld dw;
	CHECK( dw.create( WorldSettings() ) );
	const float qid[4] = { 0, 0, 0, 1 };
	BodySettings stat;
	stat.type = BodySettings::Type::Static;
	const double gp[3] = { 0, -0.5, 0 };
	const BodyHandle ground = dw.createBody( stat, gp, qid );
	ShapeSpec groundSpec;
	groundSpec.halfExtents[0] = groundSpec.halfExtents[2] = 100.0f;
	groundSpec.halfExtents[1] = 0.5f;
	CHECK( ground && dw.createShape( ground, stat, groundSpec ) );

	ShapeSpec cube; // unit cube, volume 1
	BodySettings dyn;
	const double p0[3] = { 0, 0.5, 0 };
	const BodyHandle b = dw.createBody( dyn, p0, qid );
	CHECK( b && dw.createShape( b, dyn, cube ) );
	CHECK( std::fabs( dw.massByHandle( b ) - 1.0f ) < 1e-4f );

	// friction / restitution reach the shapes
	dw.setMaterialByHandle( b, 0.25f, 0.75f );
	float f = 0, r = 0;
	CHECK( dw.getMaterialByHandle( b, f, r ) && f == 0.25f && r == 0.75f );

	// damping, gravity scale
	dw.setDampingByHandle( b, 0.5f, 2.0f );
	float ld = 0, ad = 0;
	CHECK( dw.getDampingByHandle( b, ld, ad ) && ld == 0.5f && ad == 2.0f );
	dw.setGravityScaleByHandle( b, -1.0f );
	CHECK( dw.gravityScaleByHandle( b ) == -1.0f );

	// mass via density (volume 1 -> mass = density); inertia follows the shape
	dw.applyMassByHandle( b, 7.0f, false, 1.0f, true );
	CHECK( std::fabs( dw.massByHandle( b ) - 7.0f ) < 1e-4f );

	// gravity scale -1 floats the body up
	for ( int i = 0; i < 30; ++i )
		dw.step( 1.0f / 60.0f );
	double p[3];
	float q[4];
	CHECK( dw.getTransformByHandle( b, p, q ) && p[1] > 0.6 );

	// sleep permission: a body that may not sleep is awake and stays so
	dw.setGravityScaleByHandle( b, 0.0f );
	const float zero[3] = { 0, 0, 0 };
	dw.setVelocityByHandle( b, zero, zero );
	dw.setSleepEnabledByHandle( b, false );
	CHECK( !dw.isSleepEnabledByHandle( b ) );
	for ( int i = 0; i < 200; ++i )
		dw.step( 1.0f / 60.0f );
	CHECK( dw.isAwakeByHandle( b ) );
	dw.setSleepEnabledByHandle( b, true );
	CHECK( dw.isSleepEnabledByHandle( b ) );
	for ( int i = 0; i < 200; ++i )
		dw.step( 1.0f / 60.0f );
	CHECK( !dw.isAwakeByHandle( b ) ); // still body, allowed to sleep: asleep

	// disabled: does not move, keeps its shapes; enabled again: moves
	dw.setGravityScaleByHandle( b, 1.0f );
	dw.wakeByHandle( b );
	CHECK( dw.isAwakeByHandle( b ) );
	dw.setEnabledByHandle( b, false );
	CHECK( !dw.isEnabledByHandle( b ) );
	double before[3];
	CHECK( dw.getTransformByHandle( b, before, q ) );
	for ( int i = 0; i < 10; ++i )
		dw.step( 1.0f / 60.0f );
	CHECK( dw.getTransformByHandle( b, p, q ) && p[1] == before[1] );
	CHECK( dw.shapeCountByHandle( b ) == 1 );
	dw.setEnabledByHandle( b, true );
	CHECK( dw.isEnabledByHandle( b ) );
	for ( int i = 0; i < 10; ++i )
		dw.step( 1.0f / 60.0f );
	CHECK( dw.getTransformByHandle( b, p, q ) && p[1] < before[1] );

	// rebuild: drop the shapes, add a bigger cube; mass recomputed from the new
	// shape
	float pts[24];
	int k = 0;
	for ( int sx = -1; sx <= 1; sx += 2 )
		for ( int sy = -1; sy <= 1; sy += 2 )
			for ( int sz = -1; sz <= 1; sz += 2 )
			{
				pts[k++] = static_cast<float>( sx );
				pts[k++] = static_cast<float>( sy );
				pts[k++] = static_cast<float>( sz );
			}
	float hv = 0;
	const HullId big = dw.createHull( pts, 8, &hv ); // 2x2x2, volume 8
	CHECK( big && std::fabs( hv - 8.0f ) < 1e-3f );
	dw.destroyShapesByHandle( b );
	CHECK( dw.shapeCountByHandle( b ) == 0 );
	BodySettings dense = dyn;
	dense.props.density = 7.0f / 8.0f; // keep mass 7
	CHECK( dw.createHullShape( b, dense, big, false ) );
	dw.applyMassFromShapes( b );
	CHECK( dw.shapeCountByHandle( b ) == 1 );
	CHECK( std::fabs( dw.massByHandle( b ) - 7.0f ) < 1e-3f );
	// rebuilding again releases the hull it used: prune empties the cache
	dw.destroyShapesByHandle( b );
	CHECK( dw.createShape( b, dyn, cube ) );
	dw.pruneHulls();
	CHECK( dw.hullCacheSize() == 0 );
	// and the body comes to rest on the ground at its new size
	for ( int i = 0; i < 240; ++i )
		dw.step( 1.0f / 60.0f );
	CHECK( dw.getTransformByHandle( b, p, q ) && std::fabs( p[1] - 0.5 ) < 0.05 );

	// handle 0 is ignored everywhere
	dw.setMaterialByHandle( 0, 1, 1 );
	dw.setDampingByHandle( 0, 1, 1 );
	dw.setGravityScaleByHandle( 0, 1 );
	dw.setSleepEnabledByHandle( 0, true );
	dw.applyMassByHandle( 0, 1, false, 1, true );
	dw.setEnabledByHandle( 0, false );
	dw.destroyShapesByHandle( 0 );
	dw.wakeByHandle( 0 );
	CHECK( !dw.isEnabledByHandle( 0 ) && dw.massByHandle( 0 ) == 0.0f && dw.shapeCountByHandle( 0 ) == 0 );

	// the world's sleep switch wins: with sleep off in the world a body never
	// sleeps
	KeyedWorld nw;
	WorldSettings noSleep;
	noSleep.enableSleep = false;
	CHECK( nw.create( noSleep ) );
	const BodyHandle nb = nw.createBody( dyn, p0, qid );
	CHECK( nb && nw.createShape( nb, dyn, cube ) );
	nw.setSleepEnabledByHandle( nb, true );
	const float zv[3] = { 0, 0, 0 };
	nw.setVelocityByHandle( nb, zv, zv );
	for ( int i = 0; i < 120; ++i )
		nw.step( 1.0f / 60.0f );
	CHECK( nw.isAwakeByHandle( nb ) ); // still falling, but more to the point: never sleeps
	// tuning on the live world: turning sleep back on lets it sleep once it
	// rests
	WorldSettings on = noSleep;
	on.enableSleep = true;
	on.gravity[1] = 0.0f;
	nw.applyTuning( on );
	nw.setVelocityByHandle( nb, zv, zv );
	for ( int i = 0; i < 120; ++i )
		nw.step( 1.0f / 60.0f );
	CHECK( !nw.isAwakeByHandle( nb ) );
	std::printf( "mid-sim update checks done\n" );
}

struct TestCase
{
	const char* name;
	void ( *run )();
};

// The cases, in run order. Keep each entry on one line as {"name", function}:
// CMake reads the names from these lines to register one CTest test per case.
const TestCase kTests[] = {
	{ "body_basics", testBodyBasics },
	{ "body_extent", testBodyExtent },
	{ "hull_centroid", testHullCentroid },
	{ "point_mapping", testPointMapping },
	{ "convex_shapes", testConvexShapes },
	{ "primitive_overrides", testPrimitiveOverrides },
	{ "move_events", testMoveEvents },
	{ "stored_pose", testStoredPose },
	{ "handle_build", testHandleBuild },
	{ "mid_sim_changes", testMidSimChanges },
	{ "fast_rotation", testFastRotation },
	{ "body_props_equality", testBodyPropsEquality },
	{ "body_props_sanitize", testBodyPropsSanitize },
	{ "ccd_threshold", testCcdThreshold },
	{ "collision_filtering", testCollisionFiltering },
	{ "mass_sleep_kinematic", testMassSleepKinematic },
	{ "kinematic_bounce", testKinematicBounce },
	{ "external_forces", testExternalForces },
	{ "contact_recycling", testContactRecycling },
	{ "surface_material", testSurfaceMaterial },
	{ "world_settings", testWorldSettings },
	{ "hull_cache", testHullCache },
	{ "mirrored_packed", testMirroredPacked },
	{ "estimated_bytes", testEstimatedBytes },
	{ "mix_hash", testMixHash },
	{ "sleep_enable", testSleepEnable },
	{ "bulk_create", testBulkCreate },
	{ "lazy_pose", testLazyPose },
	{ "step_profile", testStepProfile },
};

// Usage: b3dop_core_physics [--list | <case>...]
//   (no arguments)  run every case
//   <case>...       run only the named cases
//   --list          print the case names, one per line
int main( int argc, char** argv )
{
	std::vector<const TestCase*> selected;
	if ( argc <= 1 )
	{
		for ( const TestCase& t : kTests )
			selected.push_back( &t );
	}
	else if ( std::strcmp( argv[1], "--list" ) == 0 )
	{
		for ( const TestCase& t : kTests )
			std::printf( "%s\n", t.name );
		return 0;
	}
	else
	{
		for ( int i = 1; i < argc; ++i )
		{
			const TestCase* found = nullptr;
			for ( const TestCase& t : kTests )
				if ( std::strcmp( t.name, argv[i] ) == 0 )
					found = &t;
			if ( !found )
			{
				std::printf( "unknown test case '%s' (try --list)\n", argv[i] );
				return 2;
			}
			selected.push_back( found );
		}
	}

	std::vector<const char*> failed;
	for ( const TestCase* t : selected )
	{
		std::printf( "[ RUN  ] %s\n", t->name );
		std::fflush( stdout ); // so a crash still shows which case was running
		const int before = fails;
		try
		{
			t->run();
		}
		catch ( const std::exception& e )
		{
			std::printf( "FAIL %s: uncaught exception: %s\n", t->name, e.what() );
			++fails;
		}
		catch ( ... )
		{
			std::printf( "FAIL %s: uncaught non-standard exception\n", t->name );
			++fails;
		}
		if ( fails == before )
		{
			std::printf( "[  OK  ] %s\n", t->name );
		}
		else
		{
			std::printf( "[ FAIL ] %s (%d failed)\n", t->name, fails - before );
			failed.push_back( t->name );
		}
	}

	if ( failed.empty() )
	{
		std::printf( "OK: %zu case(s) passed\n", selected.size() );
		return 0;
	}
	std::printf( "FAILED: %zu of %zu case(s):", failed.size(), selected.size() );
	for ( const char* name : failed )
		std::printf( " %s", name );
	std::printf( "\n" );
	return 1;
}
