// Box3DMotion.h
//
// Header-only, Houdini-free: the per-point drag and speed limits RBD Bullet
// Solver reads from POP-style attributes (airresist, spinresist, drag,
// dragexp, targetv, targetw, speedmin, speedmax, spinmin, spinmax, accelmax,
// angaccelmax), as SIM_BulletRigidBody applies them: a drag that slows the body
// towards a target velocity, clamps on its speed and spin, and on the change in
// them the constraints make.
#pragma once

#include <algorithm>
#include <cmath>

namespace b3dop
{

// One point's values. The limits are off below 0, as in Bullet; the resists
// already include `drag` (a factor on both).
struct MotionParams
{
	float airResist = 0.0f;
	float spinResist = 0.0f;
	float dragExp = 2.0f; // 1 (linear) to 2 (quadratic)
	float targetV[3] = { 0.0f, 0.0f, 0.0f };
	float targetW[3] = { 0.0f, 0.0f, 0.0f };
	float minSpeed = -1.0f, maxSpeed = -1.0f;
	float minSpin = -1.0f, maxSpin = -1.0f;
	// accelmax / angaccelmax: the most the constraints (contacts and joints) may
	// change the velocity / spin, per second.
	float maxAccel = -1.0f, maxAngAccel = -1.0f;

	bool hasDrag() const
	{
		return airResist > 0.0f || spinResist > 0.0f;
	}
	bool hasLimits() const
	{
		return minSpeed >= 0.0f || maxSpeed >= 0.0f || minSpin >= 0.0f || maxSpin >= 0.0f;
	}
	bool hasAccelLimits() const
	{
		return maxAccel >= 0.0f || maxAngAccel >= 0.0f;
	}
	bool any() const
	{
		return hasDrag() || hasLimits() || hasAccelLimits();
	}
};

// Bullet's: the exponent stays between linear and quadratic drag.
inline float clampDragExp( float e )
{
	return std::isfinite( e ) ? std::min( std::max( e, 1.0f ), 2.0f ) : 2.0f;
}

namespace motion_detail
{
inline double safeDiv( double a, double b )
{
	return b != 0.0 ? a / b : 0.0;
}
} // namespace motion_detail

// What drag does to a velocity `v` over `dt`, applied implicitly (so a strong
// drag cannot overshoot): the velocity relative to `target` decays under
// resist * invMass * |v - target|^(dragExp - 1) (see POP's drag), exactly
// integrated for the linear, quadratic and in-between exponents. Bullet's
// simApplyImplicitDrag. The angular case passes invMass = 1.
inline void applyImplicitDrag( float dragExp, float resist, const float target[3], float dt, float invMass, float v[3] )
{
	double r[3] = { static_cast<double>( v[0] ) - target[0], static_cast<double>( v[1] ) - target[1],
					static_cast<double>( v[2] ) - target[2] };
	const double coeff = static_cast<double>( resist ) * invMass;
	const double len = std::sqrt( r[0] * r[0] + r[1] * r[1] + r[2] * r[2] );
	double scale = 1.0;
	if ( std::fabs( dragExp - 2.0f ) < 1e-6f )
	{
		scale = motion_detail::safeDiv( 1.0, coeff * dt * len + 1.0 );
	}
	else if ( std::fabs( dragExp - 1.0f ) < 1e-6f )
	{
		scale = std::exp( -coeff * dt );
	}
	else
	{
		const double a = ( 2.0 - dragExp ) * coeff, b = ( dragExp - 1.0 ) * coeff;
		const double t0 = motion_detail::safeDiv( std::log( motion_detail::safeDiv( a, b * len ) + 1.0 ), a );
		const double vNew = motion_detail::safeDiv( a, b * ( std::exp( a * ( t0 + dt ) ) - 1.0 ) );
		scale = motion_detail::safeDiv( vNew, len );
	}
	for ( int k = 0; k < 3; ++k )
		v[k] = static_cast<float>( r[k] * scale + target[k] );
}

// The clamps on a body's velocities (Bullet's clampSpeed): a speed under the
// minimum is raised to it and one over the maximum cut to it, never turning a
// rest into motion; the same for the spin. Both tests use the speed from
// before either change, as Bullet's do. True if either changed.
inline bool clampMotion( const MotionParams& m, float lin[3], float ang[3] )
{
	bool changed = false;
	const auto clampOne = [&]( float v[3], float lo, float hi ) {
		const float speed = std::sqrt( v[0] * v[0] + v[1] * v[1] + v[2] * v[2] );
		if ( !( speed > 0.0f ) || !std::isfinite( speed ) )
			return;
		float factor = 1.0f;
		if ( lo >= 0.0f && speed < lo )
			factor *= lo / speed;
		if ( hi >= 0.0f && speed > hi )
			factor *= hi / speed;
		if ( factor != 1.0f )
		{
			for ( int k = 0; k < 3; ++k )
				v[k] *= factor;
			changed = true;
		}
	};
	clampOne( lin, m.minSpeed, m.maxSpeed );
	clampOne( ang, m.minSpin, m.maxSpin );
	return changed;
}

// What a Box3D step of `dt` makes of velocity `v0` with no constraints: an
// acceleration `accel` (gravity and forces, constant over the step) and a
// damping `damping`, over the step's `subSteps` solver substeps (Constraint
// Iterations), each v <- accel h + v / (1 + h damping) with h = dt / subSteps,
// as Box3D's integrate velocities does (the substep's own change undamped).
// Closed form: d^n v0 + accel h (1 - d^n) / (1 - d), d = 1 / (1 + h damping).
inline void undisturbedVelocity( const float v0[3], const float accel[3], float damping, float dt, int subSteps,
								 float out[3] )
{
	const int n = std::max( subSteps, 1 );
	const double h = static_cast<double>( dt ) / n;
	const double c = damping > 0.0f && std::isfinite( damping ) ? damping : 0.0;
	const double d = 1.0 / ( 1.0 + h * c );
	const double dn = std::pow( d, n );
	// The sum 1 + d + ... + d^(n-1): n without damping.
	const double sum = c > 0.0 ? ( 1.0 - dn ) / ( 1.0 - d ) : static_cast<double>( n );
	for ( int k = 0; k < 3; ++k )
		out[k] = static_cast<float>( dn * v0[k] + accel[k] * h * sum );
}

// Bullet's accelmax / angaccelmax on one velocity: `v` after a step of `dt`
// and `expected`, what the step's forces and gravity alone would have made of
// the velocity before it. Their difference, the change the constraints made, is
// shortened to at most `maxAccel * dt` (a negative `maxAccel`: no limit), so
// the velocity carried on is changed and the step's motion is not. True if `v`
// changed.
inline bool clampConstraintChange( float maxAccel, float dt, const float expected[3], float v[3] )
{
	if ( !( maxAccel >= 0.0f ) || !( dt > 0.0f ) || !std::isfinite( maxAccel ) )
		return false;
	const double d[3] = { static_cast<double>( v[0] ) - expected[0], static_cast<double>( v[1] ) - expected[1],
						  static_cast<double>( v[2] ) - expected[2] };
	const double len = std::sqrt( d[0] * d[0] + d[1] * d[1] + d[2] * d[2] );
	const double cap = static_cast<double>( maxAccel ) * dt;
	if ( !( len > cap ) || !std::isfinite( len ) )
		return false;
	const double scale = cap / len;
	for ( int k = 0; k < 3; ++k )
		v[k] = static_cast<float>( expected[k] + d[k] * scale );
	return true;
}

} // namespace b3dop
