// Box3DGlue.h
//
// Header-only, Houdini-free: SideFX Glue's impulse model (the Glue Constraint
// Relationship help, as the Bullet solver runs it). A glued pair moves as one
// until things colliding with it put enough impulse on the bond:
//   * every body collects the impulse of its contacts (hits) over the step;
//   * that impulse spreads over the bond network, `iterations` rounds: a body
//     keeps a share of 1 and gives each neighbour a share of the bond's
//     propagation rate (0 to 1), all divided by the sum of the shares;
//   * each bond adds the impulses its two bodies ended up with to its `impact`,
//     after that has decayed by the half-life;
//   * a bond whose impact passes its strength (< 0: never) breaks. So does one
//     whose decayed impact plus the spread estimates (core/Box3DGlueEstimate.h:
//     what Bullet breaks glue on before the step, from every contact point of
//     the glued body) passes it, and that impact is what it reports then.
// A hit is the normal impulse of a body's contacts, as in Bullet, so a body
// resting on something adds its weight every step. A piece hanging from its
// bond has no contacts and holds at any strength.
#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace b3dop
{

// What a body must be able to deliver for holdCompoundsForApproach to free the
// pieces in its way, as a share of the weakest bond's Strength: Bullet breaks a
// glued body that lands on an estimate several times the impulse the step
// carries (core/Box3DGlueEstimate.h), which the momentum it brings falls short
// of by about this much. A heuristic for when to free pieces, not the break
// test.
constexpr float kGlueHoldBreakScale = 5.3f;

// An impact below this (N*s) is zero: a decayed impact would otherwise halve
// for minutes before it underflowed, and the frame would not be idle until it
// did. Far below anything a bond's strength is set against.
constexpr float kGlueImpactFloor = 1e-6f;

// A compound is taken apart (see holdCompoundsForApproach) for a body that is
// faster than this (m/s) and can deliver more than the weakest bond's break
// impulse, and stays apart this many seconds after the last such body came
// near it.
constexpr float kGlueHoldMinSpeed = 2.0f;
constexpr float kGlueHoldSeconds = 2.0f;
// How far ahead (seconds of the body's motion) it is looked for: only when the
// release starts, which has to be before the body touches the compound (the
// pieces must be free in the step it hits; at least two frames are used, see
// holdCompoundsForApproach). Which pieces are freed is the body's whole way
// through the compound, not what it reaches in this time.
constexpr float kGlueHoldLeadSeconds = 0.25f;
// The pieces taken out of a compound are those in the body's swept bounds grown
// by this many times its own half size, so the pieces around its path can
// give way too.
constexpr float kGlueHoldHaloScale = 3.0f;
// The most rounds of propagation a bond may ask for: each costs a pass over
// every bond every frame, so a typo (1e6) must not freeze the solve.
constexpr int kMaxGlueIterations = 1000;

struct GlueParams
{
	float strength = 10000.0f; // impulse to break; < 0 never
	float halfLife = 0.1f;	   // seconds for the impact to halve
	float rate = 1.0f;		   // propagation rate, clamped to 0..1
	int iterations = 1;		   // rounds of propagation; < 1 counts as 1
};

// A propagation rate kept to 0..1. A non-finite one (std::clamp would pass a
// NaN on, and every share it touches would become NaN) takes the default.
inline float glueRate( float rate )
{
	if ( !std::isfinite( rate ) )
		return GlueParams().rate;
	return std::clamp( rate, 0.0f, 1.0f );
}

// One bond between two nodes (bodies). `impact` is the Glue's `impact`
// attribute: it persists from step to step.
struct GlueBond
{
	int a = -1, b = -1;
	GlueParams params;
	float impact = 0.0f;
};

// The impulse each node ends up with after `rounds` rounds of propagation.
// `hits` per node; bonds with fewer iterations than the round stop carrying
// (an impulse never travels more bonds than the bond's iterations).
inline void glueSpread( const std::vector<GlueBond>& bonds, std::vector<float>& hits )
{
	int rounds = 1;
	for ( const GlueBond& bond : bonds )
		rounds = std::min( std::max( rounds, bond.params.iterations ), kMaxGlueIterations );
	const size_t n = hits.size();
	// Kept between calls (every frame): no allocation. `next` trades buffers
	// with `hits` below; both are fully overwritten each round.
	thread_local std::vector<float> next, weight;
	next.resize( n );
	weight.resize( n );
	for ( int round = 0; round < rounds; ++round )
	{
		// Each node's total share: itself (1) and its bonds' rates.
		std::fill( weight.begin(), weight.end(), 1.0f );
		for ( const GlueBond& bond : bonds )
		{
			if ( bond.params.iterations <= round )
				continue;
			const float r = glueRate( bond.params.rate );
			weight[static_cast<size_t>( bond.a )] += r;
			weight[static_cast<size_t>( bond.b )] += r;
		}
		for ( size_t i = 0; i < n; ++i )
			next[i] = hits[i] / weight[i];
		for ( const GlueBond& bond : bonds )
		{
			if ( bond.params.iterations <= round )
				continue;
			const float r = glueRate( bond.params.rate );
			const size_t a = static_cast<size_t>( bond.a ), b = static_cast<size_t>( bond.b );
			next[b] += hits[a] * r / weight[a];
			next[a] += hits[b] * r / weight[b];
		}
		hits.swap( next );
	}
}

// One step of `dt` seconds: the bonds' impacts decay, take the propagated
// hits, and those over their strength are listed in `broken` (indices into
// `bonds`). `hits` (per node) is consumed. `estimates` (per node, or empty):
// the frame's impulse estimates (glueContactEstimate): spread the same way, a
// bond whose decayed impact plus its ends' estimates passes its strength breaks
// too, and keeps that as its impact. A bond with a < 0 or b < 0 is ignored.
inline void glueAdvance( std::vector<GlueBond>& bonds, std::vector<float> hits, std::vector<float> estimates, float dt,
						 std::vector<int>& broken )
{
	broken.clear();
	// Node counts in `bonds` must fit `hits`.
	const auto usable = [&hits]( const GlueBond& bond ) {
		return bond.a >= 0 && bond.b >= 0 && static_cast<size_t>( bond.a ) < hits.size() &&
			   static_cast<size_t>( bond.b ) < hits.size();
	};
	// Kept between calls: no allocation. Only filled when some bond is ignored;
	// otherwise `bonds` is spread as it is.
	thread_local std::vector<GlueBond> live;
	thread_local std::vector<int> index;
	live.clear();
	index.clear();
	const bool all = std::all_of( bonds.begin(), bonds.end(), usable );
	if ( !all )
	{
		live.reserve( bonds.size() );
		index.reserve( bonds.size() );
		for ( size_t i = 0; i < bonds.size(); ++i )
		{
			if ( !usable( bonds[i] ) )
				continue;
			live.push_back( bonds[i] );
			index.push_back( static_cast<int>( i ) );
		}
	}
	const std::vector<GlueBond>& spread = all ? bonds : live;
	glueSpread( spread, hits );
	const bool estimated = estimates.size() == hits.size() &&
						   std::any_of( estimates.begin(), estimates.end(), []( float e ) { return e > 0.0f; } );
	if ( estimated )
		glueSpread( spread, estimates );
	// Most bonds share a half-life: the decay is worked out once per value.
	float lastHalfLife = 0.0f, lastDecay = 0.0f;
	bool haveDecay = false;
	for ( size_t k = 0; k < spread.size(); ++k )
	{
		const size_t at = all ? k : static_cast<size_t>( index[k] );
		GlueBond& bond = bonds[at];
		const float hl = bond.params.halfLife;
		if ( !haveDecay || !( hl == lastHalfLife ) )
		{
			lastDecay = hl > 0.0f && dt > 0.0f ? std::exp2( -dt / hl ) : ( hl > 0.0f ? 1.0f : 0.0f );
			lastHalfLife = hl;
			haveDecay = true;
		}
		const float decayed = bond.impact * lastDecay;
		bond.impact = decayed + hits[static_cast<size_t>( bond.a )] + hits[static_cast<size_t>( bond.b )];
		if ( bond.impact < kGlueImpactFloor ) // not for NaN: that stays visible
			bond.impact = 0.0f;
		const bool breakable = bond.params.strength >= 0.0f && std::isfinite( bond.params.strength );
		if ( !breakable )
			continue;
		if ( bond.impact > bond.params.strength )
		{
			broken.push_back( static_cast<int>( at ) );
			continue;
		}
		if ( estimated )
		{
			const float before =
				decayed + estimates[static_cast<size_t>( bond.a )] + estimates[static_cast<size_t>( bond.b )];
			if ( before > bond.params.strength )
			{
				bond.impact = before;
				broken.push_back( static_cast<int>( at ) );
			}
		}
	}
}

// The same without estimates: a bond breaks on its propagated hits alone.
inline void glueAdvance( std::vector<GlueBond>& bonds, std::vector<float> hits, float dt, std::vector<int>& broken )
{
	glueAdvance( bonds, std::move( hits ), std::vector<float>(), dt, broken );
}

} // namespace b3dop
