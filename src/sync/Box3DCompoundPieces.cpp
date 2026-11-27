// Box3DCompoundPieces.cpp
//
// What the solver asks of a compound's pieces each step: their moves, live
// poses, glue hits and the state and force a Force DOP sees.
#include "Box3DCompoundInternal.h"
#include "Box3DCompoundSync.h"
#include "Box3DHoudiniUtil.h"
#include "Box3DJointSync.h"
#include "Box3DState.h"
#include "core/Box3DCompound.h"
#include "core/Box3DDebug.h"
#include "core/Box3DMath.h"
#include "core/Box3DShapes.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iterator>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace compoundsync;

void recordCompoundMotion( Box3DSharedState& st )
{
	if ( st.glueCompoundLoad <= 0.0f )
		return;
	for ( auto& c : st.compounds )
	{
		b3dop::MergedBody mb;
		bool awake = false;
		c->stepPreOk = st.world.isAwakeByHandle( c->body ) && bodyMotion( st.world, c->body, mb, awake ) && awake;
		c->stepPre = mb;
	}
}

void expandCompoundMoves( Box3DSharedState& st )
{
	if ( st.compounds.empty() )
		return;
	b3dop::Box3DWorld& w = st.world;
	std::unordered_map<BodyHandle, Box3DCompound*> byBody;
	byBody.reserve( st.compounds.size() );
	for ( auto& c : st.compounds )
		byBody[c->body] = c.get();
	const size_t original = st.moves.size();
	bool moved = false;
	for ( size_t i = 0; i < original; ++i )
	{
		const auto it = byBody.find( st.moves[i].handle );
		if ( it == byBody.end() )
			continue;
		moved = true;
		const b3dop::BodyMove ev = st.moves[i]; // push_back may move the vector
		Box3DCompound& c = *it->second;
		Pose cp;
		b3dop::copy3( cp.p, ev.pos );
		b3dop::copy4( cp.q, ev.rot );
		b3dop::MergedBody mb;
		bool awake = false;
		if ( !bodyMotion( w, c.body, mb, awake ) )
			continue;
		for ( Box3DCompoundMember& m : c.members )
		{
			if ( !m.entry )
				continue;
			const Pose pp = placed( cp, m );
			double com[3];
			centreOf( pp, m, com );
			b3dop::pieceVelocity( mb, com, m.entry->glueLin, m.entry->glueAng );
			st.moves.push_back( moveAt( *m.entry, m.body, pp, ev.fellAsleep, m.entry->glueLin, m.entry->glueAng ) );
		}
	}
	// The parked bodies are where the pieces were: whatever reads one first
	// puts it where the piece is now (Box3DWorld::setLazyPose).
	if ( moved )
		w.invalidateLazyPoses();
}

void refreshLivePoses( Box3DSharedState& st )
{
	for ( auto& c : st.compounds )
		c->livePoseOk = bodyPose( st.world, c->body, c->livePose );
}

bool compoundJointPoses( const Box3DSharedState& st, const Box3DJointEntry& e, b3dop::BodyPose& poseA,
						 b3dop::BodyPose& poseB )
{
	const auto a = st.compoundOf.find( e.bodyA );
	const auto b = st.compoundOf.find( e.bodyB );
	if ( a == st.compoundOf.end() || b == st.compoundOf.end() || a->second.first != b->second.first )
		return false;
	const Box3DCompound& c = *a->second.first;
	if ( !c.livePoseOk )
		return false;
	poseA = placed( c.livePose, c.members[static_cast<size_t>( a->second.second )] );
	poseB = placed( c.livePose, c.members[static_cast<size_t>( b->second.second )] );
	return true;
}

bool compoundGlueHits( Box3DSharedState& st, BodyHandle body, float& hits )
{
	const auto it = st.compoundOf.find( body );
	if ( it == st.compoundOf.end() )
		return false;
	Box3DCompound& c = *it->second.first;
	const Box3DCompoundMember& m = c.members[static_cast<size_t>( it->second.second )];
	hits = 0.0f;
	// Per compound, once a tick: hits land on moving bodies, a sleeping one has
	// none.
	if ( c.hitsTick != st.hitsTick )
	{
		c.hitsTick = st.hitsTick;
		c.hitsActive =
			st.world.typeByHandle( c.body ) == b3dop::BodySettings::Type::Dynamic && st.world.isAwakeByHandle( c.body );
		if ( c.hitsActive )
		{
			// Contacts with pieces of its own group (the pieces held out of it, say,
			// resting on it) carry weight, not hits.
			const int group = compoundGlueGroup( st, c );
			const std::function<bool( BodyHandle )> sameGroup = [&st, group]( BodyHandle other ) {
				return glueIgnoresContact( st, other, group );
			};
			st.world.contactImpulsesByShape( c.body, c.shapeImpulse, group >= 0 ? &sameGroup : nullptr );
		}
	}
	if ( !c.hitsActive )
		return true;
	for ( int i = m.shapeFirst; i < m.shapeFirst + m.shapeCount && static_cast<size_t>( i ) < c.shapeImpulse.size();
		  ++i )
		hits += c.shapeImpulse[static_cast<size_t>( i )];
	// Inertial load: slowing the whole body at one point also loads the pieces
	// away from it. A piece takes at least the impulse it needs to follow the
	// body's change of motion (a piece touching the ground keeps its contact
	// hit when that is larger).
	if ( st.glueCompoundLoad > 0.0f && c.stepPreOk )
	{
		if ( c.loadTick != st.hitsTick )
		{
			c.loadTick = st.hitsTick;
			bool awake = false;
			if ( !bodyPose( st.world, c.body, c.postPose ) || !bodyMotion( st.world, c.body, c.stepPost, awake ) )
				c.stepPost.ok = false;
		}
		if ( c.stepPost.ok && c.stepPre.ok )
		{
			double com[3];
			centreOf( placed( c.postPose, m ), m, com );
			hits = std::max( hits, st.glueCompoundLoad * b3dop::pieceLoad( c.stepPre, c.stepPost, com, m.mass ) );
		}
	}
	return true;
}

bool compoundPieceState( Box3DSharedState& st, const Box3DBodyMirror& piece, b3dop::BodyState& out )
{
	const auto it = st.compoundOf.find( piece.handle );
	if ( it == st.compoundOf.end() )
		return false;
	const Box3DCompound& c = *it->second.first;
	const Box3DCompoundMember& m = c.members[static_cast<size_t>( it->second.second )];
	Pose cp;
	b3dop::MergedBody mb;
	bool awake = false;
	b3dop::BodyState own;
	if ( !bodyPose( st.world, c.body, cp ) || !bodyMotion( st.world, c.body, mb, awake ) ||
		 !st.world.bodyStateByHandle( piece.handle, own ) )
		return false;
	const Pose pp = placed( cp, m );
	out.awake = awake;
	out.mass = own.mass;
	centreOf( pp, m, out.com );
	b3dop::pieceVelocity( mb, out.com, out.lin, out.ang );
	return true;
}

bool compoundPieceAwake( Box3DSharedState& st, const Box3DBodyMirror& piece )
{
	const auto it = st.compoundOf.find( piece.handle );
	return it != st.compoundOf.end() && st.world.isAwakeByHandle( it->second.first->body );
}

bool compoundPieceForce( Box3DSharedState& st, const Box3DBodyMirror& piece, float force[3], float torque[3],
						 BodyHandle& target )
{
	const auto it = st.compoundOf.find( piece.handle );
	if ( it == st.compoundOf.end() )
		return false;
	const Box3DCompound& c = *it->second.first;
	const Box3DCompoundMember& m = c.members[static_cast<size_t>( it->second.second )];
	Pose cp;
	b3dop::BodyState cs;
	if ( !bodyPose( st.world, c.body, cp ) || !st.world.bodyStateByHandle( c.body, cs ) )
		return false;
	const Pose pp = placed( cp, m );
	// The torque about the compound's centre of mass: the piece's own plus
	// (where it acts - centre) x force. A force acts at the piece's centre.
	double at[3];
	centreOf( pp, m, at );
	const double r[3] = { at[0] - cs.com[0], at[1] - cs.com[1], at[2] - cs.com[2] };
	const double f[3] = { force[0], force[1], force[2] };
	double rxf[3];
	b3dop::cross3( r, f, rxf );
	for ( int i = 0; i < 3; ++i )
		torque[i] += static_cast<float>( rxf[i] );
	target = c.body;
	return true;
}
