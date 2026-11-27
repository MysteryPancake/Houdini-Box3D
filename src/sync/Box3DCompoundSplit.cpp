// Box3DCompoundSplit.cpp
//
// Bonds that broke inside compounds: which pieces leave a compound, as small
// groups that split off it (Box3DCompoundSync.h, `splitOffSmallPieces`).
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

namespace
{

// What bonds breaking inside one compound do to it, found from the broken
// bonds' nodes alone.
struct SmallSplit
{
	Box3DCompound* compound = nullptr;
	// True: the pieces that stay are everything not in `leaving`. False: they are
	// `stay` (every piece was reached, so all are listed).
	bool remainderStays = true;
	std::vector<int> stay;
	// The glue nodes of each group of pieces that leaves the compound.
	std::vector<std::vector<int>> leaving;
};

// Searches from `seeds` (glue nodes of `c`, the ends of the bonds that broke),
// each one a node at a time in turn, so the work is in proportion to the pieces
// that came loose and not to the compound. Searches that meet are one; the
// search stops when at most one is still growing: what it has not reached is
// the rest of the compound, which stays. Gives 1 with `out` set when pieces
// leave, 0 when the compound is as it was, and -1 when the work is not simple
// enough (a search reaches a piece outside the compound, the part that stays is
// not the biggest or is too small to be a compound): syncCompounds then works
// the components out the full way.
int planSmallSplit( const Box3DSharedState& st, Box3DCompound& c, const std::vector<int>& seeds, SmallSplit& out )
{
	const size_t n = seeds.size();
	std::vector<std::vector<int>> queue( n );
	std::vector<size_t> head( n, 0 ), parent( n );
	std::unordered_map<int, size_t> owner; // node -> the search that reached it
	for ( size_t i = 0; i < n; ++i )
	{
		queue[i].push_back( seeds[i] );
		parent[i] = i;
		owner.emplace( seeds[i], i );
	}
	const auto find = [&parent]( size_t x ) {
		while ( parent[x] != x )
		{
			parent[x] = parent[parent[x]];
			x = parent[x];
		}
		return x;
	};
	std::vector<char> growing( n );
	size_t aliveCount = 0;
	for ( ;; )
	{
		std::fill( growing.begin(), growing.end(), 0 );
		for ( size_t i = 0; i < n; ++i )
			if ( head[i] < queue[i].size() )
				growing[find( i )] = 1;
		aliveCount = static_cast<size_t>( std::count( growing.begin(), growing.end(), static_cast<char>( 1 ) ) );
		if ( aliveCount <= 1 )
			break;
		for ( size_t i = 0; i < n; ++i )
		{
			if ( head[i] >= queue[i].size() )
				continue;
			const int node = queue[i][head[i]++];
			for ( const BodyHandle h : st.glueNodes[static_cast<size_t>( node )].partners )
			{
				const auto it = st.glueNodeOf.find( h );
				if ( it == st.glueNodeOf.end() || heldNode( st, it->second ) )
					continue;
				const auto in = st.compoundOf.find( h );
				if ( in == st.compoundOf.end() || in->second.first != &c )
					return -1; // the bonds lead out of the compound
				const auto ins = owner.emplace( it->second, i );
				if ( ins.second )
				{
					queue[i].push_back( it->second );
				}
				else
				{
					const size_t a = find( i ), b = find( ins.first->second );
					if ( a != b )
						parent[a] = b;
				}
			}
		}
	}

	// The searches that ran out are whole components.
	std::unordered_map<size_t, std::vector<int>> byRoot;
	for ( const auto& kv : owner )
		byRoot[find( kv.second )].push_back( kv.first );
	std::vector<std::vector<int>> done;
	size_t doneTotal = 0;
	for ( auto& kv : byRoot )
	{
		if ( aliveCount == 1 && growing[kv.first] )
			continue;
		std::sort( kv.second.begin(), kv.second.end() );
		doneTotal += kv.second.size();
		done.push_back( std::move( kv.second ) );
	}
	std::sort( done.begin(), done.end(),
			   []( const std::vector<int>& a, const std::vector<int>& b ) { return a.front() < b.front(); } );

	const size_t members = c.members.size();
	size_t best = 0;
	if ( aliveCount == 1 )
	{
		if ( done.empty() )
			return 0; // all still joined
		if ( doneTotal >= members )
			return -1;
		best = members - doneTotal;
		for ( const auto& d : done )
			if ( d.size() > best )
				return -1; // the rest is not the biggest piece left
		out.remainderStays = true;
		out.leaving = std::move( done );
	}
	else
	{
		if ( doneTotal != members )
			return -1;
		if ( done.size() == 1 )
			return 0;
		size_t stayAt = 0;
		for ( size_t k = 1; k < done.size(); ++k )
			if ( done[k].size() > done[stayAt].size() )
				stayAt = k;
		best = done[stayAt].size();
		out.remainderStays = false;
		out.stay = std::move( done[stayAt] );
		done.erase( done.begin() + static_cast<std::ptrdiff_t>( stayAt ) );
		out.leaving = std::move( done );
	}
	if ( best < 2 || best * 2 < members )
		return -1; // the compound would not stay (dissolved the full way)
	out.compound = &c;
	return 1;
}

// A piece group that leaves a compound, and whether it can become a compound of
// its own (the checks the full regrouping makes for every group).
struct LeavingGroup
{
	std::vector<Box3DBodyMirror*> pieces;
	bool ok = true;
};

LeavingGroup leavingGroupOf( Box3DSharedState& st, const std::vector<int>& nodes )
{
	LeavingGroup g;
	for ( const int node : nodes )
	{
		const BodyHandle h = st.glueNodes[static_cast<size_t>( node )].body;
		Box3DBodyMirror* e = static_cast<Box3DBodyMirror*>( st.world.bodyUserDataByHandle( h ) );
		if ( !e || e->handle != h || e->wholeObject || !st.isLive( *e ) || !eligible( *e ) || st.tiedBodies.count( h ) )
		{
			g.ok = false;
			continue;
		}
		g.pieces.push_back( e );
	}
	for ( const Box3DBodyMirror* e : g.pieces )
		if ( !sameBodyLevel( e->props, g.pieces[0]->props ) )
			g.ok = false;
	return g;
}

} // namespace

namespace compoundsync
{

// Bonds broke inside compounds (st.glueTouched: their nodes). For each compound
// that loses pieces, the pieces leave it (shrink) and groups of two or more
// become compounds of their own, with work in proportion to the pieces that
// left and not to the compounds. What it cannot do that way is left in
// glueTouched for the full regrouping.
void splitOffSmallPieces( Box3DSharedState& st )
{
	std::vector<std::pair<Box3DCompound*, std::vector<int>>> byCompound;
	std::vector<int> left; // nodes for the full regrouping
	{
		std::unordered_set<int> seen;
		for ( const int node : st.glueTouched )
		{
			if ( !seen.insert( node ).second )
				continue;
			const auto in = st.compoundOf.find( st.glueNodes[static_cast<size_t>( node )].body );
			if ( in == st.compoundOf.end() || heldNode( st, node ) )
			{
				left.push_back( node );
				continue;
			}
			Box3DCompound* const c = in->second.first;
			auto it =
				std::find_if( byCompound.begin(), byCompound.end(), [c]( const auto& p ) { return p.first == c; } );
			if ( it == byCompound.end() )
			{
				byCompound.emplace_back( c, std::vector<int>() );
				it = std::prev( byCompound.end() );
			}
			it->second.push_back( node );
		}
	}

	// Plan every compound before changing any.
	std::vector<SmallSplit> plans;
	for ( auto& entry : byCompound )
	{
		SmallSplit plan;
		Pose cp;
		b3dop::MergedBody mb;
		bool awake = false;
		int result = -1;
		// shrink needs the body's pose and motion
		if ( bodyPose( st.world, entry.first->body, cp ) && bodyMotion( st.world, entry.first->body, mb, awake ) )
			result = planSmallSplit( st, *entry.first, entry.second, plan );
		if ( result == 1 )
			plans.push_back( std::move( plan ) );
		else if ( result < 0 )
			left.insert( left.end(), entry.second.begin(), entry.second.end() );
	}

	std::vector<int> affectedNodes;
	for ( SmallSplit& plan : plans )
	{
		Box3DCompound& c = *plan.compound;
		std::vector<LeavingGroup> groups;
		groups.reserve( plan.leaving.size() );
		for ( const std::vector<int>& nodes : plan.leaving )
			groups.push_back( leavingGroupOf( st, nodes ) );

		std::vector<int> cid( c.members.size(), plan.remainderStays ? 0 : 1 );
		const auto setMembers = [&]( const std::vector<int>& nodes, int value ) {
			for ( const int node : nodes )
			{
				const auto in = st.compoundOf.find( st.glueNodes[static_cast<size_t>( node )].body );
				if ( in != st.compoundOf.end() )
					cid[static_cast<size_t>( in->second.second )] = value;
			}
		};
		if ( plan.remainderStays )
			for ( const std::vector<int>& nodes : plan.leaving )
				setMembers( nodes, 1 );
		else
			setMembers( plan.stay, 0 );
		for ( const std::vector<int>& nodes : plan.leaving )
			affectedNodes.insert( affectedNodes.end(), nodes.begin(), nodes.end() );

		if ( !shrink( st, c, cid, 0 ) )
		{
			// (the body was readable when planned) leave it to the full way
			st.groupsRecheck = true;
			continue;
		}
		for ( LeavingGroup& g : groups )
		{
			if ( !g.ok || g.pieces.size() < 2 )
				continue;
			std::sort( g.pieces.begin(), g.pieces.end(),
					   []( const Box3DBodyMirror* a, const Box3DBodyMirror* b ) { return keyOf( *a ) < keyOf( *b ); } );
			if ( merge( st, g.pieces ) && !st.glueHold.empty() )
				bridgeBoundaries( st, *st.compounds.back() );
		}
	}
	if ( !affectedNodes.empty() )
		markCompoundBonds( st, &affectedNodes );
	st.glueTouched = std::move( left );
}

} // namespace compoundsync
