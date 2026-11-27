// Box3DGlueMaterial.cpp
//
// A glued group's material, as RBD Bullet Solver gives it: Bullet runs a glue
// cluster as one compound body with one friction and bounce, the pieces'
// values weighted by their mass (test bullet_glue_materials). Every dynamic
// piece of a group, and the body of each compound of it, gets that average;
// a piece that leaves every group (its bonds broke, its network went away)
// gets its own material back.
#include "Box3DCompoundSync.h"
#include "Box3DJointSync.h"
#include "Box3DState.h"
#include "core/Box3DShapes.h"

#include <algorithm>
#include <vector>

void applyGlueMaterials( Box3DSharedState& st, bool bodiesChanged )
{
	if ( !bodiesChanged && st.glueMaterialKnown && st.glueMaterialSignature == st.glueSignature &&
		 st.glueMaterialNodes == st.glueNodes.size() && st.glueMaterialCompounds == st.compounds.size() )
		return;
	st.glueMaterialKnown = true;
	st.glueMaterialSignature = st.glueSignature;
	st.glueMaterialNodes = st.glueNodes.size();
	st.glueMaterialCompounds = st.compounds.size();
	if ( st.glueNodes.empty() && st.glueMaterialKeys.empty() )
		return;
	refreshGlueGroups( st );
	b3dop::Box3DWorld& w = st.world;

	struct Sum
	{
		double mass = 0.0, friction = 0.0, restitution = 0.0;
	};
	int groups = 0;
	for ( const int g : st.glueGroup )
		groups = std::max( groups, g + 1 );
	std::vector<Sum> sums( static_cast<size_t>( groups ) );
	std::vector<Box3DBodyMirror*> entries( st.glueNodes.size(), nullptr );
	for ( size_t i = 0; i < st.glueNodes.size(); ++i )
	{
		const b3dop::BodyHandle h = st.glueNodes[i].body;
		auto* e = static_cast<Box3DBodyMirror*>( w.bodyUserDataByHandle( h ) );
		const int g = i < st.glueGroup.size() ? st.glueGroup[i] : -1;
		// Static and kinematic pieces keep their own material, and add nothing.
		if ( !e || e->handle != h || !e->dynamic() || g < 0 )
			continue;
		const double m = w.massByHandle( h );
		if ( !( m > 0.0 ) )
			continue;
		entries[i] = e;
		Sum& s = sums[static_cast<size_t>( g )];
		s.mass += m;
		s.friction += m * e->props.friction;
		s.restitution += m * e->props.restitution;
	}
	auto average = [&sums]( int g, float& friction, float& restitution ) {
		if ( g < 0 || static_cast<size_t>( g ) >= sums.size() )
			return false;
		const Sum& s = sums[static_cast<size_t>( g )];
		if ( !( s.mass > 0.0 ) )
			return false;
		friction = static_cast<float>( s.friction / s.mass );
		restitution = static_cast<float>( s.restitution / s.mass );
		return true;
	};

	std::vector<uint64_t> keys;
	keys.reserve( st.glueNodes.size() );
	float friction = 0.0f, restitution = 0.0f;
	for ( size_t i = 0; i < st.glueNodes.size(); ++i )
	{
		Box3DBodyMirror* e = entries[i];
		if ( !e || !average( st.glueGroup[i], friction, restitution ) )
			continue;
		w.setMaterialByHandle( e->handle, friction, restitution );
		keys.push_back( b3dop::makeBodyKey( e->objectId, e->pointId ) );
	}
	for ( const auto& c : st.compounds )
		if ( average( compoundGlueGroup( st, *c ), friction, restitution ) )
			w.setMaterialByHandle( c->body, friction, restitution );

	// The pieces given a group's material last time and not now: their own.
	std::sort( keys.begin(), keys.end() );
	for ( const uint64_t key : st.glueMaterialKeys )
	{
		if ( std::binary_search( keys.begin(), keys.end(), key ) )
			continue;
		const Box3DBodyMirror* e = st.mirror.find( key );
		if ( e && e->handle && st.isLive( *e ) )
			w.setMaterialByHandle( e->handle, e->props.friction, e->props.restitution );
	}
	st.glueMaterialKeys = std::move( keys );
}
