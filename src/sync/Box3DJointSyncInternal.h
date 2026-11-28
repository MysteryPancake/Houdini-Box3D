// Box3DJointSyncInternal.h
//
// What the Box3DJointSync*.cpp files share (the constraint network pass,
// reading a network's settings and SideFX data, finding anchors): helpers that
// used to be local to one file. Only those files include it.
#pragma once

#include "Box3DHoudiniUtil.h"
#include "Box3DJointNetwork.h"
#include "Box3DJointSync.h"
#include "core/Box3DDebug.h"
#include "dop/SIM_Box3DConRel.h"
#include "dop/SIM_Box3DWorld.h"

#include <GA/GA_Handle.h>
#include <GA/GA_Iterator.h>
#include <GA/GA_Primitive.h>
#include <GU/GU_Detail.h>
#include <GU/GU_DetailHandle.h>
#include <GU/GU_PackedFragment.h>
#include <GU/GU_PrimPacked.h>
#include <SIM/SIM_ConstraintNetworkRelationship.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Relationship.h>
#include <SIM/SIM_RelationshipArray.h>
#include <SIM/SIM_Utils.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_String.h>
#include <UT/UT_StringHolder.h>
#include <UT/UT_Vector3.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace jointsync
{

// More attribute names are in Box3DJointNetwork.h (shared with the
// after-step code).
inline const char* const kConstraintType = "constraint_type"; // SideFX data only

// With Attach Internal Constraints to Object, the network is this data on the
// object itself.
inline const char* const kInternalGeometry = "ConstraintGeometry";

// The Constraint Network's anchor attributes: anchor_id (point attribute, int)
// binds an anchor to a point (or vertex) of the named object; anchor_type
// (string "point" / "vertex" / "agent", or an int 0 / 1 / 2) says which.
inline const char* const kAnchorId = "anchor_id";

inline const char* const kAnchorType = "anchor_type";

// The Constraint Network data a relationship carries, or null for other
// relationships.
const SIM_ConstraintNetworkRelationship* networkData( const SIM_Relationship* rel );

// The Box3D constraint relationships a network's `constraint_name` can refer
// to, by data name. The Constraint Network DOP attaches the nodes wired into
// its second input under the network's relationship; the relationship, its
// Constraint Network data and the geometry are all searched (the first name
// found wins), and for an internal network the geometry and the object.
// SideFX's Glue / Hard / Spring / Bullet Soft / Cone Twist data count too
// (mapped per primitive).
std::unordered_map<std::string, Box3DJointKind> networkKinds( const SIM_Object& obj, const NetworkSource& src );

bool sameKinds( const std::unordered_map<std::string, Box3DJointKind>& a,
				const std::unordered_map<std::string, Box3DJointKind>& b );

// A resolved anchor: its body (0: the world) and, for a point body, its mirror
// entry (the rest pose).
struct AnchorBody
{
	b3dop::BodyHandle handle = 0;
	const Box3DBodyMirror* entry = nullptr;
	// The DOP object of a body in a solved object (an anchor_id names a point of
	// a whole object's geometry, or of a packed piece's own), else null.
	const SIM_Object* object = nullptr;
	// Found as an object with a single body: adding or removing a body of that
	// object changes it.
	bool byObjectName = false;
	// An agent anchor on a mirrored rig transform (Box3DAgentFrame): directions
	// in the transform's space have their z negated.
	bool mirrored = false;
};

// An anchor's anchor_type.
enum class AnchorKind
{
	Point,
	Vertex,
	Agent
};

// World position of point (or vertex) `id` of the object's geometry through its
// geometry transform. False if the object has no geometry or no such point.
bool pointAnchorWorld( const SIM_Object& obj, int id, bool vertex, double out[3] );

// Why an anchor_id found no point.
enum class AnchorLookup
{
	Ok,
	NoPoint,
	NoGeometry
};

// World position of point (or vertex) `id` of a packed piece's own geometry (a
// fragment's own primitives of the shared source), now: the packed point's
// position plus the geometry point from the pivot through the piece's matrix.
// `pointOff` is the piece's point in the object's geometry.
AnchorLookup packedAnchorWorld( const SIM_Object& obj, int64_t pointOff, int id, bool vertex, double out[3] );

// Anchor names -> bodies, over the objects of this solve (in their order) and
// the affectors. The name tables are built on the first resolve(): a step whose
// networks are all unchanged never resolves a name, so it pays nothing for
// them. After that a name is a hash lookup, not a scan of every object per
// anchor.
class Anchors
{
public:
	Anchors( const Box3DSharedState& st, SIM_ObjectArray& objects )
		: myState( st )
	{
		for ( int i = 0; i < objects.entries(); ++i )
		{
			const SIM_Object* obj = objects( i );
			const auto it = st.objectSync.find( obj->getObjectId() );
			if ( it == st.objectSync.end() || it->second.visited != st.syncStamp )
				continue;
			const UT_String& nm = obj->getName();
			myObjects.push_back( { nm.isstring() ? std::string( nm.buffer() ) : std::string(), &it->second, obj } );
		}
	}

	// "<agent>/<rig transform>" of a ragdoll agent whose transform has no body on
	// or above it, or of an agent that is not a ragdoll
	// (Box3DObjectSync::agentWithoutBody, agentsNotRagdoll). `expected` is set
	// when the agent left the ragdoll state (agentsLeftRagdoll): nothing to warn
	// about.
	bool agentWithoutBody( const std::string& name, bool& expected ) const
	{
		expected = false;
		// A ragdoll's transform with a body in any object is not (an agent of the
		// same name elsewhere being animated does not hide it).
		for ( const ObjectRef& o : myObjects )
			if ( o.sync->agentByName.count( name ) )
				return false;
		const size_t slash = name.find( '/' );
		const std::string agent = slash != std::string::npos ? name.substr( 0, slash ) : name;
		for ( const ObjectRef& o : myObjects )
		{
			if ( o.sync->agentsNotRagdoll.count( agent ) )
			{
				expected = o.sync->agentsLeftRagdoll.count( agent ) > 0;
				return true;
			}
			if ( o.sync->agentWithoutBody.count( name ) )
				return true;
		}
		return false;
	}

	// "<agent>/..." of a ragdoll agent that left the crowd on this scan
	// (Box3DObjectSync::agentsGone).
	bool agentGone( const std::string& name ) const
	{
		const size_t slash = name.find( '/' );
		if ( slash == std::string::npos )
			return false;
		const std::string agent = name.substr( 0, slash );
		for ( const ObjectRef& o : myObjects )
			if ( o.sync->agentsGone.count( agent ) )
				return true;
		return false;
	}

	// Empty: the world (0). Else a piece's `name` in any object, "object/piece",
	// an object with a single body, or an affector object. False if nothing
	// matches.
	bool resolve( const std::string& name, AnchorBody& out ) const
	{
		out = AnchorBody();
		if ( name.empty() )
			return true;
		buildTables();
		const size_t slash = name.find( '/' );
		if ( slash != std::string::npos )
		{
			// "<agent>/<rig transform>": the body of a crowd agent's transform.
			const auto agent = myAgentTable.find( name );
			if ( agent != myAgentTable.end() )
			{
				out = agent->second;
				return true;
			}
			// The first object with this name decides, whether or not it has the
			// piece.
			const auto o = myObjectByName.find( name.substr( 0, slash ) );
			return o != myObjectByName.end() && findIn( *o->second, name.substr( slash + 1 ), out );
		}
		// Order: a piece in any object, an object with one body, an affector.
		if ( myPieces )
		{
			const auto piece = myPieceTable.find( name );
			if ( piece != myPieceTable.end() )
			{
				out = piece->second;
				return true;
			}
		}
		else
		{
			for ( const ObjectRef& o : myObjects )
				if ( findIn( o, name, out ) )
					return true;
		}
		const auto single = mySingleBody.find( name );
		if ( single != mySingleBody.end() )
		{
			out = single->second;
			out.byObjectName = true;
			return true;
		}
		const auto affector = myAffector.find( name );
		if ( affector != myAffector.end() )
		{
			out = affector->second;
			return true;
		}
		return false;
	}

private:
	using HandleTable = std::unordered_map<std::string, AnchorBody>;

	// `object`: the DOP object the body is in (an anchor_id names a point of its
	// geometry).
	static AnchorBody bodyOf( const Box3DBodyMirror* e, const SIM_Object* object )
	{
		AnchorBody b;
		b.handle = e->handle;
		b.entry = e;
		b.object = object;
		return b;
	}

	struct ObjectRef
	{
		std::string name;
		const Box3DObjectSync* sync;
		const SIM_Object* object;
	};

	static bool findIn( const ObjectRef& o, const std::string& piece, AnchorBody& out )
	{
		const auto it = o.sync->byName.find( piece );
		if ( it == o.sync->byName.end() || !it->second->handle )
			return false;
		out = bodyOf( it->second, o.object );
		return true;
	}

	// Each table keeps the first match in object order (what a scan in that order
	// would find).
	void buildTables() const
	{
		if ( myBuilt )
			return;
		myBuilt = true;
		for ( const auto& o : myObjects )
		{
			myObjectByName.try_emplace( o.name, &o );
			for ( const auto& kv : o.sync->agentByName )
				if ( kv.second->handle )
					myAgentTable.try_emplace( kv.first, bodyOf( kv.second, o.object ) );
			if ( o.sync->bodies.size() == 1 && o.sync->bodies[0]->handle )
				mySingleBody.try_emplace( o.name, bodyOf( o.sync->bodies[0], o.object ) );
		}
		for ( const auto& kv : myState.affectors )
			if ( kv.second.handle )
			{
				AnchorBody b;
				b.handle = kv.second.handle;
				myAffector.try_emplace( kv.second.name, b );
			}
		// With many objects a scan per anchor costs more than one table of every
		// piece (the first object in order wins, as in the scan). The keys point
		// into the objects' own name maps, which do not change during a sync.
		if ( myObjects.size() > kPieceTableObjects )
		{
			myPieces = true;
			for ( const ObjectRef& o : myObjects )
				for ( const auto& kv : o.sync->byName )
					if ( kv.second->handle )
						myPieceTable.try_emplace( std::string_view( kv.first ), bodyOf( kv.second, o.object ) );
		}
	}

	static constexpr size_t kPieceTableObjects = 16;

	const Box3DSharedState& myState;
	std::vector<ObjectRef> myObjects;

	mutable bool myBuilt = false;
	mutable std::unordered_map<std::string, const ObjectRef*> myObjectByName;
	mutable HandleTable mySingleBody;
	mutable HandleTable myAffector;
	mutable HandleTable myAgentTable;
	mutable bool myPieces = false; // myPieceTable is in use
	mutable std::unordered_map<std::string_view, AnchorBody> myPieceTable;
};

// The primitive attributes that override a setting, bound once per network.
struct SettingReader
{
	const b3dop::JointParam* param = nullptr;
	GA_ROHandleI i;
	GA_ROHandleF f;
	GA_ROHandleV3 v;
};

std::vector<SettingReader> bindSettings( const GU_Detail* gdp );

// The primitive's attributes on top of its constraint data's settings, as the
// Constraint Network DOP documents: a float attribute multiplies the setting
// (angles too: it is a factor, not degrees), any other kind replaces it
// (vectors and Euler angles in Houdini units). With `replaceFloats` (the node's
// Attributes Replace Values) floats replace too, in Houdini units.
void applyOverrides( const std::vector<SettingReader>& readers, b3dop::JointType type, GA_Offset prim,
					 bool replaceFloats, b3dop::JointSettings& s );

// The primitive attributes that modify a SideFX data's options
// (b3dop::sideFXParams), bound once per network.
struct SideFXReader
{
	const b3dop::SideFXParam* param = nullptr;
	GA_ROHandleF f;
	GA_ROHandleI i;
	GA_ROHandleV3 v;
	GA_ROHandleV4 q;
};

std::vector<SideFXReader> bindSideFX( const GU_Detail* gdp );

// The Constraint Network DOP's rule on SideFX data too: floats multiply, the
// rest replace.
void applySideFXOverrides( const std::vector<SideFXReader>& readers, b3dop::SideFXType type, GA_Offset prim,
						   b3dop::SideFXSettings& s );

// A body as the SideFX mapping needs it: Spring converts its stiffness from the
// masses.
b3dop::SideFXBody sideFXBody( const b3dop::Box3DWorld& world, b3dop::BodyHandle h );
// The same with its inertia about `point` (world space), as a joint there turns it.
b3dop::SideFXBody sideFXBodyAbout( const b3dop::Box3DWorld& world, b3dop::BodyHandle h, const double point[3] );

// Two readings of a body are the same for the Spring conversion (the same mass
// and inertia, to a part in ten thousand).
bool sameSideFXBody( const b3dop::SideFXBody& a, const b3dop::SideFXBody& b );

// What decides where a SideFX joint's frames sit (not the anchors' positions:
// those are live).
uint64_t sideFXFrameSignature( const b3dop::SideFXJoint& m );

// A joint frame from its anchor and offset, and back. Identity offsets (Box3D
// nodes) pass the frame through untouched, bit for bit.
b3dop::JointFrame jointFrameOf( const b3dop::JointFrame& anchor, const b3dop::JointFrame& adj );

// A saved local frame still describes its anchor: through the body's pose now
// it lands on the anchor's P / orient (written together after the same step). A
// renamed, moved or new anchor does not, and is made from P / orient instead.
bool localFrameAgrees( const b3dop::Box3DWorld& world, b3dop::BodyHandle body, const b3dop::JointFrame& local,
					   const b3dop::JointFrame& live );

// An authored anchor frame (world space at rest) moved with its body since
// rest, as Bullet maps anchors through restxform: so a joint made after its
// pieces moved (a new primitive, a remake after an upstream edit) attaches
// where the anchor sits on the piece now. Unchanged for the world, affectors,
// whole objects and points without a rest; identity before anything moved.
b3dop::JointFrame restToNow( const b3dop::Box3DWorld& world, const AnchorBody& body, const b3dop::JointFrame& f );

// The inverse of restToNow for a position: where the world position `p` of a
// point on the piece now was at rest. Unchanged where restToNow leaves a frame
// alone.
void nowToRest( const b3dop::Box3DWorld& world, const AnchorBody& body, double p[3] );

// A body's rotation at rest for the `angle` reading: its restxform's, else its
// rotation now (when the joint is made), identity for the world.
void restRotationOf( const b3dop::Box3DWorld& world, const AnchorBody& body, float out[4] );

} // namespace jointsync
