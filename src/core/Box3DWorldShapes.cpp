// Box3DWorldShapes.cpp
//
// Box3DWorld's shapes: creating them on a body (box, sphere, capsule, cylinder,
// hull, mesh, height field), copying and destroying them, their outlines,
// filters and bounds.
#include "Box3DDebug.h"
#include "Box3DShapes.h"
#include "Box3DWorldHulls.h"
#include "Box3DWorldInternal.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace b3dop
{

namespace
{
// Scratch for a body's shapes (reused, so a call does not allocate).
thread_local std::vector<b3ShapeId> tShapes;
} // namespace

namespace
{

// Null, or a rotation within float noise of identity (qx, qy, qz ~ 0, w ~ +-1).
bool isIdentityRotation( const float q[4] )
{
	return !q || ( std::fabs( q[0] ) < 1e-6f && std::fabs( q[1] ) < 1e-6f && std::fabs( q[2] ) < 1e-6f );
}

b3Filter toB3Filter( const CollisionFilter& f )
{
	b3Filter out = b3DefaultFilter();
	out.categoryBits = f.category;
	out.maskBits = f.mask;
	out.groupIndex = f.group;
	return out;
}

b3ShapeDef shapeDefFor( const BodySettings& b, bool updateMass, float densityScale = 1.0f )
{
	const BodyProps& p = b.props;
	float tangent[3];
	b3ShapeDef def = b3DefaultShapeDef();
	def.updateBodyMass = updateMass; // false: the caller applies the mass afterwards
	def.density = p.density * cleanDensityScale( densityScale );
	def.userData = densityScaleTag( densityScale );
	def.baseMaterial.friction = p.friction;
	def.baseMaterial.restitution = p.restitution;
	def.baseMaterial.rollingResistance = nonNegOr( p.rollingResistance );
	cleanTangent( p.tangentVelocity, tangent );
	def.baseMaterial.tangentVelocity = makeVec3( tangent );
	def.filter = toB3Filter( p.filter );
	def.enableHitEvents = true; // for the Impacts speed filter; costs nothing without a threshold
	return def;
}

} // namespace

bool Box3DWorld::createShape( BodyHandle h, const BodySettings& body, const ShapeSpec& shape, bool updateMass,
							  float densityScale )
{
	switch ( shape.type )
	{
		case ShapeType::Sphere:
			return sphereShape( h, body, shape.radius, shape.center, updateMass, densityScale );
		case ShapeType::Capsule:
			return capsuleShape( h, body, shape.radius, shape.halfLength, shape.center, shape.rot, updateMass,
								 densityScale );
		case ShapeType::Cylinder:
			return cylinderShape( h, body, shape.radius, shape.halfLength, shape.center, shape.rot, updateMass,
								  densityScale );
		case ShapeType::Plane:
		{
			// A ground slab with its top through the body origin, facing its +Y
			// (static bodies only).
			if ( !h || b3Body_GetType( b3LoadBodyId( h ) ) != b3_staticBody )
				return false;
			// The spec's half extents are the ground's size (groundSpec); its top is
			// the body's y = 0.
			const float below[3] = { 0.0f, -shape.halfExtents[1], 0.0f };
			return boxShape( h, body, shape.halfExtents, below, nullptr, updateMass );
		}
		default:
			return boxShape( h, body, shape.halfExtents, shape.center, shape.rot, updateMass, densityScale );
	}
}

bool Box3DWorld::createHullShape( BodyHandle h, const BodySettings& body, HullId hullId, bool updateMass,
								  float densityScale )
{
	const HullRef hull = h ? myImpl->heldHull( hullId ) : nullptr;
	if ( !hull )
		return false;
	const b3ShapeDef def = shapeDefFor( body, updateMass, densityScale );
	return !B3_IS_NULL( b3CreateHullShape( b3LoadBodyId( h ), &def, hull->hull ) ); // added to the world's database
}

int Box3DWorld::copyShapesByHandle( BodyHandle src, BodyHandle dst, const float center[3], const float rotXYZW[4] )
{
	if ( !src || !dst || src == dst || !center || !rotXYZW || !finite3( center ) || !finite3( rotXYZW ) )
		return -1;
	const b3BodyId from = b3LoadBodyId( src ), to = b3LoadBodyId( dst );
	std::vector<b3ShapeId> shapes;
	const int got = bodyShapes( from, shapes );
	// Box3D lists shapes newest first: copy in creation order.
	std::reverse( shapes.begin(), shapes.begin() + got );
	for ( int i = 0; i < got; ++i )
	{
		const b3ShapeType type = b3Shape_GetType( shapes[static_cast<size_t>( i )] );
		if ( type != b3_hullShape && type != b3_sphereShape && type != b3_capsuleShape )
			return -1; // meshes, planes...: nothing is copied
	}
	float rot[4] = { rotXYZW[0], rotXYZW[1], rotXYZW[2], rotXYZW[3] };
	sanitizeQuat( rot ); // Box3D asserts on a non-unit quaternion
	b3Transform local;
	local.p = makeVec3( center );
	local.q = makeQuat( rot );
	int added = 0;
	for ( int i = 0; i < got; ++i )
	{
		const b3ShapeId s = shapes[static_cast<size_t>( i )];
		b3ShapeDef def = b3DefaultShapeDef();
		def.updateBodyMass = false; // the caller applies the mass once
		def.density = b3Shape_GetDensity( s );
		def.userData = b3Shape_GetUserData( s ); // its density scale (densityScaleTag)
		def.baseMaterial = b3Shape_GetSurfaceMaterial( s );
		def.filter = b3Shape_GetFilter( s );
		def.enableHitEvents = true;
		b3ShapeId made = b3_nullShapeId;
		switch ( b3Shape_GetType( s ) )
		{
			case b3_hullShape:
				made = b3CreateTransformedHullShape( to, &def, b3Shape_GetHull( s ), local, b3Vec3_one );
				break;
			case b3_sphereShape:
			{
				b3Sphere sphere = b3Shape_GetSphere( s );
				sphere.center = b3Add( local.p, b3RotateVector( local.q, sphere.center ) );
				made = b3CreateSphereShape( to, &def, &sphere );
				break;
			}
			default:
			{
				b3Capsule capsule = b3Shape_GetCapsule( s );
				capsule.center1 = b3Add( local.p, b3RotateVector( local.q, capsule.center1 ) );
				capsule.center2 = b3Add( local.p, b3RotateVector( local.q, capsule.center2 ) );
				made = b3CreateCapsuleShape( to, &def, &capsule );
				break;
			}
		}
		if ( B3_IS_NULL( made ) )
			return -1; // the caller drops the partly built body
		++added;
	}
	return added;
}

bool Box3DWorld::createMeshShape( BodyHandle h, const BodySettings& body, const float* xyz, int vertexCount,
								  const int32_t* triangles, int triangleCount, float weldTolerance,
								  const float scale[3] )
{
	if ( !h || !xyz || !triangles || vertexCount < 3 || triangleCount < 1 )
		return false;
	// Static or kinematic bodies only (a triangle mesh has no volume or mass):
	// refuse a dynamic one.
	if ( b3Body_GetType( b3LoadBodyId( h ) ) == b3_dynamicBody )
		return false;

	// Reject bad input here: Box3D asserts on non-finite vertices or out-of-range
	// indices.
	const size_t floatCount = static_cast<size_t>( vertexCount ) * 3;
	const size_t indexCount = static_cast<size_t>( triangleCount ) * 3;
	for ( int i = 0; i < vertexCount; ++i )
		if ( !finite3( xyz + 3 * i ) )
			return false;
	for ( size_t i = 0; i < indexCount; ++i )
		if ( triangles[i] < 0 || triangles[i] >= vertexCount )
			return false;
	if ( scale && ( !finite3( scale ) || !( scale[0] > 0.0f ) || !( scale[1] > 0.0f ) || !( scale[2] > 0.0f ) ) )
		return false;

	// The same mesh (vertices, indices, weld) on another body shares one
	// b3MeshData: Box3D only points at it, and the scale is the shape's own. A
	// hash match is checked against the mesh's own copy of what it was built
	// from; a different mesh with the same hash takes the next free key.
	const float weld = weldTolerance > 0.0f ? weldTolerance : 0.0f;
	uint64_t key = hashFloats( xyz, floatCount );
	key = hashBytes( key, triangles, indexCount * sizeof( int32_t ) );
	key = mixHash( key, static_cast<uint64_t>( std::hash<float>()( weld ) ) );
	Impl::SharedMesh* shared = nullptr;
	for ( ;; ++key )
	{
		const auto it = myImpl->sharedMeshes.find( key );
		if ( it == myImpl->sharedMeshes.end() )
			break;
		const Impl::SharedMesh& m = it->second;
		if ( m.weld == weld && m.xyz.size() == floatCount && m.tris.size() == indexCount &&
			 std::memcmp( m.xyz.data(), xyz, floatCount * sizeof( float ) ) == 0 &&
			 std::memcmp( m.tris.data(), triangles, indexCount * sizeof( int32_t ) ) == 0 )
		{
			shared = &it->second;
			break;
		}
	}

	std::vector<b3Vec3> verts; // only for a mesh not built yet
	std::vector<int32_t> idx;
	b3MeshDef def = {};
	if ( !shared )
	{
		verts.resize( static_cast<size_t>( vertexCount ) );
		for ( int i = 0; i < vertexCount; ++i )
			verts[static_cast<size_t>( i )] = makeVec3( xyz + 3 * i );
		idx.assign( triangles, triangles + indexCount ); // b3MeshDef wants non-const
	}
	def.vertices = verts.data();
	def.stride = 0;
	def.indices = idx.data();
	def.materialIndices = nullptr;
	def.vertexCount = vertexCount;
	def.triangleCount = triangleCount;
	def.weldVertices = weldTolerance > 0.0f;
	def.weldTolerance = weldTolerance > 0.0f ? weldTolerance : 0.0f;
	def.identifyEdges = true; // smooth rolling over triangle edges (less ghost collisions)

	b3MeshData* mesh = shared ? shared->data : b3CreateMesh( &def, nullptr, 0 );
	if ( !mesh )
		return false;

	const b3ShapeDef sdef = shapeDefFor( body, false );
	const b3ShapeId shape = b3CreateMeshShape( b3LoadBodyId( h ), &sdef, mesh, scale ? makeVec3( scale ) : b3Vec3_one );
	if ( B3_IS_NULL( shape ) )
	{
		if ( !shared )
			b3DestroyMesh( mesh );
		return false;
	}
	if ( !shared )
	{
		shared = &myImpl->sharedMeshes[key];
		shared->xyz.assign( xyz, xyz + floatCount );
		shared->tris.assign( triangles, triangles + indexCount );
		shared->weld = weld;
	}
	shared->data = mesh;
	++shared->users;
	myImpl->bodyGeometry[h].meshKeys.push_back( key );
	return true;
}

bool Box3DWorld::createHeightFieldShape( BodyHandle h, const BodySettings& body, const float* heights, int countX,
										 int countZ, const float scale[3], const uint8_t* holes )
{
	if ( !h || !heights || !scale || countX < 2 || countZ < 2 )
		return false;
	// Box3D indexes with int and asserts on these: refuse instead.
	if ( static_cast<int64_t>( countX ) * countZ > ( int64_t{ 1 } << 28 ) )
		return false;
	if ( !finite3( scale ) || !( scale[0] > 0.0f ) || !( scale[1] > 0.0f ) || !( scale[2] > 0.0f ) )
		return false;
	if ( b3Body_GetType( b3LoadBodyId( h ) ) != b3_staticBody )
		return false;

	const size_t count = static_cast<size_t>( countX ) * static_cast<size_t>( countZ );
	float lo = heights[0], hi = heights[0];
	for ( size_t i = 0; i < count; ++i )
	{
		const float v = heights[i];
		if ( !std::isfinite( v ) )
			return false;
		lo = std::min( lo, v );
		hi = std::max( hi, v );
	}

	const size_t cells = static_cast<size_t>( countX - 1 ) * static_cast<size_t>( countZ - 1 );
	std::vector<uint8_t> material( cells, 0 );
	if ( holes )
		for ( size_t i = 0; i < cells; ++i )
			if ( holes[i] )
				material[i] = B3_HEIGHT_FIELD_HOLE;

	b3HeightFieldDef def = {};
	// Box3D takes float* but only reads the heights (it quantises them into its
	// own data): no copy of a field that can be a gigabyte.
	def.heights = const_cast<float*>( heights );
	def.materialIndices = material.data();
	def.scale = makeVec3( scale );
	def.countX = countX;
	def.countZ = countZ;
	// Quantised to 16 bits between these (unscaled): the field's own range.
	def.globalMinimumHeight = lo;
	def.globalMaximumHeight = hi;
	def.clockwiseWinding = false; // triangles face +y

	b3HeightFieldData* hf = b3CreateHeightField( &def );
	if ( !hf )
		return false;
	const b3ShapeDef sdef = shapeDefFor( body, false );
	const b3ShapeId shape = b3CreateHeightFieldShape( b3LoadBodyId( h ), &sdef, hf );
	if ( B3_IS_NULL( shape ) )
	{
		b3DestroyHeightField( hf );
		return false;
	}
	myImpl->bodyGeometry[h].heightFields.push_back( hf );
	return true;
}

void Box3DWorld::applyMassFromShapes( BodyHandle h )
{
	if ( h )
		b3Body_ApplyMassFromShapes( b3LoadBodyId( h ) );
}

bool Box3DWorld::boxShape( BodyHandle h, const BodySettings& b, const float halfExtents[3], const float center[3],
						   const float rotationXYZW[4], bool updateMass, float densityScale )
{
	if ( !h )
		return false;

	const bool offset = center && ( center[0] != 0.0f || center[1] != 0.0f || center[2] != 0.0f );
	b3BoxHull hull;
	if ( !isIdentityRotation( rotationXYZW ) )
	{
		b3Transform local;
		local.p = makeVec3( center );
		local.q = cleanQuat( rotationXYZW );
		hull = b3MakeTransformedBoxHull( halfExtents[0], halfExtents[1], halfExtents[2], local );
	}
	else if ( offset )
	{
		hull = b3MakeOffsetBoxHull( halfExtents[0], halfExtents[1], halfExtents[2], makeVec3( center ) );
	}
	else
	{
		hull = b3MakeBoxHull( halfExtents[0], halfExtents[1], halfExtents[2] );
	}
	const b3ShapeDef def = shapeDefFor( b, updateMass, densityScale );
	const b3ShapeId shape = b3CreateHullShape( b3LoadBodyId( h ), &def, &hull.base );
	return !B3_IS_NULL( shape );
}

bool Box3DWorld::sphereShape( BodyHandle h, const BodySettings& b, float radius, const float center[3], bool updateMass,
							  float densityScale )
{
	if ( !h )
		return false;

	b3Sphere sphere;
	sphere.center = makeVec3( center );
	sphere.radius = radius;
	const b3ShapeDef def = shapeDefFor( b, updateMass, densityScale );
	const b3ShapeId shape = b3CreateSphereShape( b3LoadBodyId( h ), &def, &sphere );
	return !B3_IS_NULL( shape );
}

bool Box3DWorld::capsuleShape( BodyHandle h, const BodySettings& b, float radius, float halfLength,
							   const float center[3], const float rotationXYZW[4], bool updateMass, float densityScale )
{
	if ( !h )
		return false;

	const float c[3] = { center ? center[0] : 0.0f, center ? center[1] : 0.0f, center ? center[2] : 0.0f };
	// The axis is the shape's Y, turned by the shape rotation.
	float axis[3] = { 0.0f, 1.0f, 0.0f };
	if ( !isIdentityRotation( rotationXYZW ) )
	{
		const b3Vec3 up = { 0.0f, 1.0f, 0.0f };
		const b3Vec3 r = b3RotateVector( cleanQuat( rotationXYZW ), up );
		axis[0] = r.x;
		axis[1] = r.y;
		axis[2] = r.z;
	}
	b3Capsule capsule;
	capsule.center1.x = c[0] - halfLength * axis[0];
	capsule.center1.y = c[1] - halfLength * axis[1];
	capsule.center1.z = c[2] - halfLength * axis[2];
	capsule.center2.x = c[0] + halfLength * axis[0];
	capsule.center2.y = c[1] + halfLength * axis[1];
	capsule.center2.z = c[2] + halfLength * axis[2];
	capsule.radius = radius;
	// Box3D turns a capsule that is too short into a sphere by itself.
	const b3ShapeDef def = shapeDefFor( b, updateMass, densityScale );
	const b3ShapeId shape = b3CreateCapsuleShape( b3LoadBodyId( h ), &def, &capsule );
	return !B3_IS_NULL( shape );
}

bool Box3DWorld::cylinderShape( BodyHandle h, const BodySettings& b, float radius, float halfLength,
								const float center[3], const float rotationXYZW[4], bool updateMass,
								float densityScale )
{
	if ( !h || !std::isfinite( radius ) || !std::isfinite( halfLength ) )
		return false;
	// b3CreateCylinder asserts on a zero or negative size.
	radius = std::max( radius, kMinExtent );
	halfLength = std::max( halfLength, kMinExtent );
	const float c[3] = { center ? center[0] : 0.0f, center ? center[1] : 0.0f, center ? center[2] : 0.0f };
	const bool turned = !isIdentityRotation( rotationXYZW );
	const b3Quat q = turned ? cleanQuat( rotationXYZW ) : b3Quat_identity;

	// One key per size and pose, so instances share the hull.
	const float keyData[] = { radius, halfLength, c[0],	 c[1], c[2],
							  q.v.x,  q.v.y,	  q.v.z, q.s,  static_cast<float>( kCylinderSides ) };
	HullId key = hashFloats( keyData, sizeof( keyData ) / sizeof( keyData[0] ) );
	key = mixHash( key, 0xc711d3ull );
	if ( key == 0 )
		key = 1;

	if ( !findHull( key ) )
	{
		// The cylinder is centred on its own origin: b3CreateCylinder starts at
		// yOffset and rises by the height.
		b3HullData* hull = b3CreateCylinder( 2.0f * halfLength, radius, -halfLength, kCylinderSides );
		if ( !hull )
			return false;
		if ( turned || c[0] != 0.0f || c[1] != 0.0f || c[2] != 0.0f )
		{
			// Move the prism's points and hull them again.
			const b3Vec3* src = b3GetHullPoints( hull );
			std::vector<b3Vec3> pts( static_cast<size_t>( hull->vertexCount ) );
			for ( size_t i = 0; i < pts.size(); ++i )
			{
				const b3Vec3 r = b3RotateVector( q, src[i] );
				pts[i] = { r.x + c[0], r.y + c[1], r.z + c[2] };
			}
			b3DestroyHull( hull );
			hull = b3CreateHull( pts.data(), static_cast<int>( pts.size() ), B3_MAX_HULL_VERTICES );
			if ( !hull )
				return false;
		}
		const HullRef ref = HullLibrary::instance().insert( key, hull );
		myImpl->hold( key, ref );
	}
	return createHullShape( h, b, key, updateMass, densityScale );
}

namespace
{

// A mesh's edges worth drawing: boundary edges and ones where the two
// triangles meet at more than ~20 degrees (a flat quad's diagonal is left out).
// More than kMaxMeshOutlineEdges are thinned evenly.
void meshOutline( const b3Mesh& mesh, Box3DWorld::ShapeOutline& o )
{
	const b3Vec3* v = b3GetMeshVertices( mesh.data );
	const b3MeshTriangle* t = b3GetMeshTriangles( mesh.data );
	const int vc = mesh.data->vertexCount, tc = mesh.data->triangleCount;
	if ( !v || !t || vc < 3 || tc < 1 )
		return;
	o.kind = Box3DWorld::ShapeOutline::Mesh;
	o.verts.resize( static_cast<size_t>( vc ) * 3 );
	for ( int k = 0; k < vc; ++k )
	{
		o.verts[3 * static_cast<size_t>( k )] = v[k].x * mesh.scale.x;
		o.verts[3 * static_cast<size_t>( k ) + 1] = v[k].y * mesh.scale.y;
		o.verts[3 * static_cast<size_t>( k ) + 2] = v[k].z * mesh.scale.z;
	}
	struct Edge
	{
		uint64_t key;
		b3Vec3 normal;
		int count;
	};
	std::vector<Edge> edges;
	edges.reserve( static_cast<size_t>( tc ) * 3 );
	for ( int k = 0; k < tc; ++k )
	{
		const int idx[3] = { t[k].index1, t[k].index2, t[k].index3 };
		if ( idx[0] < 0 || idx[1] < 0 || idx[2] < 0 || idx[0] >= vc || idx[1] >= vc || idx[2] >= vc )
			continue;
		const float* p[3];
		for ( int j = 0; j < 3; ++j )
			p[j] = &o.verts[3 * static_cast<size_t>( idx[j] )];
		const float a[3] = { p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2] };
		const float b[3] = { p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2] };
		b3Vec3 n = { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
		const float len = std::sqrt( n.x * n.x + n.y * n.y + n.z * n.z );
		if ( len > 0.0f )
		{
			n.x /= len;
			n.y /= len;
			n.z /= len;
		}
		for ( int j = 0; j < 3; ++j )
		{
			const uint64_t lo = static_cast<uint64_t>( std::min( idx[j], idx[( j + 1 ) % 3] ) );
			const uint64_t hi = static_cast<uint64_t>( std::max( idx[j], idx[( j + 1 ) % 3] ) );
			edges.push_back( { ( lo << 32 ) | hi, n, 1 } );
		}
	}
	std::sort( edges.begin(), edges.end(), []( const Edge& x, const Edge& y ) { return x.key < y.key; } );
	std::vector<uint64_t> keep;
	constexpr float kCreaseDot = 0.94f; // cos 20 degrees
	for ( size_t k = 0; k < edges.size(); )
	{
		size_t e = k + 1;
		while ( e < edges.size() && edges[e].key == edges[k].key )
			++e;
		bool draw = e - k != 2; // boundary or non-manifold
		if ( !draw )
		{
			const b3Vec3 &x = edges[k].normal, &y = edges[k + 1].normal;
			draw = x.x * y.x + x.y * y.y + x.z * y.z < kCreaseDot;
		}
		if ( draw )
			keep.push_back( edges[k].key );
		k = e;
	}
	const size_t stride = keep.size() / kMaxMeshOutlineEdges + 1;
	for ( size_t k = 0; k < keep.size(); k += stride )
	{
		o.loops.push_back( static_cast<int>( keep[k] >> 32 ) );
		o.loops.push_back( static_cast<int>( keep[k] & 0xffffffffu ) );
	}
}

} // namespace

bool Box3DWorld::bodyShapeOutlines( BodyHandle h, std::vector<ShapeOutline>& out ) const
{
	out.clear();
	if ( !h )
		return false;
	const b3BodyId body = b3LoadBodyId( h );
	if ( !b3Body_IsValid( body ) )
		return false;
	std::vector<b3ShapeId> shapes;
	const int got = bodyShapes( body, shapes );
	auto put3 = []( float* dst, b3Vec3 v ) {
		dst[0] = static_cast<float>( v.x );
		dst[1] = static_cast<float>( v.y );
		dst[2] = static_cast<float>( v.z );
	};
	for ( int i = 0; i < got; ++i )
	{
		ShapeOutline o;
		switch ( b3Shape_GetType( shapes[static_cast<size_t>( i )] ) )
		{
			case b3_sphereShape:
			{
				const b3Sphere sp = b3Shape_GetSphere( shapes[static_cast<size_t>( i )] );
				o.kind = ShapeOutline::Sphere;
				put3( o.c1, sp.center );
				o.radius = sp.radius;
				break;
			}
			case b3_capsuleShape:
			{
				const b3Capsule c = b3Shape_GetCapsule( shapes[static_cast<size_t>( i )] );
				o.kind = ShapeOutline::Capsule;
				put3( o.c1, c.center1 );
				put3( o.c2, c.center2 );
				o.radius = c.radius;
				break;
			}
			case b3_hullShape:
			{
				const b3HullData* hull = b3Shape_GetHull( shapes[static_cast<size_t>( i )] );
				const b3Vec3* pts = hull ? b3GetHullPoints( hull ) : nullptr;
				const b3HullFace* faces = hull ? b3GetHullFaces( hull ) : nullptr;
				const b3HullHalfEdge* edges = hull ? b3GetHullEdges( hull ) : nullptr;
				if ( !pts || !faces || !edges )
					continue;
				o.kind = ShapeOutline::Hull;
				o.verts.resize( static_cast<size_t>( hull->vertexCount ) * 3 );
				for ( int v = 0; v < hull->vertexCount; ++v )
					put3( &o.verts[static_cast<size_t>( v ) * 3], pts[v] );
				for ( int f = 0; f < hull->faceCount; ++f )
				{
					const size_t countAt = o.loops.size();
					o.loops.push_back( 0 );
					const int first = faces[f].edge;
					int e = first;
					for ( int guard = 0; guard < hull->edgeCount; ++guard )
					{
						o.loops.push_back( edges[e].origin );
						++o.loops[countAt];
						e = edges[e].next;
						if ( e == first )
							break;
					}
				}
				break;
			}
			case b3_meshShape:
			{
				const b3Mesh mesh = b3Shape_GetMesh( shapes[static_cast<size_t>( i )] );
				if ( !mesh.data )
					continue;
				meshOutline( mesh, o );
				break;
			}
			default:
				continue; // compounds: not made for bodies here
		}
		out.push_back( std::move( o ) );
	}
	return true;
}

void Box3DWorld::setMeshScaleByHandle( BodyHandle h, const float scale[3] )
{
	if ( !h || !scale || !finite3( scale ) )
		return;
	const b3Vec3 sc = makeVec3( scale );
	const int n = bodyShapes( b3LoadBodyId( h ), tShapes );
	for ( int i = 0; i < n; ++i )
	{
		const b3ShapeId sid = tShapes[static_cast<size_t>( i )];
		if ( b3Shape_GetType( sid ) != b3_meshShape )
			continue;
		const b3Mesh mesh = b3Shape_GetMesh( sid );
		if ( mesh.scale.x != sc.x || mesh.scale.y != sc.y || mesh.scale.z != sc.z )
			b3Shape_SetMesh( sid, mesh.data, sc );
	}
}

void Box3DWorld::destroyShapesByHandle( BodyHandle h )
{
	if ( !h )
		return;
	const b3BodyId id = b3LoadBodyId( h );
	const int n = bodyShapes( id, tShapes );
	for ( int i = 0; i < n; ++i )
		b3DestroyShape( tShapes[static_cast<size_t>( i )],
						false ); // the caller applies the mass afterwards
	myImpl->releaseBodyResources( h );
}

int Box3DWorld::destroyShapeRangesByHandle( BodyHandle h, const std::vector<std::pair<int, int>>& ranges )
{
	if ( !h )
		return 0;
	const b3BodyId id = b3LoadBodyId( h );
	const int n = bodyShapes( id, tShapes ); // newest first
	std::vector<int> doomed;				 // creation indices, each once (ranges may overlap)
	for ( const auto& r : ranges )
		for ( int i = r.first; i < r.first + r.second; ++i )
			if ( i >= 0 && i < n )
				doomed.push_back( i );
	std::sort( doomed.begin(), doomed.end() );
	doomed.erase( std::unique( doomed.begin(), doomed.end() ), doomed.end() );
	// Collect the ids before destroying any: tShapes is not touched by it, but
	// the list is read newest first.
	std::vector<b3ShapeId> ids;
	ids.reserve( doomed.size() );
	for ( const int i : doomed )
		ids.push_back( tShapes[static_cast<size_t>( n - 1 - i )] );
	for ( const b3ShapeId s : ids )
		b3DestroyShape( s, false ); // the caller applies the mass afterwards
	return static_cast<int>( ids.size() );
}

namespace
{

// A convex hull that is a box: six faces of four vertices, eight vertices, the
// faces' normals along three perpendicular axes.
bool isBoxOutline( const Box3DWorld::ShapeOutline& o )
{
	if ( o.kind != Box3DWorld::ShapeOutline::Hull || o.verts.size() != 8 * 3 )
		return false;
	float normals[6][3];
	int faces = 0;
	for ( size_t at = 0; at < o.loops.size(); )
	{
		const int n = o.loops[at];
		if ( n != 4 || faces >= 6 || at + 1 + static_cast<size_t>( n ) > o.loops.size() )
			return false;
		const float* p[3];
		for ( int k = 0; k < 3; ++k )
			p[k] = &o.verts[static_cast<size_t>( o.loops[at + 1 + static_cast<size_t>( k )] ) * 3];
		const float a[3] = { p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2] };
		const float b[3] = { p[2][0] - p[1][0], p[2][1] - p[1][1], p[2][2] - p[1][2] };
		cross3( a, b, normals[faces] );
		const float len = length3( normals[faces] );
		if ( !( len > 0.0f ) )
			return false;
		for ( float& c : normals[faces] )
			c /= len;
		++faces;
		at += 5;
	}
	if ( faces != 6 )
		return false;
	for ( int i = 0; i < 6; ++i )
		for ( int j = i + 1; j < 6; ++j )
		{
			const float d = std::fabs( dot3( normals[i], normals[j] ) );
			if ( d > 1e-3f && d < 1.0f - 1e-3f )
				return false;
		}
	return true;
}

} // namespace

bool Box3DWorld::boundsInertiaExtentsByHandle( BodyHandle h, const float* restRot, float pad, float extents[3] ) const
{
	std::vector<ShapeOutline> shapes;
	if ( !bodyShapeOutlines( h, shapes ) || shapes.empty() )
		return false;
	if ( shapes.size() == 1 && ( shapes[0].kind != ShapeOutline::Hull || isBoxOutline( shapes[0] ) ) )
		return false; // one sphere, capsule or box: its own inertia is exact
	float lo[3] = { FLT_MAX, FLT_MAX, FLT_MAX }, hi[3] = { -FLT_MAX, -FLT_MAX, -FLT_MAX };
	const auto add = [&]( const float p[3], float radius ) {
		float q[3] = { p[0], p[1], p[2] };
		if ( restRot )
			quatRotate( restRot, p, q );
		for ( int k = 0; k < 3; ++k )
		{
			lo[k] = std::min( lo[k], q[k] - radius );
			hi[k] = std::max( hi[k], q[k] + radius );
		}
	};
	for ( const ShapeOutline& o : shapes )
	{
		switch ( o.kind )
		{
			case ShapeOutline::Hull:
				for ( size_t i = 0; i + 2 < o.verts.size(); i += 3 )
					add( &o.verts[i], 0.0f );
				break;
			case ShapeOutline::Sphere:
				add( o.c1, o.radius );
				break;
			case ShapeOutline::Capsule:
				add( o.c1, o.radius );
				add( o.c2, o.radius );
				break;
			case ShapeOutline::Mesh:
				return false;
		}
	}
	for ( int k = 0; k < 3; ++k )
	{
		extents[k] = hi[k] - lo[k] + pad;
		if ( !( extents[k] > 0.0f ) || !std::isfinite( extents[k] ) )
			return false;
	}
	return true;
}

bool Box3DWorld::extentByHandle( BodyHandle h, float out[3] ) const
{
	if ( !h || b3Body_GetShapeCount( b3LoadBodyId( h ) ) == 0 )
		return false;
	freshenPose( h );
	const b3AABB box = b3Body_ComputeAABB( b3LoadBodyId( h ) );
	const b3Vec3 e = b3Sub( box.upperBound, box.lowerBound );
	out[0] = e.x;
	out[1] = e.y;
	out[2] = e.z;
	return finite3( out ) && out[0] >= 0.0f && out[1] >= 0.0f && out[2] >= 0.0f;
}

bool Box3DWorld::boundsByHandle( BodyHandle h, double lo[3], double hi[3] ) const
{
	if ( !h )
		return false;
	const b3BodyId id = b3LoadBodyId( h );
	if ( !b3Body_IsValid( id ) || b3Body_GetShapeCount( id ) == 0 )
		return false;
	const b3AABB box = b3Body_ComputeAABB( id );
	lo[0] = box.lowerBound.x;
	lo[1] = box.lowerBound.y;
	lo[2] = box.lowerBound.z;
	hi[0] = box.upperBound.x;
	hi[1] = box.upperBound.y;
	hi[2] = box.upperBound.z;
	return true;
}

void Box3DWorld::setFilterByHandle( BodyHandle h, const CollisionFilter& f )
{
	if ( !h )
		return;
	const b3Filter bf = toB3Filter( f );
	const int n = bodyShapes( b3LoadBodyId( h ), tShapes );
	for ( int i = 0; i < n; ++i )
		b3Shape_SetFilter( tShapes[static_cast<size_t>( i )], bf,
						   true ); // no-op when unchanged
}

int Box3DWorld::shapeCountByHandle( BodyHandle h ) const
{
	return h ? b3Body_GetShapeCount( b3LoadBodyId( h ) ) : 0;
}

} // namespace b3dop
