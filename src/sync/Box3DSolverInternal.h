// Box3DSolverInternal.h
//
// What the Box3D Solver's parts share (not a public interface: only the
// solver's own files include it). The parts, all in namespace box3dsolver:
//   Box3DShapeBuild.cpp   collision source geometry, packed primitives,
//   building shapes Box3DPointSync.cpp    one body per point: reading points,
//   settings, syncObject() Box3DWholeObject.cpp  one body per DOP object (RBD
//   Object etc.): sync and write-back Box3DPostStep.cpp     after the step:
//   write-back, impacts, point writes, aging, deletion Box3DForces.cpp Force
//   DOPs and point force / torque
// SIM_SolverBox3D.cpp holds the node and the solve loop.
#pragma once

#include "Box3DAttribNames.h"
#include "Box3DHoudiniUtil.h"
#include "core/Box3DDebug.h"
#include "dop/SIM_Box3DData.h"
#include "dop/SIM_Box3DWorld.h"

#include <GA/GA_AIFSharedStringTuple.h>
#include <GA/GA_Handle.h>
#include <GA/GA_Iterator.h>
#include <GA/GA_Primitive.h>
#include <GU/GU_Detail.h>
#include <GU/GU_DetailHandle.h>
#include <GU/GU_PackedFragment.h>
#include <GU/GU_PackedImpl.h>
#include <GU/GU_PrimPacked.h>
#include <SIM/SIM_ColliderLabel.h>
#include <SIM/SIM_Data.h>
#include <SIM/SIM_DataFilter.h>
#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Error.h>
#include <SIM/SIM_Force.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Impacts.h>
#include <SIM/SIM_Motion.h>
#include <SIM/SIM_Names.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Position.h>
#include <SIM/SIM_PositionSimple.h>
#include <SIM/SIM_Time.h>
#include <SIM/SIM_Utils.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_Matrix3.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_ParallelUtil.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_Vector3.h>
#include <UT/UT_XformOrder.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// File scope (not the anonymous namespace): the header forward-declares it.
// Escape during one solve. Like SideFX's own SIM code
// (SIM_VolumetricConnectedComponentBuilder), no operation of our own:
// opInterrupt() under the one Houdini already runs while stepping, so its
// "Stepping simulation" message and progress bar stay as they are (no percent
// passed). Checked only from serial code, between whole points (never inside a
// Box3D call).
struct StepProgress {
  UT_Interrupt *boss = nullptr;
  bool cancelled = false; // latched: once true, every check says so

  bool check() {
    if (!cancelled && boss)
      cancelled = boss->opInterrupt() != 0;
    return cancelled;
  }
};

namespace box3dsolver {

// Below these counts a pass runs serially (untuned starting points).
constexpr size_t kMinParallelPoints = 2048; // default
constexpr size_t kMinParallelReadPlain =
    8192; // sync read, no primitives in the geometry
constexpr size_t kMinParallelReadPacked =
    2048;                                  // sync read, geometry has primitives
constexpr size_t kMinParallelWrite = 1024; // write-back

// The thresholds in use. In BOX3DDOP_DEBUG builds an environment variable
// overrides each one (a decimal count, read once per process), so a benchmark
// can sweep them without rebuilding:
//   BOX3DDOP_MIN_PARALLEL_POINTS / _READ_PLAIN / _READ_PACKED / _WRITE
// Release builds always use the constants above.
inline size_t tunedMinParallel(const char *envName, size_t fallback) {
#ifdef BOX3DDOP_DEBUG
  if (const char *v = std::getenv(envName)) {
    char *end = nullptr;
    const unsigned long long n = std::strtoull(v, &end, 10);
    if (end != v)
      return static_cast<size_t>(n);
  }
#endif
  (void)envName;
  return fallback;
}
inline size_t minParallelPoints() {
  static const size_t v =
      tunedMinParallel("BOX3DDOP_MIN_PARALLEL_POINTS", kMinParallelPoints);
  return v;
}
inline size_t minParallelReadPlain() {
  static const size_t v = tunedMinParallel("BOX3DDOP_MIN_PARALLEL_READ_PLAIN",
                                           kMinParallelReadPlain);
  return v;
}
inline size_t minParallelReadPacked() {
  static const size_t v = tunedMinParallel("BOX3DDOP_MIN_PARALLEL_READ_PACKED",
                                           kMinParallelReadPacked);
  return v;
}
inline size_t minParallelWrite() {
  static const size_t v =
      tunedMinParallel("BOX3DDOP_MIN_PARALLEL_WRITE", kMinParallelWrite);
  return v;
}

// f(i) for i in [0, n), on Houdini's TBB pool when n is large. f may only read
// shared data and write element i of its own output.
template <class F>
void parallelForEach(size_t n, const F &f,
                     size_t minParallel = minParallelPoints(),
                     size_t grain = 1) {
  if (n < minParallel) {
    for (size_t i = 0; i < n; ++i)
      f(i);
    return;
  }
  UTparallelFor(UT_BlockedRange<size_t>(0, n, grain),
                [&f](const UT_BlockedRange<size_t> &r) {
                  for (size_t i = r.begin(); i != r.end(); ++i)
                    f(i);
                });
}

// Point attributes the solver reads (Houdini RBD names). All optional: invalid
// = default.
struct PointAttrs {
  // One GA_ROHandle per entry of Box3DAttribNames.h (id, active, v, w, orient,
  // then the props and shape attributes); see that file for what each means.
  // Invalid = default.
#define BOX3D_ATTR_MEMBER(member, name, Tag) GA_ROHandle##Tag member;
  BOX3D_SYNC_ATTRS(BOX3D_ATTR_MEMBER)
  BOX3D_PROP_ATTRS(BOX3D_ATTR_MEMBER)
  BOX3D_SHAPE_ATTRS(BOX3D_ATTR_MEMBER)
#undef BOX3D_ATTR_MEMBER

  // The constructor binds what the skip check and write-back need;
  // bindBuildAttrs() the rest, once an object is actually scanned.
  explicit PointAttrs(const GU_Detail *gdp) {
#define BOX3D_ATTR_BIND(member, name, Tag)                                     \
  member = GA_ROHandle##Tag(gdp, GA_ATTRIB_POINT, name);
    BOX3D_SYNC_ATTRS(BOX3D_ATTR_BIND)
  }

  void bindBuildAttrs(const GU_Detail *gdp) {
    BOX3D_PROP_ATTRS(BOX3D_ATTR_BIND)
    BOX3D_SHAPE_ATTRS(BOX3D_ATTR_BIND)
  }
#undef BOX3D_ATTR_BIND
};

// ---- packed primitives (Box3DShapeBuild.cpp) ----

struct PackedInfo {
  const GU_PrimPacked *prim = nullptr; // null: a plain point
  GA_Offset primOff = GA_INVALID_OFFSET;
  double P[3] = {0, 0, 0}; // the pivot in world space
  double pivot[3] = {
      0, 0,
      0}; // in the packed geometry's space; filled by readPackedPivot only
  double m4[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  double m3[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // m4's upper 3x3
  float rot[4] = {0, 0, 0, 1};                // rotation split out of m4's 3x3
  double scale[3] = {1, 1, 1};
};

bool splitPackedMatrix(const double m3[9], float q[4], double scale[3]);
void composePackedMatrix(const float q[4], const double sc[3], double out[9]);
void fetchFull4(const GU_PrimPacked &p, double out[16]);
void setLocal3(GU_PrimPacked &p, const double m[9]);
PackedInfo getPackedInfo(const GU_Detail *gdp, GA_Offset ptoff);
void readPackedPivot(PackedInfo &pk);

// ---- collision source geometry and shapes (Box3DShapeBuild.cpp) ----

// One piece's points, raw xyz in their own space.
struct NamedPiece {
  std::string name;
  std::vector<GA_Offset>
      points; // while collecting (finishPieces reads and drops them)
  std::vector<float> xyz;
  uint64_t hash = 0; // of xyz
};

struct PieceGroup {
  std::vector<NamedPiece> pieces;
  std::unordered_map<std::string, size_t> byName;
  uint64_t hash = 0; // of all pieces: the merged hull's key

  NamedPiece &piece(const std::string &name) {
    auto it = byName.find(name);
    if (it == byName.end()) {
      it = byName.emplace(name, pieces.size()).first;
      pieces.emplace_back();
      pieces.back().name = name;
    }
    return pieces[it->second];
  }
};

// Some piece has points (a group whose first piece is empty is still usable).
inline bool anyPieceNonEmpty(const PieceGroup &g) {
  for (const NamedPiece &np : g.pieces)
    if (!np.xyz.empty())
      return true;
  return false;
}

void collectPieces(const GU_Detail *gdp, PieceGroup &out);
void collectConnectedPieces(const GU_Detail *gdp, const GA_Range &prims,
                            PieceGroup &out);

// A triangle mesh: xyz per vertex, three vertex indices per triangle. In a
// ShapeSources entry the source's own space and Houdini's winding; in a Piece
// body-local with Box3D's (counter-clockwise front faces).
struct MeshPiece {
  std::vector<float> xyz;
  std::vector<int32_t> tris;
};

struct SourceSummary;

// Collision source geometry (what is packed in each point's primitive), cached
// for one syncObject() call. Thread-safe: the shape pre-pass plans points in
// parallel.
struct ShapeSources {
  // Per distinct source detail; the handle keeps the detail (the key) alive.
  struct InnerEntry {
    GU_ConstDetailHandle handle;
    PieceGroup pieces;    // grouped by the `name` attribute
    PieceGroup connected; // one piece per set of connected primitives
    bool connectedBuilt = false;
  };
  std::mutex mutex;
  std::unordered_map<const GU_Detail *, InnerEntry> inner;
  // Fragments: (shared source detail, first primitive offset) -> that
  // fragment's piece.
  std::map<std::pair<const GU_Detail *, GA_Offset>, InnerEntry> fragments;

  // The primitive's source detail. A packed fragment shares the WHOLE source
  // detail (detailPtr) with every fragment of it and stands for its own
  // primitives in it. Not getPackedDetail(): for a fragment that is a different
  // detail from the one getPrimitiveRange() indexes, so the range's offsets
  // picked the wrong primitives or none (none: the pscale * scale box).
  static GU_ConstDetailHandle sourceHandle(const GU_PrimPacked &prim,
                                           const GU_PackedFragment *frag) {
    return frag ? frag->detailPtr() : prim.getPackedDetail();
  }
  // GU_PackedFragment does not expose its attribute value; its primitive range
  // does identify it (no two fragments of one source share a primitive).
  // GA_INVALID_OFFSET for a non-fragment.
  static GA_Offset fragmentFirst(const GU_PackedFragment *frag) {
    if (!frag)
      return GA_INVALID_OFFSET;
    const GA_Iterator first(frag->getPrimitiveRange());
    return first.atEnd() ? GA_INVALID_OFFSET : *first;
  }

  // The source's summary, if one was recorded for it (no geometry is read).
  static bool summary(const GU_PrimPacked &prim, bool connected,
                      SourceSummary &out);

  // The primitive's pieces (by `name`, or connected sets), or null. Records the
  // summary.
  const PieceGroup *innerPieces(const GU_PrimPacked &prim, bool connected);

  // The primitive's polygons as triangles (fan-triangulated, source space,
  // Houdini winding), or null if it has none. Per source detail / fragment.
  struct TriEntry {
    GU_ConstDetailHandle handle;
    std::shared_ptr<const MeshPiece> mesh;
  };
  std::map<std::pair<const GU_Detail *, GA_Offset>, TriEntry> triangles;
  std::shared_ptr<const MeshPiece> innerTriangles(const GU_PrimPacked &prim);
};

// One collision piece: a hull, a box / sphere / capsule, or (a static `concave`
// point) a triangle mesh.
struct Piece {
  bool isHull = false;
  b3dop::HullId hull = 0;
  b3dop::ShapeSpec spec;
  std::shared_ptr<const MeshPiece> mesh; // set: a mesh (static bodies only)
  float volume = 0.0f;
};

struct BuiltShapes {
  std::vector<Piece> pieces;
  float totalVolume = 0.0f;
  const char *note = nullptr; // set when something fell back
  bool mesh = false;          // the pieces are one triangle mesh
};

// staticMesh: the body will be static, so `concave` becomes a triangle mesh of
// the packed polygons (houdiniWinding: Houdini's clockwise front faces, i.e.
// Box3D Data's Flip Static Mesh Winding off). No polygons: hulls as usual.
BuiltShapes buildShapes(b3dop::Box3DWorld &world, ShapeSources &src,
                        b3dop::ShapeType type, bool hullPerConnected,
                        float pscale, const float scale[3],
                        const PackedInfo &pk, bool autofit,
                        const b3dop::PrimOverrides &ov, bool staticMesh = false,
                        bool houdiniWinding = true);
BuiltShapes buildWholeObjectShapes(b3dop::Box3DWorld &world,
                                   const GU_Detail *gdp, const double m[16],
                                   const double pivot[3], b3dop::ShapeType type,
                                   bool hullPerConnected);

// A point's `bullet_georep` via the shared-string handle (an int read), each
// distinct handle parsed once per scan (handles are per attribute table, so the
// map is per scan). Without the shared-string interface the string is parsed
// directly.
struct GeorepLookup {
  const GA_Attribute *attr = nullptr;
  const GA_AIFSharedStringTuple *aif = nullptr;
  struct Parsed {
    bool ok = false;
    b3dop::ShapeType type = b3dop::ShapeType::Box;
  };
  std::mutex mutex; // get() runs in the parallel shape pre-pass
  std::unordered_map<int64_t, Parsed> byHandle;

  void bind(const PointAttrs &a) {
    attr = a.georep.isValid() ? a.georep.getAttribute() : nullptr;
    aif = attr ? attr->getAIFSharedStringTuple() : nullptr;
  }

  // False (out untouched) when missing, empty or unknown.
  bool get(const PointAttrs &a, GA_Offset off, b3dop::ShapeType &out) {
    if (!attr)
      return false;
    if (!aif)
      return b3dop::parseShapeType(a.georep.get(off), out);
    const GA_StringIndexType handle = aif->getHandle(attr, off, 0);
    if (handle == GA_INVALID_STRING_INDEX)
      return false;
    std::lock_guard<std::mutex> lock(mutex);
    auto it = byHandle.find(static_cast<int64_t>(handle));
    if (it == byHandle.end()) {
      Parsed p;
      p.ok = b3dop::parseShapeType(aif->getTableString(attr, handle), p.type);
      it = byHandle.emplace(static_cast<int64_t>(handle), p).first;
    }
    if (it->second.ok)
      out = it->second.type;
    return it->second.ok;
  }
};

// ---- point state and body settings (Box3DPointSync.cpp) ----

bool readPointState(const GU_Detail *gdp, const PointAttrs &a, GA_Offset off,
                    const PackedInfo &pk, Box3DBodyPose &out,
                    bool *badVelocity = nullptr);
bool poseDiffers(const Box3DBodyPose &a, const Box3DBodyPose &b);
Box3DSyncFingerprint makeFingerprint(const GU_Detail *gdp, const PointAttrs &a,
                                     int nodeType,
                                     const b3dop::BodyProps &nodeProps,
                                     bool addImpact);
b3dop::BodyProps readDynamicProps(const b3dop::BodySettings &node,
                                  const PointAttrs &a, GA_Offset off);
float boxDensity(const b3dop::BodyProps &p, float volume);
float effectiveMass(const b3dop::BodyProps &p, float volume);
b3dop::BodySettings::Type wantedType(const b3dop::BodySettings &node,
                                     bool active, bool animated,
                                     bool shapeless);
void applyBodyMass(b3dop::Box3DWorld &world, b3dop::BodyHandle h,
                   const b3dop::BodyProps &p, float volume,
                   bool setShapeDensity = true);
bool startAwakeFor(const b3dop::BodySettings &node, const PointAttrs &a,
                   GA_Offset off);

// What every point of one syncObject() scan shares.
struct ScanContext {
  Box3DSharedState &st;
  b3dop::Box3DWorld &world;
  const GU_Detail *gdp;
  const PointAttrs &a;
  const b3dop::BodySettings &node;
  const bool anyPrims; // else no packed points
  ShapeSources sources;
  GeorepLookup georep;

  ScanContext(Box3DSharedState &state, const GU_Detail *g,
              const PointAttrs &attrs, const b3dop::BodySettings &n)
      : st(state), world(state.world), gdp(g), a(attrs), node(n),
        anyPrims(g->getNumPrimitives() > 0) {
    georep.bind(a);
  }

  // The point's packed primitive with its pivot (what building shapes needs).
  PackedInfo packedInfo(GA_Offset off) const {
    PackedInfo pk = anyPrims ? getPackedInfo(gdp, off) : PackedInfo();
    readPackedPivot(pk);
    return pk;
  }

  // The point's changeable settings.
  b3dop::BodyProps props(GA_Offset off) const {
    return readDynamicProps(node, a, off);
  }
};

// A point's shapes, plus their signature and scale key.
struct ShapePlan {
  b3dop::ShapeType type = b3dop::ShapeType::Box;
  BuiltShapes shapes;         // no pieces for `none`
  bool plannedStatic = false; // planned for a static body (concave: a mesh)
  uint64_t sig = 0;
  bool autoScale = false;
  double sizeKey[3] = {1, 1, 1};
};

int addShapes(b3dop::Box3DWorld &world, b3dop::BodyHandle h,
              const b3dop::BodySettings &b, const BuiltShapes &shapes);
b3dop::BodyHandle
createBodyWithShapes(b3dop::Box3DWorld &world, const b3dop::BodySettings &b,
                     const Box3DBodyPose &pose, const BuiltShapes &shapes,
                     bool shapeless, const b3dop::BodyProps &p,
                     bool giveVelocity, Box3DBodyMirror &entry);
unsigned applyPropChanges(b3dop::Box3DWorld &world, Box3DBodyMirror &e,
                          const b3dop::BodyProps &p, bool shapesRebuilt);

struct SyncCounts {
  int added = 0, edited = 0, retyped = 0, failed = 0, duplicates = 0, badP = 0;
  int rebuilt = 0;     // shapes replaced
  int dead = 0;        // `dead` points
  int badVelocity = 0; // non-finite v or w, read as zero
  int updated = 0;     // settings changed
  int skipped = 0;     // objects not scanned
  std::unordered_map<std::string, int> fallbacks; // note -> count
  int failedObjects = 0; // whole objects (syncWholeObject) that got no body
  std::unordered_map<std::string, int>
      objectFallbacks; // whole objects: note -> count
  // Box3D Data values overridden by point attributes (RBD Packed Object writes
  // several from its own parameters): attribute -> objects, sorted for the
  // message. Each object counts once (noteOverriddenData).
  std::map<std::string, int> overriddenData;
  int overriddenObjects = 0;
};

constexpr size_t kProgressEvery =
    64; // points between Escape checks in the sync
constexpr size_t kPlanChunk =
    256; // shape plans between Escape checks in the pre-pass

void syncObject(Box3DSharedState &st, const SIM_Object &obj, SyncCounts &c,
                StepProgress &progress);

// ---- whole-object bodies (Box3DWholeObject.cpp) ----

// The point id of a whole object's single body in the mirror key.
inline constexpr int64_t kWholeObjectId = -1;

bool isWholeObject(const SIM_Object &obj);
// The body's pose from the object's Position data (pivot in object space, pose
// at the pivot). False if the object has none.
bool readWholePose(const SIM_Object &obj, double pivot[3], Box3DBodyPose &out);
void syncWholeObject(Box3DSharedState &st, const SIM_Object &obj,
                     const SIM_Geometry &geo, const GU_Detail *gdp,
                     SyncCounts &c);
bool writeWholeObject(SIM_Object &obj, Box3DBodyMirror &e, const double pos[3],
                      const float rot[4], const float lin[3],
                      const float ang[3]);

// ---- after the step (Box3DPostStep.cpp) and forces (Box3DForces.cpp) ----

SIM_GeometryCopy *getOrMakeWritableGeometry(SIM_Object &obj);
void recordFingerprintAfterWrite(Box3DSharedState &st, const SIM_Object &obj);
void forceScan(Box3DSharedState &st, int objectId);
void recordImpacts(Box3DSharedState &st, SIM_ObjectArray &objects,
                   const SIM_Time &now);
void writePointAttrs(Box3DSharedState &st, SIM_ObjectArray &objects);
void ageObjects(Box3DSharedState &st, SIM_ObjectArray &objects, float dt);
void deleteDeadPoints(Box3DSharedState &st, SIM_ObjectArray &objects);

struct ForceStats {
  int bodies = 0; // bodies that got a force or torque
  int nonFinite =
      0; // bodies whose force DOPs returned NaN / inf (those are skipped)
};

ForceStats applyExternalForces(Box3DSharedState &st, SIM_ObjectArray &objects,
                               bool wake);

// ---- guide geometry (Box3DGuide.cpp) ----

// The Box3D Solver's guide for one object: each of its bodies' collision shapes
// (as Box3D holds them) as wireframe polylines at the pose the object's
// geometry shows on this frame, coloured `color` (dimmed for sleeping and
// static bodies). Nothing if no Box3D world has the object's bodies.
void buildCollisionGuide(const SIM_Object &obj, GU_Detail &out,
                         const UT_Vector3 &color);

} // namespace box3dsolver
