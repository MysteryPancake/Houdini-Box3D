// Box3DSolverInternal.h
//
// What the Box3D Solver's parts share (not a public interface: only the
// solver's own files include it). The parts, all in namespace box3dsolver:
//   Box3DShapeBuild.cpp   collision source geometry, packed primitives,
//                         building shapes
//   Box3DPointSync.cpp    one body per point: reading points, the slot
//                         passes, syncObject()
//   Box3DPointShapes.cpp  a point's shape plan (planShapes)
//   Box3DPointBody.cpp    body settings, mass, type, making a body
//   Box3DWholeObject.cpp  one body per DOP object (RBD Object etc.): sync and
//                         write-back
//   Box3DWriteBack.cpp    after the step: write-back to points and whole
//   objects Box3DImpacts.cpp      after the step: Impacts data
//   Box3DPostStep.cpp     after the step: point writes, move events, aging,
//                         deletion
//   Box3DForces.cpp       Force DOPs and point force / torque
//   Box3DGuide.cpp        the guide geometry and Export Collision Shapes
// SIM_SolverBox3D.cpp holds the node and the solve loop.
#pragma once

#include "Box3DAttribNames.h"
#include "Box3DHoudiniUtil.h"
#include "core/Box3DDebug.h"
#include "core/Box3DShapes.h"
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
#include <SIM/SIM_Solver.h>
#include <SIM/SIM_Time.h>
#include <SIM/SIM_Utils.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_Matrix3.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_ParallelUtil.h>
#include <UT/UT_PerfMonAutoEvent.h>
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
#include <shared_mutex>
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
  const SIM_Solver *solver = nullptr; // for Performance Monitor events
  bool cancelled = false;             // latched: once true, every check says so

  bool check() {
    if (!cancelled && boss)
      cancelled = boss->opInterrupt() != 0;
    return cancelled;
  }
};

namespace box3dsolver {

// Below these counts a pass runs serially (untuned starting points).
constexpr size_t kMinParallelPoints = 512; // default
constexpr size_t kMinParallelReadPlain =
    1024; // sync read, no primitives in the geometry
constexpr size_t kMinParallelReadPacked =
    512;                                  // sync read, geometry has primitives
constexpr size_t kMinParallelWrite = 256; // write-back
// Bodies of one object before its Force DOPs are evaluated in parallel.
constexpr size_t kMinParallelForces = 256;

// The thresholds in use. In BOX3DDOP_DEBUG builds an environment variable
// overrides each one (a decimal count, read once per process), so a benchmark
// can sweep them without rebuilding:
//   BOX3DDOP_MIN_PARALLEL_POINTS / _READ_PLAIN / _READ_PACKED / _WRITE /
//   _FORCES
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

inline size_t minParallelForces() {
  static const size_t v =
      tunedMinParallel("BOX3DDOP_MIN_PARALLEL_FORCES", kMinParallelForces);
  return v;
}

// f(i) for i in [0, n), on Houdini's TBB pool when n is large. f may only read
// shared data and write element i of its own output.
template <class F>
void parallelForEach(size_t n, const F &f,
                     size_t minParallel = minParallelPoints(),
                     size_t grain = 1) {
  box3dParallelFor(n, f, minParallel, grain);
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
// Packed points' Bullet convention (docs/agents/body-features.md, "Rest
// transform"): `orient` is the rotation since `restxform`
// (box3dRestToAbsolute).
bool restRotation(const UT_Matrix4F &m, float q[4]);
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
  std::vector<float> samples; // quadric primitives' surface points
                              // (Box3DQuadrics.h), added to xyz
  // The piece is one Sphere primitive of uniform scale (sphereValid, after
  // finishPieces): its centre and radius in the source's space, built as a
  // Box3D sphere instead of a hull.
  int primCount = 0;
  bool sphereValid = false;
  double sphereCentre[3] = {0, 0, 0};
  double sphereRadius = 0;
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

// A map whose values are built once, outside the map lock. get() returns the
// value for a key, inserting a default one if missing (shared lock for a hit,
// exclusive only to insert); the caller then builds it with std::call_once on
// a once_flag in V, so different keys build in parallel and the same key
// builds once. Values never move (unordered_map nodes), so references stay
// valid while others insert.
template <class K, class V, class Hash> class OnceCache {
public:
  V &get(const K &key) {
    {
      std::shared_lock<std::shared_mutex> lock(myMutex);
      const auto it = myMap.find(key);
      if (it != myMap.end())
        return it->second;
    }
    std::unique_lock<std::shared_mutex> lock(myMutex);
    return myMap.try_emplace(key).first->second;
  }

private:
  std::shared_mutex myMutex;
  std::unordered_map<K, V, Hash> myMap;
};

struct SourceKeyHash {
  size_t operator()(const std::pair<const GU_Detail *, GA_Offset> &k) const {
    return static_cast<size_t>(
        b3dop::mixHash(reinterpret_cast<uintptr_t>(k.first),
                       static_cast<uint64_t>(static_cast<int64_t>(k.second))));
  }
  size_t operator()(const GU_Detail *k) const {
    return static_cast<size_t>(
        b3dop::mixHash(reinterpret_cast<uintptr_t>(k), 0));
  }
};

// Collision source geometry (what is packed in each point's primitive), cached
// for one syncObject() call. Thread-safe: the shape pre-pass plans points in
// parallel.
struct ShapeSources {
  // Per distinct source detail; the handle keeps the detail (the key) alive.
  struct InnerEntry {
    GU_ConstDetailHandle handle;
    PieceGroup pieces;            // grouped by the `name` attribute
    PieceGroup connected;         // one piece per set of connected primitives
    std::once_flag piecesOnce;    // guards handle + pieces
    std::once_flag connectedOnce; // guards connected
  };
  using SourceKey = std::pair<const GU_Detail *, GA_Offset>;
  OnceCache<const GU_Detail *, InnerEntry, SourceKeyHash> inner;
  // Fragments: (shared source detail, first primitive offset) -> that
  // fragment's piece.
  OnceCache<SourceKey, InnerEntry, SourceKeyHash> fragments;

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
  static std::shared_ptr<const SourceSummary> summary(const GU_PrimPacked &prim,
                                                      bool connected);

  // The primitive's pieces (by `name`, or connected sets), or null. Records the
  // summary.
  const PieceGroup *innerPieces(const GU_PrimPacked &prim, bool connected);

  // The primitive's polygons as triangles (fan-triangulated, source space,
  // Houdini winding), or null if it has none. Per source detail / fragment.
  struct TriEntry {
    GU_ConstDetailHandle handle;
    std::shared_ptr<const MeshPiece> mesh;
    std::once_flag once;
  };
  OnceCache<SourceKey, TriEntry, SourceKeyHash> triangles;
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
  bool mesh = false; // static-only pieces: one triangle mesh, or a ground
};

// staticMesh: the body will be static, so `concave` becomes a triangle mesh of
// the packed polygons (Houdini's clockwise front faces). No polygons: hulls as
// usual.
BuiltShapes buildShapes(b3dop::Box3DWorld &world, ShapeSources &src,
                        b3dop::ShapeType type, bool hullPerConnected,
                        float pscale, const float scale[3],
                        const PackedInfo &pk, bool autofit,
                        const b3dop::PrimOverrides &ov, bool staticMesh = false,
                        const b3dop::GroundSize &ground = b3dop::GroundSize());
BuiltShapes buildWholeObjectShapes(b3dop::Box3DWorld &world,
                                   const GU_Detail *gdp, const double m[16],
                                   const double pivot[3], b3dop::ShapeType type,
                                   bool hullPerConnected,
                                   bool staticMesh = false);

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
  std::shared_mutex mutex; // get() runs in the parallel shape pre-pass
  std::unordered_map<int64_t, Parsed> byHandle;
  uint64_t generation = 0; // which scan's handles the thread memos hold

  void bind(const PointAttrs &a) {
    attr = a.georep.isValid() ? a.georep.getAttribute() : nullptr;
    aif = attr ? attr->getAIFSharedStringTuple() : nullptr;
    static std::atomic<uint64_t> next{1};
    generation = next.fetch_add(1);
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
    const int64_t key = static_cast<int64_t>(handle);
    // The thread's last answer first: the points of a scan mostly share one
    // handle, and a shared lock taken by every thread for every point is a
    // contended cache line.
    struct Memo {
      uint64_t generation = 0;
      int64_t key = 0;
      Parsed parsed;
    };
    thread_local Memo memo;
    if (memo.generation == generation && memo.key == key) {
      if (memo.parsed.ok)
        out = memo.parsed.type;
      return memo.parsed.ok;
    }
    {
      // Nearly every call finds its handle: read under a shared lock.
      std::shared_lock<std::shared_mutex> lock(mutex);
      const auto it = byHandle.find(key);
      if (it != byHandle.end()) {
        memo = {generation, key, it->second};
        if (it->second.ok)
          out = it->second.type;
        return it->second.ok;
      }
    }
    Parsed p;
    p.ok = b3dop::parseShapeType(aif->getTableString(attr, handle), p.type);
    {
      std::unique_lock<std::shared_mutex> lock(mutex);
      byHandle.emplace(key, p); // another thread may have added it: same value
    }
    memo = {generation, key, p};
    if (p.ok)
      out = p.type;
    return p.ok;
  }
};

// ---- point state and body settings (Box3DPointSync.cpp, Box3DPointBody.cpp)
// ----

bool readPointState(const GU_Detail *gdp, const PointAttrs &a, GA_Offset off,
                    const PackedInfo &pk, Box3DBodyPose &out,
                    bool *badVelocity = nullptr,
                    Box3DRestCache *rest = nullptr);
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
// Sets the body's type to `want` if it differs (a dynamic body gets its mass
// back: a type change recomputes it from the shapes). True if it changed.
bool retypeBody(b3dop::Box3DWorld &world, Box3DBodyMirror &entry,
                b3dop::BodySettings::Type want);
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
  // The slots' cached rest rotations describe the current `restxform`.
  bool restReuse = false;
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
  // Bodies planned from one source (an instance run) share it: a copy is a
  // reference count, not a heap allocation of its own.
  std::shared_ptr<const BuiltShapes> built;
  // No pieces for `none` (or before it is set).
  const BuiltShapes &shapes() const {
    static const BuiltShapes kNone;
    return built ? *built : kNone;
  }
  bool plannedStatic = false; // planned for a static body (concave: a mesh)
  uint64_t sig = 0;
  bool autoScale = false;
  double sizeKey[3] = {1, 1, 1};
};

int addShapes(b3dop::Box3DWorld &world, b3dop::BodyHandle h,
              const b3dop::BodySettings &b, const BuiltShapes &shapes);
bool addAllShapes(b3dop::Box3DWorld &world, b3dop::BodyHandle h,
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
  // Points on primitives that are not packed, in objects that also have
  // packed points: not simulated (warning).
  int unpackedPrimPoints = 0;
  // Objects with unpacked primitives and nothing packed: not simulated at all
  // (warning, as RBD Bullet Solver's "Geometry does not contain any primitive
  // types supported").
  std::vector<std::string> unsupportedObjects;
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
// at the pivot). False if the object has none. `forcePivot` (object space)
// replaces the data's pivot.
bool readWholePose(const SIM_Object &obj, double pivot[3], Box3DBodyPose &out,
                   const double *forcePivot = nullptr);
void syncWholeObject(Box3DSharedState &st, const SIM_Object &obj,
                     const SIM_Geometry &geo, const GU_Detail *gdp,
                     SyncCounts &c);
bool writeWholeObject(SIM_Object &obj, Box3DBodyMirror &e, const double pos[3],
                      const float rot[4], const float lin[3],
                      const float ang[3]);

// ---- after the step (Box3DPostStep.cpp, Box3DWriteBack.cpp, Box3DImpacts.cpp)
// and forces (Box3DForces.cpp) ----

// Our own write to the object's Geometry this step: its fingerprint is
// re-recorded by recordFingerprintsAfterWrites, once every lock is released.
void noteOwnWrite(Box3DSharedState &st, const SIM_Object &obj);
void recordFingerprintsAfterWrites(Box3DSharedState &st,
                                   SIM_ObjectArray &objects);
void forceScan(Box3DSharedState &st, int objectId);
// Some solved object has Add Impact Data on.
bool wantsImpacts(const Box3DSharedState &st);
// The contacts of the bodies in `moves` (a Box3D step's move events) as this
// frame's impact hits; `append` keeps the earlier steps' (Substeps > 1: every
// step's contacts are impacts, so the frame's impulses add up). Sets
// st.impactsCollected.
void collectImpacts(Box3DSharedState &st, bool append);
// Merges one Box3D step's move events into st.moves (the frame's, by body).
void mergeMoveEvents(Box3DSharedState &st,
                     const std::vector<b3dop::BodyMove> &events,
                     std::unordered_map<b3dop::BodyHandle, size_t> &slot,
                     bool mayRepeat);
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

// A point's non-zero `force` / `torque` (what POP Force and other POP / SOP
// nodes write).
struct PointForce {
  GA_Offset off;
  UT_Vector3 force, torque;
};

// The point forces of one frame's Box3D steps (Substeps > 1): the first call
// reads and clears the attributes and keeps what it read; `replay` makes the
// later calls apply the kept ones again, so the frame's force acts in every
// step, not only the first.
struct PointForceCarry {
  // What one object's Force DOPs need that does not change within a frame: the
  // forces on it and their split into thread-safe resolvers and the rest
  // (made on first use). Taken again by the frame's first step.
  struct ObjectForces {
    std::vector<const SIM_Force *> forces;
    bool split = false;
    std::vector<std::unique_ptr<SIM_ForceResolver>> parallel;
    std::vector<const SIM_Force *> serial;
  };
  bool replay = false;
  std::unordered_map<int, std::vector<PointForce>> byObject; // object id
  std::unordered_map<int, ObjectForces> forcesOf;            // object id
};

ForceStats applyExternalForces(Box3DSharedState &st, SIM_ObjectArray &objects,
                               bool wake, PointForceCarry *carry = nullptr);

// ---- guide geometry (Box3DGuide.cpp) ----

// The Box3D Solver's guide for one object: each of its bodies' collision shapes
// (as Box3D holds them) as wireframe polylines at the pose the object's
// geometry shows on this frame, coloured `color` (dimmed for sleeping and
// static bodies). Nothing if no Box3D world has the object's bodies.
void buildCollisionGuide(const SIM_Object &obj, GU_Detail &out,
                         const UT_Vector3 &color);

// Export Collision Shapes: the same shapes as world-space polylines (Cd = the
// dimming, white when awake) in each object's kShapesDataName geometry,
// rewritten every step. The solver's lock is held by the caller; poses are the
// written-back ones, so call it after the step's point writes.
inline constexpr const char *kShapesDataName = "Box3DShapes";
void exportCollisionShapes(Box3DSharedState &st, SIM_ObjectArray &objects);

// ---- animated bodies follow their source SOP (Box3DSourceSync.cpp) ----

// Before the sync: each kinematic piece (a point with `animated`, or every
// point of a Kinematic Box3D Data) of an object with SourceObject data gets the
// P and `orient` its piece has in that SOP at the end of the step (`now`,
// simulation time), written as an edit would be, when that pose differs from
// the one last read (the first read compares with the SOP a step `dt` earlier).
// Pieces are matched by `name`.
void followSourceSops(Box3DSharedState &st, SIM_ObjectArray &objects,
                      double now, double dt);

// ---- shared by the post-step files (Box3DPostStep.cpp, Box3DWriteBack.cpp,
// Box3DImpacts.cpp) ----

// Elements per task for the loops whose work per element is a few memory
// reads and writes.
inline constexpr size_t kCheapGrain = 256;

// The point `pointId` (its `id`, else its point number) is still at `off`.
inline bool samePointAt(const GU_Detail *gdp, const GA_ROHandleI &id,
                        GA_Offset off, int64_t pointId) {
  if (off < 0 || off >= gdp->getNumPointOffsets() ||
      gdp->getPointMap().isOffsetVacant(off))
    return false;
  return id.isValid() ? static_cast<int64_t>(id.get(off)) == pointId
                      : static_cast<int64_t>(gdp->pointIndex(off)) == pointId;
}

// Each item's point offset in the geometry about to be written: the entry's
// stored offset if it still names the point (in range, same id / number), else
// looked up by id; GA_INVALID_OFFSET if not found. The stored offsets are
// checked in parallel (read-only); only the points that moved are looked up.
template <class Item> // needs .entry and .off
void resolvePointOffsets(const GU_Detail *gdp, const PointAttrs &a,
                         std::vector<Item> &items, size_t begin, size_t end) {
  // A first lookup by the serial code, in case the point map builds anything
  // on first use, before threads read it.
  if (end > begin)
    (void)gdp->pointIndex(static_cast<GA_Offset>(items[begin].entry->pointOff));
  box3dParallelFor(
      end - begin,
      [&](size_t j) {
        Item &w = items[begin + j];
        const Box3DBodyMirror &m = *w.entry;
        const GA_Offset stored = static_cast<GA_Offset>(m.pointOff);
        w.off = samePointAt(gdp, a.id, stored, m.pointId) ? stored
                                                          : GA_INVALID_OFFSET;
      },
      minParallelWrite(), kCheapGrain);

  std::unordered_map<int64_t, GA_Offset> byId; // built only when needed
  bool builtById = false;
  for (size_t k = begin; k < end; ++k) {
    Item &w = items[k];
    if (w.off != GA_INVALID_OFFSET)
      continue;
    const Box3DBodyMirror &m = *w.entry;
    if (!a.id.isValid()) {
      if (m.pointId >= 0 &&
          m.pointId < static_cast<int64_t>(gdp->getNumPoints()))
        w.off = gdp->pointOffset(static_cast<GA_Index>(m.pointId));
      continue;
    }
    if (!builtById) {
      builtById = true;
      for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
        byId.emplace(static_cast<int64_t>(a.id.get(*it)),
                     *it); // first point of a duplicate id wins
    }
    const auto f = byId.find(m.pointId);
    if (f != byId.end())
      w.off = f->second;
  }
}

// f(begin, end) per run of items with the same objectId. Items are first
// grouped by object, in O(n): a stable bucket pass (items of one object keep
// their relative order), not a sort, since objects are few and the order of the
// move events changes from step to step. Runs come in the order each object
// first appears, not in id order. Already grouped items are not moved.
template <class Item, class F>
void forEachObjectRun(std::vector<Item> &items, const F &f) {
  const size_t n = items.size();
  if (n == 0)
    return;

  // Pass 1: the bucket of every item, and the size of every bucket.
  // Object ids in first-seen order: the index is the bucket. Objects are few,
  // so a linear search beats a hash map. The vectors are kept between calls (no
  // allocation); `bucket` is fully overwritten below.
  thread_local std::vector<int> bucketIds;
  thread_local std::vector<uint32_t> bucket;
  // The size of a bucket, then its first slot.
  thread_local std::vector<size_t> start;
  bucketIds.clear();
  start.clear();
  bucket.resize(n);
  size_t runs = 0;
  int lastId = items[0].objectId;
  uint32_t lastBucket = 0;
  bool haveLast = false;
  for (size_t i = 0; i < n; ++i) {
    const int id = items[i].objectId;
    if (!haveLast || id != lastId) {
      size_t b = 0;
      while (b < bucketIds.size() && bucketIds[b] != id)
        ++b;
      if (b == bucketIds.size())
        bucketIds.push_back(id);
      lastBucket = static_cast<uint32_t>(b);
      lastId = id;
      haveLast = true;
      ++runs;
    }
    if (lastBucket >= start.size())
      start.resize(static_cast<size_t>(lastBucket) + 1, 0);
    ++start[lastBucket];
    bucket[i] = lastBucket;
  }

  // More runs than objects: an object's items are not together, so move them.
  if (runs != bucketIds.size()) {
    size_t sum = 0;
    for (size_t &c : start) {
      const size_t count = c;
      c = sum;
      sum += count;
    }
    thread_local std::vector<Item> scratch; // kept between steps: no allocation
    scratch.resize(n);
    for (size_t i = 0; i < n; ++i)
      scratch[start[bucket[i]]++] = items[i];
    items.swap(scratch);
  }

  for (size_t runBegin = 0; runBegin < n;) {
    size_t runEnd = runBegin;
    while (runEnd < n && items[runEnd].objectId == items[runBegin].objectId)
      ++runEnd;
    f(runBegin, runEnd);
    runBegin = runEnd;
  }
}

// A new quaternion attribute is 0,0,0,0: start every point at identity.
void startOrientAtIdentity(GU_Detail *gdp);

// Creates `orient`, `v`, `w` if missing. True if `orient` was created.
bool ensureResultAttribs(GU_Detail *gdp);

// Harden only the offset ranges the parallel write pass touches (hardenAllPages
// takes a range). Written offsets are bucketed by 1024 and runs of buckets
// hardened; nearly all hit: one full call. `end` is the first offset after a
// run, right whether the HDK reads it as exclusive or not.
void hardenWrittenPages(const GU_Detail *gdp,
                        const std::vector<Box3DWriteItem> &items, size_t begin,
                        size_t end, const std::array<GA_Attribute *, 5> &attrs);

// ---- shared by the point-sync files (Box3DPointSync.cpp,
// Box3DPointShapes.cpp, Box3DPointBody.cpp) ----

// Point attributes that replace a Box3D Data value the user changed, on an
// object with a Box3D Data node. RBD Packed Object writes several of them onto
// every point from its own parameters (mass, density, computemass, friction,
// bounce in Houdini 22.0), so a Box3D Data value set for those does nothing;
// this makes that visible. A value left at its default is not reported: then
// the attribute is the only setting (the Box3D Packed Object writes the
// settings that differ from the defaults as attributes, and its Box3D Data
// keeps them all at the defaults). Reported once per object: on its first full
// scan, and again only if the set changes. (`active` is left out: it only takes
// over Body Type when 0.)
void noteOverriddenData(const GU_Detail *gdp, Box3DObjectSync &os,
                        SyncCounts &c, const b3dop::BodySettings &node);

// pscale / scale (a packed point's size is its transform; these only size its
// fallback box).
void readPscaleScale(const PointAttrs &a, GA_Offset off, float &ps,
                     float sc[3]);

// The scale convex / concave shapes depend on: packed row lengths, else pscale
// * scale.
bool sizeKey(bool packed, const double m3[9], float ps, const float sc[3],
             double out[3]);

// A point's shapes from its current attributes and packed geometry. Only the
// hull cache is touched, no body.
ShapePlan planShapes(ScanContext &ctx, GA_Offset off, int64_t bodyId,
                     const PackedInfo &pk, bool staticBody);

enum : unsigned { kPropsChanged = 1, kMassChanged = 2 };

} // namespace box3dsolver
