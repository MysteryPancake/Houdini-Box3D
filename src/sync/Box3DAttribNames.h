// Box3DAttribNames.h
//
// The point attributes the solver reads by name, written once. PointAttrs (its
// handles and how they are bound), the fingerprint's prop markers
// (kPropAttribNames) and their count are all generated from the lists below, so
// a name is typed in one place and the count cannot drift. Houdini-free (macros
// and strings only) so Box3DState.h can include it.
//
// Each entry is X(member, "attribute", Tag): `member` is the PointAttrs handle,
// Tag the GA_ROHandle suffix (F, I, V3, ID, S, Q). The README's attribute table
// is prose; tools/check_docs.py checks that every name here is mentioned in it.
#pragma once

#include <cstddef>
#include <iterator>

// Bound when a PointAttrs is made: what the skip check and the write-back need.
#define BOX3D_SYNC_ATTRS(X)                                                    \
  X(id, "id", I)         /* else the point number */                           \
  X(active, "active", I) /* 0 = inactive (static, or kinematic if animated) */ \
  X(v, "v", V3)                                                                \
  X(w, "w", V3) /* radians / second */                                         \
  X(orient, "orient",                                                          \
    Q) /* ignored for packed points (the primitive's transform) */

// The fingerprint's prop markers, in marker order: the attributes whose data
// ids say "settings changed, compare every body". Bound with bindBuildAttrs()
// once an object is scanned. The shape attributes below are left out on
// purpose: like Bullet, editing them alone changes nothing until
// bullet_autofit_valid = 0 or deforming = 1.
#define BOX3D_PROP_ATTRS(X)                                                    \
  X(mass, "mass", F)                                                           \
  X(friction, "friction", F)                                                   \
  X(bounce, "bounce", F) /* Box3D restitution */                               \
  X(lineardamping, "lineardamping", F)                                         \
  X(angulardamping, "angulardamping", F)                                       \
  X(gravityscale, "gravityscale", F)                                           \
  X(enablesleep, "enablesleep", I) /* 0 = this body never sleeps */            \
  X(enabled, "enabled", I)         /* 0 = left out of the simulation */        \
  X(awake, "awake", I)             /* 0 = asleep: starts the body */           \
  X(deforming, "deforming", I)     /* rebuild the shape every step */          \
  X(autofitValid, "bullet_autofit_valid", I) /* 0 = rebuild once */            \
  X(pscale, "pscale", F) /* a plain point's convex / concave */                \
  X(scale, "scale", V3)  /* fallback box */                                    \
  X(collisionCategory, "collision_category", ID) /* read as 64-bit */          \
  X(collisionMask, "collision_mask", ID)                                       \
  X(animated, "animated", I)                                                   \
  X(density, "density", F)                                                     \
  X(computeMass, "computemass", I)                                             \
  X(inertiaStiffness, "inertialtensorstiffness", F)                            \
  X(sleepThreshold, "sleepthreshold", F)                                       \
  X(ccdThreshold, "ccdthreshold", F)                                           \
  X(fastObject, "fastobject", I)                                               \
  X(collisionGroup, "collision_group", I)                                      \
  X(dead, "dead", I)                                                           \
  X(contactRecycling, "contactrecycling", I)                                   \
  X(lockLinear, "lock_linear", V3) /* per world axis, nonzero = locked */      \
  X(lockAngular, "lock_angular", V3)                                           \
  X(fastRotation, "fastrotation", I)                                           \
  X(rollingResistance, "rollingresistance", F)                                 \
  X(tangentVelocity, "tangentvelocity", V3)

// Prop markers with no PointAttrs handle (read by name only). Y("attribute"),
// after the list above.
// Joint anchors find bodies by `name`.
#define BOX3D_PROP_NAME_ONLY(Y) Y("name")

// Shape attributes: bound with the props, but not prop markers (see above).
#define BOX3D_SHAPE_ATTRS(X)                                                   \
  X(georep, "bullet_georep", S) /* b3dop::parseShapeType */                    \
  X(autofit, "bullet_autofit", I)                                              \
  X(primS, "bullet_primS", V3) /* bullet_prim* / radius / length: */           \
  X(primR, "bullet_primR", V3) /* see b3dop::PrimOverrides */                  \
  X(primT, "bullet_primT", V3)                                                 \
  X(radius, "bullet_radius", F)                                                \
  X(length, "bullet_length", F)

namespace box3dsolver {

// Names of the prop markers, in order (Box3DSyncFingerprint::propIds); the
// count is derived.
inline constexpr const char *kPropAttribNames[] = {
#define BOX3D_NAME_OF(member, name, Tag) name,
#define BOX3D_NAME_ONLY(name) name,
    BOX3D_PROP_ATTRS(BOX3D_NAME_OF) BOX3D_PROP_NAME_ONLY(BOX3D_NAME_ONLY)
#undef BOX3D_NAME_ONLY
#undef BOX3D_NAME_OF
};

namespace attribnames_detail {

inline constexpr const char *kAllNames[] = {
#define BOX3D_NAME_OF(member, name, Tag) name,
#define BOX3D_NAME_ONLY(name) name,
    BOX3D_SYNC_ATTRS(BOX3D_NAME_OF) BOX3D_PROP_ATTRS(BOX3D_NAME_OF)
        BOX3D_PROP_NAME_ONLY(BOX3D_NAME_ONLY) BOX3D_SHAPE_ATTRS(BOX3D_NAME_OF)
#undef BOX3D_NAME_ONLY
#undef BOX3D_NAME_OF
};

constexpr bool sameString(const char *a, const char *b) {
  while (*a && *a == *b) {
    ++a;
    ++b;
  }
  return *a == *b;
}

// No name is empty or listed twice (a pasted line whose name was not changed).
constexpr bool namesAreDistinct() {
  constexpr size_t n = std::size(kAllNames);
  for (size_t i = 0; i < n; ++i) {
    if (!*kAllNames[i])
      return false;
    for (size_t j = i + 1; j < n; ++j)
      if (sameString(kAllNames[i], kAllNames[j]))
        return false;
  }
  return true;
}

static_assert(namesAreDistinct(),
              "Box3DAttribNames.h: an attribute name is empty or listed twice");

} // namespace attribnames_detail

} // namespace box3dsolver
