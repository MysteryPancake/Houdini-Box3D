// Box3DBondNetwork.h
//
// The load on each bond of a rigid group of pieces (a compound), where no joint
// exists to read it from. The pieces move as one body, so what each bond must
// carry is whatever its piece needs to move as the body does: mass times its
// acceleration, less gravity and the joints that act on it from outside, and
// less what its supports push with. Where bonds form loops, or the group rests
// on many contacts, that does not fix each share (the problem is statically
// indeterminate), so the bonds are taken as small elastic springs between the
// pieces, a rigid-body spring network, and the contacts as elastic supports,
// and the loads are the springs' forces under the pieces' net load.
// Houdini-free.
//
// Per piece (a node) the caller gives the force and torque (about its centre of
// mass) the bonds and supports must supply: `mass * accel - mass * gravity -
// external force`, and the same for the torque. Their sums must be zero, as
// they are when the pieces really move as one body (supports make that hold by
// themselves: what they push with is whatever balances the rest).
//
// A support is a spring from a point on a piece to something fixed, pushing
// along its normal (stiff) and across it (less). It can push but not pull: a
// support whose spring ends up pulling is left out and the network solved
// again.
//
// Solved by conjugate gradient, matrix-free: the cost is a few dozen passes
// over the bonds. It can start from the last solve's displacement (`warm`): a
// large group does not converge in one step's iterations, but the loads change
// little from step to step, so the refinement carries on across them. Without
// supports the system is singular (the whole group can move without loading any
// bond) but consistent, which conjugate gradient handles.
#pragma once

#include "core/Box3DMath.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace b3dop {

// How many times solveBondNetwork solves again with the supports that pulled
// left out.
inline constexpr int kBondSupportPasses = 6;

// A little of the mean stiffness added to every degree of freedom, so a group
// that its supports leave free to move (a compound tipping on its front edge,
// the back ones lifted off) gives a finite answer: it only acts on motions of
// the whole group, which strain no bond.
inline constexpr double kBondRegularization = 1e-9;

struct BondNode {
  double pos[3] = {0, 0, 0};    // centre of mass
  double force[3] = {0, 0, 0};  // N the bonds must supply
  double torque[3] = {0, 0, 0}; // N*m about the centre of mass
};

struct BondLink {
  int a = -1, b = -1;           // nodes
  double anchor[3] = {0, 0, 0}; // where the bond joins them
  double stiffness = 1.0;       // N/m of the spring (only ratios matter)
  double rotStiffness = 1.0;    // N*m/rad
};

// A contact the group rests on: a point on node `node`, the unit direction the
// support pushes it, and the stiffness along and across that.
struct BondSupport {
  int node = -1;
  double pos[3] = {0, 0, 0};
  double normal[3] = {0, 1, 0};
  double normalStiffness = 1.0;
  double tangentStiffness = 0.0;
};

// What a support ended up doing: the push along its normal (>= 0 when it is in
// use), the force on the node, and whether it was used.
struct SupportForce {
  double push = 0.0;
  double force[3] = {0, 0, 0};
  bool active = false;
};

// The bond's load: the force on node b and the couple on it, in world axes (the
// force on a is the opposite; what a weld joint reports for its body B).
struct BondForce {
  double force[3] = {0, 0, 0};
  double torque[3] = {0, 0, 0};
};

namespace detail {

struct BondRegularization {
  double translation = 0.0;
  double rotation = 0.0;
};

// What one link does to the generalized displacement x (u, theta per node):
// the force and couple on node b (everything on node a is the opposite, with
// the moment about its own centre).
struct LinkResponse {
  double f[3], m[3];
};

inline LinkResponse linkResponse(const BondLink &l,
                                 const std::vector<BondNode> &nodes,
                                 const std::vector<double> &x) {
  const double *xa = &x[6 * static_cast<size_t>(l.a)];
  const double *xb = &x[6 * static_cast<size_t>(l.b)];
  double ra[3], rb[3];
  for (int i = 0; i < 3; ++i) {
    ra[i] = l.anchor[i] - nodes[static_cast<size_t>(l.a)].pos[i];
    rb[i] = l.anchor[i] - nodes[static_cast<size_t>(l.b)].pos[i];
  }
  double tb[3], ta[3];
  cross3(xb + 3, rb, tb);
  cross3(xa + 3, ra, ta);
  LinkResponse r;
  for (int i = 0; i < 3; ++i) {
    const double d = xb[i] + tb[i] - xa[i] - ta[i];
    r.f[i] = -l.stiffness * d;
    r.m[i] = -l.rotStiffness * (xb[3 + i] - xa[3 + i]);
  }
  return r;
}

// The force a support puts on its node under the displacement x: a spring along
// its normal and one across it, on the displacement of its point.
inline void supportForce(const BondSupport &s,
                         const std::vector<BondNode> &nodes,
                         const std::vector<double> &x, double force[3]) {
  const double *xn = &x[6 * static_cast<size_t>(s.node)];
  double r[3], t[3];
  for (int i = 0; i < 3; ++i)
    r[i] = s.pos[i] - nodes[static_cast<size_t>(s.node)].pos[i];
  cross3(xn + 3, r, t);
  double d[3];
  double dn = 0.0;
  for (int i = 0; i < 3; ++i) {
    d[i] = xn[i] + t[i];
    dn += d[i] * s.normal[i];
  }
  for (int i = 0; i < 3; ++i)
    force[i] = -(s.normalStiffness * dn * s.normal[i] +
                 s.tangentStiffness * (d[i] - dn * s.normal[i]));
}

// y = K x, with y the negative of the generalized force the links and the
// supports in use put on each node.
inline void applyNetwork(const std::vector<BondNode> &nodes,
                         const std::vector<BondLink> &links,
                         const std::vector<BondSupport> &supports,
                         const std::vector<char> &used,
                         const BondRegularization &reg,
                         const std::vector<double> &x, std::vector<double> &y) {
  for (size_t i = 0; i < y.size(); ++i)
    y[i] = (i % 6 < 3 ? reg.translation : reg.rotation) * x[i];
  for (const BondLink &l : links) {
    const LinkResponse r = linkResponse(l, nodes, x);
    double ra[3], rb[3], tauB[3], tauA[3], negF[3];
    for (int i = 0; i < 3; ++i) {
      ra[i] = l.anchor[i] - nodes[static_cast<size_t>(l.a)].pos[i];
      rb[i] = l.anchor[i] - nodes[static_cast<size_t>(l.b)].pos[i];
      negF[i] = -r.f[i];
    }
    cross3(rb, r.f, tauB);
    cross3(ra, negF, tauA);
    double *ya = &y[6 * static_cast<size_t>(l.a)];
    double *yb = &y[6 * static_cast<size_t>(l.b)];
    for (int i = 0; i < 3; ++i) {
      yb[i] -= r.f[i];
      yb[3 + i] -= tauB[i] + r.m[i];
      ya[i] -= negF[i];
      ya[3 + i] -= tauA[i] - r.m[i];
    }
  }
  for (size_t k = 0; k < supports.size(); ++k) {
    if (!used[k])
      continue;
    const BondSupport &s = supports[k];
    double f[3], r[3], tau[3];
    supportForce(s, nodes, x, f);
    for (int i = 0; i < 3; ++i)
      r[i] = s.pos[i] - nodes[static_cast<size_t>(s.node)].pos[i];
    cross3(r, f, tau);
    double *yn = &y[6 * static_cast<size_t>(s.node)];
    for (int i = 0; i < 3; ++i) {
      yn[i] -= f[i];
      yn[3 + i] -= tau[i];
    }
  }
}

inline double dot(const std::vector<double> &a, const std::vector<double> &b) {
  double s = 0.0;
  for (size_t i = 0; i < a.size(); ++i)
    s += a[i] * b[i];
  return s;
}

// Conjugate gradient for K x = rhs, from the guess in x (zero, or the last
// solve's). Returns the relative residual.
inline double conjugateGradient(
    const std::vector<BondNode> &nodes, const std::vector<BondLink> &links,
    const std::vector<BondSupport> &supports, const std::vector<char> &used,
    const BondRegularization &reg, const std::vector<double> &rhs,
    std::vector<double> &x, int maxIterations, double tolerance) {
  const double rhsNorm = std::sqrt(dot(rhs, rhs));
  if (!(rhsNorm > 0.0)) {
    std::fill(x.begin(), x.end(), 0.0);
    return 0.0;
  }
  // r = rhs - K x
  std::vector<double> r(rhs.size()), kp(rhs.size());
  applyNetwork(nodes, links, supports, used, reg, x, kp);
  for (size_t i = 0; i < r.size(); ++i)
    r[i] = rhs[i] - kp[i];
  double rr = dot(r, r);
  double residual = std::sqrt(rr) / rhsNorm;
  if (!(residual >= tolerance))
    return residual;
  std::vector<double> p = r;
  for (int it = 0; it < maxIterations; ++it) {
    applyNetwork(nodes, links, supports, used, reg, p, kp);
    const double pkp = dot(p, kp);
    if (!(pkp > 0.0))
      break;
    const double alpha = rr / pkp;
    for (size_t i = 0; i < x.size(); ++i) {
      x[i] += alpha * p[i];
      r[i] -= alpha * kp[i];
    }
    const double rrNew = dot(r, r);
    residual = std::sqrt(rrNew) / rhsNorm;
    if (residual < tolerance)
      break;
    const double beta = rrNew / rr;
    rr = rrNew;
    for (size_t i = 0; i < p.size(); ++i)
      p[i] = r[i] + beta * p[i];
  }
  return residual;
}

} // namespace detail

// The loads of `links` under the nodes' net loads, in `out` (one per link),
// with `supports` as elastic supports (`supportOut`, one per support, says what
// each did). Supports that would pull are left out and the network solved
// again, up to `passes` times. Returns the relative residual reached, |K x -
// rhs| / |rhs| (0 when there is nothing to carry), within `maxIterations` per
// solve, or infinity when the solve did not stay finite (the loads are then
// zero). Links and supports to a node out of range are skipped (their load is
// zero).
inline double solveBondNetwork(const std::vector<BondNode> &nodes,
                               const std::vector<BondLink> &links,
                               const std::vector<BondSupport> &supports,
                               std::vector<BondForce> &out,
                               std::vector<SupportForce> &supportOut,
                               int maxIterations = 200, double tolerance = 1e-9,
                               int passes = kBondSupportPasses,
                               std::vector<double> *warm = nullptr) {
  out.assign(links.size(), BondForce());
  supportOut.assign(supports.size(), SupportForce());
  const size_t n = nodes.size();
  std::vector<BondLink> valid;
  std::vector<size_t> index;
  for (size_t i = 0; i < links.size(); ++i) {
    const BondLink &l = links[i];
    if (l.a < 0 || l.b < 0 || static_cast<size_t>(l.a) >= n ||
        static_cast<size_t>(l.b) >= n || l.a == l.b)
      continue;
    valid.push_back(l);
    index.push_back(i);
  }
  std::vector<BondSupport> sup;
  std::vector<size_t> supIndex;
  for (size_t i = 0; i < supports.size(); ++i) {
    const BondSupport &s = supports[i];
    if (s.node < 0 || static_cast<size_t>(s.node) >= n)
      continue;
    sup.push_back(s);
    supIndex.push_back(i);
  }
  std::vector<double> rhs(6 * n), x(6 * n, 0.0);
  for (size_t i = 0; i < n; ++i)
    for (int k = 0; k < 3; ++k) {
      rhs[6 * i + static_cast<size_t>(k)] = -nodes[i].force[k];
      rhs[6 * i + 3 + static_cast<size_t>(k)] = -nodes[i].torque[k];
    }
  if (!(std::sqrt(detail::dot(rhs, rhs)) > 0.0) ||
      (valid.empty() && sup.empty())) {
    if (warm)
      warm->clear();
    return 0.0;
  }
  // The guess: the last solve's displacement, when it is for as many nodes and
  // finite. Any guess reaches the same answer; a good one takes few iterations.
  if (warm && warm->size() == x.size() &&
      std::all_of(warm->begin(), warm->end(),
                  [](double v) { return std::isfinite(v); }))
    x = *warm;
  std::vector<char> used(sup.size(), 1);
  std::vector<double> push(sup.size(), 0.0);
  detail::BondRegularization reg;
  {
    double translation = 0.0, rotation = 0.0;
    for (const BondLink &l : valid) {
      translation += l.stiffness;
      rotation += l.rotStiffness;
    }
    for (const BondSupport &s : sup)
      translation += s.normalStiffness;
    const size_t count = valid.size() + sup.size();
    reg.translation =
        kBondRegularization *
        (count > 0 ? translation / static_cast<double>(count) : 1.0);
    reg.rotation =
        kBondRegularization *
        (valid.empty() ? 1.0 : rotation / static_cast<double>(valid.size()));
  }
  double residual = 0.0;
  for (int pass = 0; pass < std::max(passes, 1); ++pass) {
    residual = detail::conjugateGradient(nodes, valid, sup, used, reg, rhs, x,
                                         maxIterations, tolerance);
    // What each support in use pushes with; one that pulls is dropped.
    double biggest = 0.0;
    for (size_t k = 0; k < sup.size(); ++k) {
      double f[3];
      detail::supportForce(sup[k], nodes, x, f);
      push[k] = 0.0;
      for (int i = 0; i < 3; ++i)
        push[k] += f[i] * sup[k].normal[i];
      biggest = std::max(biggest, std::fabs(push[k]));
    }
    bool dropped = false;
    for (size_t k = 0; k < sup.size(); ++k)
      if (used[k] && push[k] < -1e-9 * std::max(biggest, 1.0)) {
        used[k] = 0;
        dropped = true;
      }
    if (!dropped)
      break;
  }
  // A solve that did not stay finite gives nothing (infinity as the residual).
  for (const double v : x)
    if (!std::isfinite(v)) {
      if (warm)
        warm->clear();
      return std::numeric_limits<double>::infinity();
    }
  if (warm)
    *warm = x;
  for (size_t k = 0; k < valid.size(); ++k) {
    const detail::LinkResponse resp = detail::linkResponse(valid[k], nodes, x);
    BondForce &o = out[index[k]];
    for (int i = 0; i < 3; ++i) {
      o.force[i] = resp.f[i];
      o.torque[i] = resp.m[i];
    }
  }
  for (size_t k = 0; k < sup.size(); ++k) {
    SupportForce &o = supportOut[supIndex[k]];
    o.active = used[k] != 0;
    if (!o.active)
      continue;
    detail::supportForce(sup[k], nodes, x, o.force);
    o.push = push[k];
  }
  return residual;
}

// The same without supports.
inline double solveBondNetwork(const std::vector<BondNode> &nodes,
                               const std::vector<BondLink> &links,
                               std::vector<BondForce> &out,
                               int maxIterations = 200,
                               double tolerance = 1e-9) {
  std::vector<SupportForce> none;
  return solveBondNetwork(nodes, links, std::vector<BondSupport>(), out, none,
                          maxIterations, tolerance, 1);
}

} // namespace b3dop
