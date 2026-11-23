// Box3DDebug.h
//
// Debug logging to stderr (Houdini's console), compiled in with BOX3DDOP_DEBUG
// (CMake option, off by default). Environment variables (debug builds only):
//
//   BOX3DDOP_REQUESTED_CACHE=<int>
//       what getRequestedCacheSubclass() returns (read every step)
//   BOX3DDOP_VERBOSE=1
//       also log what can happen every step (slow)
//   BOX3DDOP_VERIFY_READBACK=1
//       re-read written points and log "READBACK MISMATCH" (slow)
//   BOX3DDOP_PROFILE=1
//       log the time the Bond phases took each step (B3D_PROFILE)
#pragma once

#include <atomic>
#include <cstdlib>

#ifdef BOX3DDOP_DEBUG

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <mutex>

namespace b3dop_dbg {

struct Counters {
  std::atomic<int> rebuilds{0}; // full world rebuilds
};

// Phases timed by B3D_PROFILE, summed until B3D_PROFILE_REPORT prints and
// clears them.
enum ProfileSlot {
  kProfileBondSolve,    // the compounds' bond network solves, with their set-up
  kProfileBondPatch,    // a bond's contact patch and area, when worked out
  kProfileBondSideMass, // the two sides' masses of a bond, when worked out
  kProfileBondScan, // breakBonds' loop over the bonds (includes the two above)
  kProfileSlots
};

struct Profile {
  std::atomic<long long> ns[kProfileSlots];
  std::atomic<long long> calls[kProfileSlots];
};

inline Profile &profile() {
  static Profile p{};
  return p;
}

class ScopedTimer {
public:
  explicit ScopedTimer(ProfileSlot slot)
      : mySlot(slot), myStart(std::chrono::steady_clock::now()) {}
  ~ScopedTimer() {
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() - myStart)
                        .count();
    profile().ns[mySlot] += static_cast<long long>(ns);
    ++profile().calls[mySlot];
  }
  ScopedTimer(const ScopedTimer &) = delete;
  ScopedTimer &operator=(const ScopedTimer &) = delete;

private:
  ProfileSlot mySlot;
  std::chrono::steady_clock::time_point myStart;
};

inline Counters &counters() {
  static Counters c;
  return c;
}

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
inline void log(const char *fmt, ...) {
  static std::mutex m;

  std::lock_guard<std::mutex> lock(m);
  std::fputs("[b3dop] ", stderr);
  va_list ap;
  va_start(ap, fmt);
  std::vfprintf(stderr, fmt, ap);
  va_end(ap);
  std::fputc('\n', stderr);
  std::fflush(stderr);
}

inline int envInt(const char *name, int fallback) {
  const char *v = std::getenv(name);
  return (v && *v) ? std::atoi(v) : fallback;
}

// Read once. Per-step diagnostics go through B3D_LOG_VERBOSE and stay silent
// unless this is set.
inline bool verbose() {
  static const bool v = envInt("BOX3DDOP_VERBOSE", 0) != 0;
  return v;
}

// Read once. B3D_PROFILE_REPORT prints only when this is set.
inline bool profiling() {
  static const bool v = envInt("BOX3DDOP_PROFILE", 0) != 0;
  return v;
}

// One line with the phases that ran since the last report (milliseconds, and
// how many times), then cleared.
inline void profileReport(const char *what) {
  Profile &p = profile();
  static const char *const names[kProfileSlots] = {"solve", "patch", "sideMass",
                                                   "scan"};
  char line[256];
  int len = 0;
  bool any = false;
  for (int i = 0; i < kProfileSlots; ++i) {
    const long long ns = p.ns[i].exchange(0);
    const long long calls = p.calls[i].exchange(0);
    if (!calls)
      continue;
    any = true;
    len += std::snprintf(line + len, sizeof line - static_cast<size_t>(len),
                         " %s %.3f ms (%lld)", names[i],
                         static_cast<double>(ns) * 1e-6, calls);
    if (len < 0 || len >= static_cast<int>(sizeof line))
      break;
  }
  if (any && profiling())
    log("%s:%s", what, line);
}
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

} // namespace b3dop_dbg

#define B3D_LOG(...) ::b3dop_dbg::log(__VA_ARGS__)
#define B3D_LOG_VERBOSE(...)                                                   \
  do {                                                                         \
    if (::b3dop_dbg::verbose())                                                \
      ::b3dop_dbg::log(__VA_ARGS__);                                           \
  } while (0) // arguments are only evaluated when verbose
#define B3D_LOG_IF(cond, ...)                                                  \
  do {                                                                         \
    if ((cond) || ::b3dop_dbg::verbose())                                      \
      ::b3dop_dbg::log(__VA_ARGS__);                                           \
  } while (0) // always when cond, else only when verbose
#define B3D_COUNT(field) (++::b3dop_dbg::counters().field)
#define B3D_UNCOUNT(field) (--::b3dop_dbg::counters().field)
#define B3D_COUNTER(field) (::b3dop_dbg::counters().field.load())
#define B3D_ENV_INT(name, dflt) ::b3dop_dbg::envInt(name, dflt)
#define B3D_PROFILE_CAT2(a, b) a##b
#define B3D_PROFILE_CAT(a, b) B3D_PROFILE_CAT2(a, b)
// Times the rest of the scope into a slot (kProfileBondSolve, ...).
#define B3D_PROFILE(slot)                                                      \
  ::b3dop_dbg::ScopedTimer B3D_PROFILE_CAT(b3dProfile,                         \
                                           __LINE__)(::b3dop_dbg::slot)
// Prints and clears what B3D_PROFILE collected (with BOX3DDOP_PROFILE=1).
#define B3D_PROFILE_REPORT(what) ::b3dop_dbg::profileReport(what)

#else // !BOX3DDOP_DEBUG

#include <cstdio>

// Compiled out: never evaluated, but the arguments are still type-checked
// against the format and count as used, so variables that only feed a log line
// need no (void) casts.
#define B3D_LOG(...)                                                           \
  do {                                                                         \
    if (false)                                                                 \
      std::printf(__VA_ARGS__);                                                \
  } while (0)
#define B3D_LOG_VERBOSE(...) B3D_LOG(__VA_ARGS__)
#define B3D_LOG_IF(cond, ...)                                                  \
  do {                                                                         \
    if (false) {                                                               \
      (void)(cond);                                                            \
      std::printf(__VA_ARGS__);                                                \
    }                                                                          \
  } while (0)
#define B3D_COUNT(field) ((void)0)
#define B3D_UNCOUNT(field) ((void)0)
#define B3D_COUNTER(field) (0)
#define B3D_ENV_INT(name, dflt) (dflt)
#define B3D_PROFILE(slot) ((void)0)
#define B3D_PROFILE_REPORT(what) ((void)0)

#endif
