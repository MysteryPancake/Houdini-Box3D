// Box3DDebug.h
//
// Debug logging to stderr (Houdini's console), compiled in with BOX3DDOP_DEBUG (CMake option,
// on by default). Environment variables (debug builds only):
//     BOX3DDOP_REQUESTED_CACHE=<int>  what getRequestedCacheSubclass() returns (read every step)
//     BOX3DDOP_VERBOSE=1              also log what can happen every step (slow)
//     BOX3DDOP_VERIFY_READBACK=1      re-read written points and log "READBACK MISMATCH" (slow)
#pragma once

#include <atomic>
#include <cstdlib>

#ifdef BOX3DDOP_DEBUG

#include <cstdarg>
#include <cstdio>
#include <mutex>

namespace b3dop_dbg {

struct Counters
{
    std::atomic<int> rebuilds{0};         // full world rebuilds
};

inline Counters &counters() { static Counters c; return c; }

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
inline void log(const char *fmt, ...)
{
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

inline int envInt(const char *name, int fallback)
{
    const char *v = std::getenv(name);
    return (v && *v) ? std::atoi(v) : fallback;
}

// Read once. Per-step diagnostics go through B3D_LOG_VERBOSE and stay silent unless this is set.
inline bool verbose()
{
    static const bool v = envInt("BOX3DDOP_VERBOSE", 0) != 0;
    return v;
}
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

} // namespace b3dop_dbg

#define B3D_LOG(...)             ::b3dop_dbg::log(__VA_ARGS__)
#define B3D_LOG_VERBOSE(...)     do { if (::b3dop_dbg::verbose()) ::b3dop_dbg::log(__VA_ARGS__); } while (0)   // arguments are only evaluated when verbose
#define B3D_LOG_IF(cond, ...)    do { if ((cond) || ::b3dop_dbg::verbose()) ::b3dop_dbg::log(__VA_ARGS__); } while (0)   // always when cond, else only when verbose
#define B3D_COUNT(field)         (++::b3dop_dbg::counters().field)
#define B3D_UNCOUNT(field)       (--::b3dop_dbg::counters().field)
#define B3D_COUNTER(field)       (::b3dop_dbg::counters().field.load())
#define B3D_ENV_INT(name, dflt)  ::b3dop_dbg::envInt(name, dflt)

#else // !BOX3DDOP_DEBUG

#define B3D_LOG(...)             ((void)0)
#define B3D_LOG_VERBOSE(...)     ((void)0)
#define B3D_LOG_IF(cond, ...)    ((void)0)
#define B3D_COUNT(field)         ((void)0)
#define B3D_UNCOUNT(field)       ((void)0)
#define B3D_COUNTER(field)       (0)
#define B3D_ENV_INT(name, dflt)  (dflt)

#endif
