// Box3DDebug.h
//
// Debug logging for the DOP plugin. Compiled in only when BOX3DDOP_DEBUG is
// defined (CMake option BOX3DDOP_DEBUG, ON by default; pass OFF for a quiet build).
//
// Output goes to stderr (shown in Houdini's console window).
//
// Other environment variable read (only in debug builds, read every step;
// Houdini must be restarted for the environment itself to change):
//
//     BOX3DDOP_REQUESTED_CACHE=<int>  value getRequestedCacheSubclass() returns
//     BOX3DDOP_VERIFY_READBACK=1      write-back also re-reads every point it wrote and logs
//                                     "READBACK MISMATCH" if that differs (bit for bit) from the
//                                     pose it predicted for the mirror. Slow; for checking only.
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
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

} // namespace b3dop_dbg

#define B3D_LOG(...)             ::b3dop_dbg::log(__VA_ARGS__)
#define B3D_COUNT(field)         (++::b3dop_dbg::counters().field)
#define B3D_UNCOUNT(field)       (--::b3dop_dbg::counters().field)
#define B3D_COUNTER(field)       (::b3dop_dbg::counters().field.load())
#define B3D_ENV_INT(name, dflt)  ::b3dop_dbg::envInt(name, dflt)

#else // !BOX3DDOP_DEBUG

#define B3D_LOG(...)             ((void)0)
#define B3D_COUNT(field)         ((void)0)
#define B3D_UNCOUNT(field)       ((void)0)
#define B3D_COUNTER(field)       (0)
#define B3D_ENV_INT(name, dflt)  (dflt)

#endif
