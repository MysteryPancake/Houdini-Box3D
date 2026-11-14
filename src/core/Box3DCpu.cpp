#include "core/Box3DCpu.h"
#include "core/Box3DWorld.h"

#include <algorithm>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <vector>
#include <windows.h>
#elif defined(__APPLE__)
#include <sys/sysctl.h>
#include <sys/types.h>
#elif defined(__linux__)
#include <cstdlib>
#include <fstream>
#include <set>
#include <string>
#include <vector>
#endif

namespace b3dop {

namespace {

#if defined(_WIN32)

// One RelationProcessorCore record per physical core (all processor groups).
// EfficiencyClass is 0 on every core of a non-hybrid CPU; on hybrid CPUs the
// performance cores have the highest class.
int detectCores() {
  DWORD len = 0;
  GetLogicalProcessorInformationEx(RelationProcessorCore, nullptr, &len);
  if (len == 0 || GetLastError() != ERROR_INSUFFICIENT_BUFFER) {
    return 0;
  }
  std::vector<char> buf(len);
  if (!GetLogicalProcessorInformationEx(
          RelationProcessorCore,
          reinterpret_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(
              buf.data()),
          &len)) {
    return 0;
  }
  int bestClass = -1;
  int count = 0;
  for (DWORD off = 0; off < len;) {
    const auto *info =
        reinterpret_cast<const SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX *>(
            buf.data() + off);
    if (info->Size == 0) {
      break;
    }
    if (info->Relationship == RelationProcessorCore) {
      const int cls = static_cast<int>(info->Processor.EfficiencyClass);
      if (cls > bestClass) {
        bestClass = cls;
        count = 1;
      } else if (cls == bestClass) {
        ++count;
      }
    }
    off += info->Size;
  }
  return count;
}

#elif defined(__APPLE__)

int sysctlInt(const char *name) {
  int value = 0;
  size_t size = sizeof(value);
  if (sysctlbyname(name, &value, &size, nullptr, 0) != 0) {
    return 0;
  }
  return value > 0 ? value : 0;
}

// perflevel0 is the performance cluster on Apple Silicon (macOS 12+); Intel
// Macs and older systems only have hw.physicalcpu.
int detectCores() {
  const int perf = sysctlInt("hw.perflevel0.physicalcpu");
  return perf > 0 ? perf : sysctlInt("hw.physicalcpu");
}

#elif defined(__linux__)

bool readLine(const std::string &path, std::string &out) {
  std::ifstream f(path);
  if (!f || !std::getline(f, out)) {
    return false;
  }
  while (!out.empty() &&
         (out.back() == '\n' || out.back() == '\r' || out.back() == ' ')) {
    out.pop_back();
  }
  return !out.empty();
}

// Kernel CPU list format: "0-7,16-23".
std::vector<int> parseCpuList(const std::string &s) {
  std::vector<int> cpus;
  size_t pos = 0;
  while (pos < s.size()) {
    size_t end = s.find(',', pos);
    if (end == std::string::npos) {
      end = s.size();
    }
    const std::string part = s.substr(pos, end - pos);
    const size_t dash = part.find('-');
    char *tail = nullptr;
    const long a = std::strtol(part.c_str(), &tail, 10);
    long b = a;
    if (dash != std::string::npos) {
      b = std::strtol(part.c_str() + dash + 1, &tail, 10);
    }
    for (long c = a; c <= b && c >= 0 && c < 65536; ++c) {
      cpus.push_back(static_cast<int>(c));
    }
    pos = end + 1;
  }
  return cpus;
}

int detectCores() {
  const std::string sys = "/sys/devices/system/cpu/";
  std::string list;
  // Intel hybrid CPUs list their performance cores here.
  if (!readLine("/sys/devices/cpu_core/cpus", list) &&
      !readLine(sys + "online", list)) {
    return 0;
  }
  std::vector<int> cpus = parseCpuList(list);

#if defined(__aarch64__) || defined(__arm__)
  // ARM big.LITTLE: keep only the CPUs with the highest capacity. (Not on x86,
  // where per-core capacities can differ slightly on uniform CPUs.)
  std::vector<std::pair<int, long>> caps;
  long best = 0;
  for (const int cpu : cpus) {
    std::string v;
    if (!readLine(sys + "cpu" + std::to_string(cpu) + "/cpu_capacity", v)) {
      caps.clear();
      break;
    }
    const long cap = std::strtol(v.c_str(), nullptr, 10);
    caps.emplace_back(cpu, cap);
    best = std::max(best, cap);
  }
  if (!caps.empty()) {
    cpus.clear();
    for (const auto &[cpu, cap] : caps) {
      if (cap == best) {
        cpus.push_back(cpu);
      }
    }
  }
#endif

  // Hyper-threads of one core share a sibling list: count distinct lists.
  std::set<std::string> cores;
  for (const int cpu : cpus) {
    const std::string topo = sys + "cpu" + std::to_string(cpu) + "/topology/";
    std::string siblings;
    if (!readLine(topo + "core_cpus_list", siblings) &&
        !readLine(topo + "thread_siblings_list", siblings)) {
      return 0;
    }
    cores.insert(siblings);
  }
  return static_cast<int>(cores.size());
}

#else

int detectCores() { return 0; }

#endif

} // namespace

int performanceCoreCount() {
  static const int count = detectCores();
  return count;
}

int defaultWorkerCount() {
  const int n = performanceCoreCount();
  return n > 0 ? std::clamp(n, 1, kMaxWorkerCount) : kDefaultWorkerCount;
}

} // namespace b3dop
