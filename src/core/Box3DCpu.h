#pragma once
// CPU topology for picking Box3D's worker count. Houdini-free.
//
// Box3D runs best with one worker per physical performance core: hyper-threads
// (SMT siblings) and efficiency cores on hybrid CPUs slow its spin-waiting
// solver stages down rather than help.

namespace b3dop {

// Physical performance cores on this machine: hyper-threads not counted, and
// on hybrid CPUs (Intel P/E cores, Apple Silicon, ARM big.LITTLE) only the
// fastest core type. 0 if the platform does not say.
int performanceCoreCount();

// performanceCoreCount() clamped to 1..kMaxWorkerCount, or kDefaultWorkerCount
// when it is unknown. Detected once and cached.
int defaultWorkerCount();

} // namespace b3dop
