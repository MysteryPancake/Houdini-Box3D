// Shared helpers for the core tests.
//
// CHECK(cond) prints the failing line and expression and counts the failure instead of
// aborting, so one run reports every failing check. A test's main() should end with
// `return fails ? 1 : 0;` (or an equivalent) so CTest sees the failure.
#pragma once

#include <cstdio>

// One counter per test executable (each test is a single translation unit).
inline int fails = 0;

#define CHECK(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); ++fails; } } while (0)
