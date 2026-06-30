# third_party

Shared dependency area for the core SLAM packages.

This directory contains two different types of dependency entries. Treat them
differently when updating the workspace.

## Vendored Source

These directories contain source files committed to this repository and compiled
directly in-tree.

| Entry | Used by | Notes |
|-------|---------|-------|
| `ikd-Tree/` | `spark_fast_lio` | Incremental k-d tree implementation |
| `IKFoM_toolkit/` | `spark_fast_lio` | Error-state iterated Kalman filter toolkit |
| `tsl/` | `kiss_matcher_ros` | Header-only robin-map dependency used by KISS-Matcher |

## FetchContent Helpers

These entries are small CMake wrappers. They find or download the real dependency
during the build.

| Entry | Library | Used by |
|-------|---------|---------|
| `find_dependencies.cmake` | dependency orchestrator | `kiss_matcher_ros` |
| `eigen/` | Eigen3 finder/fetch script | `kiss_matcher_ros` |
| `tbb/` | oneTBB fetch script | `kiss_matcher_ros` |
| `robin/` | ROBIN fetch script | `kiss_matcher_ros` |

GTSAM and small_gicp are fetched directly from
`../kiss_matcher/CMakeLists.txt`: GTSAM is used when no system
`GTSAMConfig.cmake` is found, and small_gicp is always fetched as the backend
registration helper.

## Rules

- Do not put runtime config, replay presets, RViz files, or simulator assets here.
- Do not put KISS-Matcher algorithm source here; it lives under
  `../kiss_matcher/include` and `../kiss_matcher/src`.
- If this directory is moved, update both core packages that refer to
  `${CMAKE_CURRENT_SOURCE_DIR}/../third_party`.
- When adding a dependency, document whether it is vendored source or a
  FetchContent helper, and check the license before committing it.
