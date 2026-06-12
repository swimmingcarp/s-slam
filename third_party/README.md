# third_party

Shared third-party dependencies for the s-slam workspace. This directory holds
**two semantically different kinds** of entry — do not assume they are all the
same:

## 1. Vendored source (checked into git, compiled in-tree)

These are real source/header trees committed to the repo and compiled directly;
nothing is downloaded.

| Entry | What | Consumed by |
|-------|------|-------------|
| `ikd-Tree/` | Incremental k-d tree (`ikd_Tree.cpp/.h`) | **`spark_fast_lio`** only — `include_directories(${THIRD_PARTY_DIR}/ikd-Tree)` + `add_library(ikd_tree …)` in [`spark_fast_lio/CMakeLists.txt`](../spark_fast_lio/CMakeLists.txt) |
| `IKFoM_toolkit/` | Error-state iterated Kalman filter toolkit (`esekfom/`, `mtk/` headers) | **`spark_fast_lio`** only — same include block |

## 2. FetchContent dependency scripts (downloaded at build time)

These are **not** the libraries themselves — each is a small CMake script that
`FetchContent`-downloads (or finds a system copy of) the real library during the
build. They are orchestrated by `find_dependencies.cmake`.

| Entry | Library | Wired in |
|-------|---------|----------|
| `find_dependencies.cmake` | orchestrator — finds-or-fetches Eigen3, TBB, ROBIN | `include(…/third_party/find_dependencies.cmake)` in [`kiss_matcher/CMakeLists.txt`](../kiss_matcher/CMakeLists.txt) |
| `eigen/` | Eigen3 (system by default, `USE_SYSTEM_EIGEN3 ON`) | via `find_dependencies.cmake` |
| `tbb/` | oneTBB (`USE_SYSTEM_TBB OFF` → fetched) | via `find_dependencies.cmake` |
| `robin/` | ROBIN (MIT-SPARK) | via `find_dependencies.cmake` |

## Notes

- The KISS-Matcher **core library source** (loop-closure / registration) is *not*
  here — it is built in-tree as part of the `kiss_matcher_ros` package, with
  headers under `kiss_matcher/include/` and sources under `kiss_matcher/src/`.
  It also vendors `tsl::robin_map` internally under `kiss_matcher/include/tsl/`,
  so there is no `tsl_robin` FetchContent entry here.
- Both packages reach this directory via `${CMAKE_CURRENT_SOURCE_DIR}/../third_party`
  (`spark_fast_lio/CMakeLists.txt` and `kiss_matcher/CMakeLists.txt`). If you
  relocate this directory, update both references.
