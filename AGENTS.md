# Agent Notes

This repository is a LiDAR-IMU SLAM workspace. Be conservative: inspect the
existing code path, keep edits scoped, and verify with repeatable replay before
claiming a fix.

## Scope Control

- Do exactly the change the user asked for. Do not add adjacent features,
  config migrations, CLI flags, workflow changes, or extra refactors unless the
  user explicitly asks for them.
- If a related change seems useful, explain it briefly and wait for explicit
  approval before editing.
- Do not introduce a new parameter, config key, CLI option, override, or
  abstraction merely to make an existing relationship adjustable. First verify
  whether the required behavior can be expressed safely with the existing
  inputs and contracts; add a new surface only when that distinction genuinely
  cannot be represented and the user approves it.

## Code Formatting

- After any identifier rename or structural refactor, inspect every changed
  declaration, initializer list, assignment group, and wrapped call for
  alignment and indentation. Mechanical replacements often leave correct but
  visually misleading code.
- Preserve the surrounding file's formatting conventions and run
  `git diff --check` before validation.

## Naming

- Name every identifier for its role and semantic value at the current
  abstraction level, not for its implementation history, producer pipeline, or
  incidental local state. Be precise, but do not repeat context already made
  clear by the class, function, or call site.
- Prefer short, literal names that state the measured or limited quantity. An
  abstract term such as `residual`, `effective`, or `correction` must identify
  its object and condition; configuration names should be understandable
  without reading the implementation.
- Model committed boolean state with a positive predicate such as
  `is_initialized_`; do not reassign a boolean to a value already guaranteed
  by the enclosing control flow.
- Prefer concise, conventional names over long literal descriptions. Avoid
  redundant qualifiers such as `InWorldFrame` when the surrounding API already
  establishes the frame.
- Apply the same rule to variables, types, functions, files, topics, services,
  actions, and configuration keys. A name should answer what the thing is or
  does for its caller, not narrate how it was produced. For example,
  `odometry_px4` states the contract while `odometry_imu_predicted_with_reset`
  leaks implementation stages. Add a qualifier only when callers must
  distinguish otherwise compatible things that coexist.
- Do not invent or preserve opaque abbreviations such as `ds`, `flg`, `wrt`, or
  misspellings such as `i_nex`. Use clear names such as `i_next` and
  `plane_classification`.
- Retain established domain abbreviations when they are broadly understood in
  this codebase and field, including `IMU`, `EKF`, `SLAM`, `PCL`, `FOV`, and
  `sync`.
- Keep unit-bearing names in external configuration where they document the
  contract. For internal identifiers, do not add redundant unit suffixes when
  the local type and context already make the unit clear; never rename an
  external parameter, topic, or config key as part of an internal cleanup.
- A function's return value must represent one clear result that its caller
  must act on, such as success/failure or a computed value. Do not return a
  boolean for indirect lifecycle state such as "has initialization happened";
  keep that state on the owning object, expose a clearly named query only when
  callers need it, and use `void` when the caller has no result to handle.

## Optimization Principle

This is a mass-production system, not a benchmark. Judge every optimization by
engineering soundness on the target platform first, never by how good it makes
test numbers or a replay look.

- Do not tune code to make a specific bag, metric, or replay result look better.
  Replay and test data are tools to detect regressions, not goals to optimize toward.
- Before changing behavior, ask what it does on the real robot: latency on
  target hardware, recovery in flight, failure and restart paths, sensor and
  timing faults, limited compute. An improvement that only holds in the lab is
  not an improvement.
- Change code for a demonstrated defect or measured evidence, not for a
  speculative gain. If there is no bug and no measurement, do not change it.
- Prefer the upstream-proven behavior for flight code. When a determinism or
  test-only need conflicts with production behavior, make it opt-in (build or
  runtime flag) so the shipped default stays production-correct.

## Replay Regression Rules

- Before editing, classify the change using Measurement
  Discipline as a functional/recovery fix, performance optimization, or pure
  refactor. State the intended acceptance metrics and tolerance; do not apply
  one generic checklist to every change.
- Record the baseline commit and worktree state, exact replay command, output
  directory, `odom_sequence_hash`, odom count, coverage, and applicable
  trajectory or GT metrics. Retain the baseline report as a reproducible
  comparison target.
- After a behavior-affecting code, config, launch, TF, topic, or replay change,
  run the selected before/after regression against project-recorded LiDAR/IMU
  bags and at least one ground-truth benchmark when available. Repeat the same
  command; replay runs must not overlap or retain stale ROS publishers.
- A small final displacement is not automatically success. Check coverage with
  every trajectory result; report partial coverage, and never compare it with a
  full-coverage baseline as if the results were equivalent.
- If the same bag gives different hashes, treat that as a bug until proven
  otherwise. Check in order: replay isolation, publisher readiness, callback
  timing, auxiliary publishers, Eigen/OpenMP nondeterminism, map insertion
  order, and uninitialized point fields.

## Measurement Discipline

- For a frontend correctness or recovery fix, compare trajectory state on
  recorded bags (final displacement, path, frame step, jumps,
  frozen/divergent state) and ATE/RTE on ground-truth benchmarks. The intended
  functional metric must improve, while other correctness metrics must not
  regress outside a stated tolerance. CPU and RSS are diagnostic only.
- For a performance optimization, resource metrics must improve and
  trajectory/functional behavior must remain unchanged.
- For a pure refactor, naming change, or code cleanup, trajectory behavior must
  remain unchanged: compare the baseline hash, odom count, coverage, trajectory
  metrics, and applicable ATE/RTE. Performance is diagnostic only unless the
  change was intended to optimize it.
- Keep performance and correctness reports separate. Performance measures CPU,
  RSS, thread count, wall time, callback time, and queue latency. Correctness
  measures recorded odometry state, drift proxies, actual frame steps,
  coverage, and benchmark APE. Do not use one class of metrics as evidence for
  the other.
- Derive trajectory rate and speed from message header timestamps, never from
  rosbag recorder storage timestamps. Storage timing measures host scheduling,
  not estimator motion.
- Throttled log warnings are diagnostics, not trajectory metrics. In
  particular, count a jump from recorded odometry state, not from a
  `WARN_THROTTLE` message count.
- A preprocessing worker is not automatically a resource optimization: compare
  it with the direct callback path on the target board. Keep it opt-in until
  target measurements show a real latency or reliability benefit.

## Safety Guardrails

- Before adding a threshold, gate, or rejection rule, establish whether it is
  a physical invariant, a sensor-format constraint, or a heuristic. Trace its
  inputs, units, cadence, expected vehicle dynamics, sensor uncertainty, and
  the downstream recovery path before choosing a value.
- Do not turn a symptom from one bag into a global limit. Check the proposed
  rule against every supported sensor rate, irregular timestamps, dropped
  samples, normal maneuvers, and the target platform's actual operating
  envelope. A limit must reject its demonstrated fault without rejecting
  normal operation or trapping the estimator in an unrecoverable state.
- Do not express a physical speed, acceleration, angular-speed, or angular-
  acceleration limit as a fixed per-frame change. Derive the allowed change
  from consecutive message header timestamps and the actual `delta_time`.
  A fixed per-frame threshold is valid only when the sampling interval is
  explicitly fixed and enforced.

## Frontend Determinism Lessons

- `SPARK_FAST_LIO_ENABLE_MP` should remain off unless deliberately testing
  parallel matching. Parallel floating-point reductions can change trajectory
  hashes.
- Keep Eigen single-threaded for deterministic regression builds.
- For replay, prefer callback-driven processing over wall-clock timer-driven
  processing so bag timing, not host scheduling, controls package processing.
- Disable high-rate auxiliary predicted odometry in replay unless explicitly
  testing it; it is not required for `/odometry` regression.
- PCL point types must be fully initialized. Hidden aligned fields and reused
  buffers can create process-to-process differences.
- Deterministic map insertion matters. Sort map points before initial build and
  incremental insertion when validating replay determinism.

## Remote Jetson Command Safety

- Do not put multi-line Python, heredocs, or heavily quoted scripts directly
  inside `ssh 'bash -lc "..."'`. Create a local script under `/tmp`, copy it to
  the Jetson with `scp`, then run that script remotely.
- Be careful with `pkill -f` in remote one-liners. The `-f` pattern matches the
  whole command line, so a command such as `ssh ... 'pkill -f rviz2; ... rviz2
  ...'` can kill its own SSH/session command before the restart runs. Prefer
  `pkill -x rviz2`, a copied remote script, or a pattern that cannot appear in
  the current command line.
- Avoid `set -u` in scripts that source ROS setup files; ROS environment scripts
  may reference unset variables. Use `set -eo pipefail` instead.
- When starting live SDK/LIO/RViz on the Jetson, write logs under a timestamped
  directory such as `~/workspace/logs/live_latest/<timestamp>/` and update a
  `latest` symlink. Always inspect SDK, frontend, and RViz logs before claiming
  the live view is ready.

## Reporting

- Say whether a run is `PASS`, `CHECK`, or failed, and explain why.
- Report exact replay output directories for reproduced evidence.
- Do not create commits until the final verified state is agreed.
