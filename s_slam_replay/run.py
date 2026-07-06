#!/usr/bin/env python3
"""Run an end-to-end s-slam offline replay regression."""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import math
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from statistics import median
from datetime import datetime
from pathlib import Path
from typing import Any

try:
    import psutil
except ImportError:  # optional dependency; resource metrics are skipped without it
    psutil = None  # type: ignore[assignment]


GENERATED_OUTPUTS = (
    "odom",
    "logs",
    "input_info.txt",
    "metrics.json",
    "report.md",
    "run_manifest.json",
)

DEFAULT_FRONTEND_LAUNCH = "s_slam_replay spark_fast_lio_replay.launch.yaml"
DEFAULT_CLEANUP_TIMEOUT_S = 10.0
DEFAULT_RECORD_START_TIMEOUT_S = 10.0
DEFAULT_PLAYER_READY_TIMEOUT_S = 30.0
DEFAULT_PLAY_START_LEAD_S = 3.0
DEFAULT_DRAIN_SILENCE_S = 4.0
DEFAULT_DRAIN_TIMEOUT_S = 300.0
DEFAULT_RUN_LOCK_TIMEOUT_S = 600.0
DEFAULT_ENDPOINT_MONITOR_INTERVAL_S = 1.0
DEFAULT_REPLAY_CLOCK_HZ = 1000.0
RESOURCE_SAMPLE_INTERVAL_S = 0.5


def expand_path(path: str | Path) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(str(path)))).resolve()


def repo_root() -> Path:
    script_path = Path(__file__).resolve()
    for parent in script_path.parents:
        if (parent / "s_slam_replay").is_dir() and (parent / "s_slam_core").is_dir():
            return parent
        if (parent / "install" / "setup.bash").is_file():
            return parent
    return script_path.parents[1]


def has_metadata(path: Path) -> bool:
    return (path / "metadata.yaml").is_file()


def discover_input_bags(input_path: Path) -> list[Path]:
    if has_metadata(input_path):
        return [input_path]

    if not input_path.is_dir():
        raise FileNotFoundError(f"Input path does not exist: {input_path}")

    bags = [child for child in sorted(input_path.iterdir()) if child.is_dir() and has_metadata(child)]
    if bags:
        return bags

    raise FileNotFoundError(
        f"No ROS 2 bag metadata.yaml found in {input_path} or its immediate child directories"
    )


def source_prefix(ros_setup: Path, workspace_setup: Path) -> str:
    if not ros_setup.is_file():
        raise FileNotFoundError(f"ROS setup file not found: {ros_setup}")
    if not workspace_setup.is_file():
        raise FileNotFoundError(
            f"Workspace setup file not found: {workspace_setup}. Build the workspace first."
        )

    return (
        f"source {shlex.quote(str(ros_setup))} && "
        f"source {shlex.quote(str(workspace_setup))}"
    )


def sourced_command(prefix: str, argv: list[str], *, start_at_epoch: float | None = None) -> str:
    wait_until = ""
    if start_at_epoch is not None:
        wait_script = (
            "import time; "
            f"target={start_at_epoch:.9f}; "
            "delay=target-time.time(); "
            "time.sleep(delay if delay > 0.0 else 0.0)"
        )
        wait_until = f" && {shlex.quote(sys.executable)} -c {shlex.quote(wait_script)}"
    return f"{prefix}{wait_until} && exec {shlex.join(argv)}"


def run_sourced_capture(
    prefix: str,
    argv: list[str],
    timeout: float = 30.0,
    *,
    start_at_epoch: float | None = None,
) -> str:
    result = subprocess.run(
        ["bash", "-lc", sourced_command(prefix, argv, start_at_epoch=start_at_epoch)],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
    )
    if result.returncode != 0:
        raise RuntimeError(f"Command failed ({result.returncode}): {shlex.join(argv)}\n{result.stdout}")
    return result.stdout


def run_sourced_capture_optional(prefix: str, argv: list[str], timeout: float = 10.0) -> tuple[int, str]:
    result = subprocess.run(
        ["bash", "-lc", sourced_command(prefix, argv)],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
    )
    return result.returncode, result.stdout


def start_sourced_process(
    prefix: str,
    argv: list[str],
    log_path: Path,
    *,
    start_at_epoch: float | None = None,
) -> subprocess.Popen[Any]:
    log_file = log_path.open("w", encoding="utf-8", errors="replace")
    return subprocess.Popen(
        ["bash", "-lc", sourced_command(prefix, argv, start_at_epoch=start_at_epoch)],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )


def log_tail(log_path: Path, max_lines: int = 40) -> str:
    if not log_path.is_file():
        return "<missing log>"
    lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    return "\n".join(lines[-max_lines:])


def terminate_process(process: subprocess.Popen[Any], name: str, timeout: float = 10.0) -> int:
    if process.poll() is not None:
        return process.returncode

    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return process.poll() or 0

    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return process.poll() or 0

    try:
        return process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        return process.wait(timeout=3.0)


def cleanup_old_processes() -> None:
    for pattern in (
        "[r]slidar_sdk_node",
        "[s]park_lio_mapping",
        "[r]os2 bag play",
        "[r]os2 bag record",
    ):
        subprocess.run(["pkill", "-f", pattern], check=False)


def acquire_run_lock(timeout: float) -> Any:
    lock_path = Path("/tmp/s_slam_replay.lock")
    lock_file = lock_path.open("w", encoding="utf-8")
    deadline = time.monotonic() + timeout
    while True:
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            lock_file.seek(0)
            lock_file.truncate()
            lock_file.write(f"pid={os.getpid()} cwd={Path.cwd()}\n")
            lock_file.flush()
            return lock_file
        except BlockingIOError:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Timed out waiting for exclusive replay lock: {lock_path}. "
                    "Another s_slam_replay/run.py instance is still running."
                )
            time.sleep(0.25)


def prepare_output_dir(output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    for name in GENERATED_OUTPUTS:
        path = output_dir / name
        if path.is_dir():
            shutil.rmtree(path)
        elif path.exists():
            path.unlink()

    (output_dir / "logs").mkdir(parents=True, exist_ok=True)


def parse_endpoint_count(topic_info: str, label: str) -> int:
    match = re.search(rf"^{re.escape(label)} count:\s+(\d+)\s*$", topic_info, re.MULTILINE)
    return int(match.group(1)) if match else 0


def topic_endpoint_counts(prefix: str, topic: str) -> tuple[int, int]:
    returncode, output = run_sourced_capture_optional(prefix, ["ros2", "topic", "info", topic])
    if returncode != 0:
        return 0, 0
    return parse_endpoint_count(output, "Publisher"), parse_endpoint_count(output, "Subscription")


def topic_endpoint_nodes(prefix: str, topic: str, endpoint_type: str) -> list[str]:
    returncode, output = run_sourced_capture_optional(prefix, ["ros2", "topic", "info", topic, "-v"])
    if returncode != 0:
        return []

    nodes: list[str] = []
    current_node: str | None = None
    current_namespace = ""
    for line in output.splitlines():
        stripped = line.strip()
        if stripped.startswith("Node name:"):
            current_node = stripped.split(":", 1)[1].strip()
            current_namespace = ""
        elif stripped.startswith("Node namespace:"):
            current_namespace = stripped.split(":", 1)[1].strip()
        elif stripped.startswith("Endpoint type:") and stripped.endswith(endpoint_type) and current_node:
            namespace = current_namespace.rstrip("/")
            nodes.append(f"{namespace}/{current_node}" if namespace else f"/{current_node}")
            current_node = None
            current_namespace = ""
    return sorted(nodes)


def topic_publisher_nodes(prefix: str, topic: str) -> list[str]:
    return topic_endpoint_nodes(prefix, topic, "PUBLISHER")


def topic_subscription_nodes(prefix: str, topic: str) -> list[str]:
    return topic_endpoint_nodes(prefix, topic, "SUBSCRIPTION")


def wait_for_no_publishers(prefix: str, topics: list[str], timeout: float) -> None:
    unique_topics = sorted(set(topics))
    deadline = time.monotonic() + timeout
    last_counts: dict[str, int] = {}
    while time.monotonic() < deadline:
        last_counts = {topic: topic_endpoint_counts(prefix, topic)[0] for topic in unique_topics}
        if all(count == 0 for count in last_counts.values()):
            return
        time.sleep(0.25)

    formatted = ", ".join(f"{topic}={count}" for topic, count in last_counts.items())
    raise TimeoutError(f"Timed out waiting for old publishers to disappear: {formatted}")


def wait_for_topic_subscription(
    prefix: str,
    topic: str,
    timeout: float,
    *,
    expected_nodes: set[str] | None = None,
) -> None:
    deadline = time.monotonic() + timeout
    last_counts = (0, 0)
    last_subscribers: list[str] = []
    while time.monotonic() < deadline:
        if expected_nodes is None:
            last_counts = topic_endpoint_counts(prefix, topic)
            if last_counts[1] > 0:
                return
        else:
            last_subscribers = topic_subscription_nodes(prefix, topic)
            if set(last_subscribers) & expected_nodes:
                return
        time.sleep(0.25)

    if expected_nodes is not None:
        raise TimeoutError(
            f"Timed out waiting for recorder subscription on {topic}: subscribers={last_subscribers}"
        )
    raise TimeoutError(
        f"Timed out waiting for a subscriber on {topic}: "
        f"publishers={last_counts[0]} subscriptions={last_counts[1]}"
    )


def wait_for_topic_drain(
    prefix: str,
    topic: str,
    *,
    silence_s: float = DEFAULT_DRAIN_SILENCE_S,
    timeout: float = DEFAULT_DRAIN_TIMEOUT_S,
) -> float:
    """Block until `topic` has been silent for two probe windows in a row.

    After bag playback ends the frontend may still be chewing through buffered
    scans; tearing it down on a fixed timer truncates the tail of the processed
    frame set in a load-dependent way. Each probe waits up to `silence_s` for
    one message: receiving one resets the drain state, timing out counts as a
    silent window. Returns the seconds spent waiting.
    """
    start = time.monotonic()
    deadline = start + timeout
    silent_probes = 0
    while time.monotonic() < deadline:
        try:
            result = subprocess.run(
                ["bash", "-lc", sourced_command(prefix, ["ros2", "topic", "echo", "--once", "--no-arr", topic])],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=silence_s,
            )
        except subprocess.TimeoutExpired:
            silent_probes += 1
            if silent_probes >= 2:
                break
            continue
        if result.returncode == 0:
            silent_probes = 0
        else:
            # Probe tooling hiccup; retry without counting it as silence.
            time.sleep(0.5)
    return time.monotonic() - start


def wait_for_topic_publishers(
    prefix: str,
    topics: list[str],
    timeout: float,
    *,
    expected_nodes: set[str] | None = None,
) -> None:
    unique_topics = sorted(set(topics))
    deadline = time.monotonic() + timeout
    last_publishers: dict[str, list[str]] = {}
    while time.monotonic() < deadline:
        if expected_nodes is None:
            last_counts = {topic: topic_endpoint_counts(prefix, topic)[0] for topic in unique_topics}
            if all(count > 0 for count in last_counts.values()):
                return
            last_publishers = {topic: [str(count)] for topic, count in last_counts.items()}
        else:
            last_publishers = {topic: topic_publisher_nodes(prefix, topic) for topic in unique_topics}
            if all(
                publishers and set(publishers).issubset(expected_nodes)
                for publishers in last_publishers.values()
            ):
                return
        time.sleep(0.25)

    formatted = ", ".join(f"{topic}={publishers}" for topic, publishers in last_publishers.items())
    raise TimeoutError(f"Timed out waiting for bag player publishers: {formatted}")


def ensure_process_running(
    process: subprocess.Popen[Any],
    *,
    name: str,
    log_path: Path,
) -> None:
    if process.poll() is None:
        return
    raise RuntimeError(
        f"{name} exited before it was ready (code {process.returncode}).\n"
        f"Last log lines from {log_path}:\n{log_tail(log_path)}"
    )


def wait_for_service(
    prefix: str,
    service_name: str,
    timeout: float,
    *,
    process: subprocess.Popen[Any] | None = None,
    process_name: str = "process",
    log_path: Path | None = None,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process is not None and log_path is not None:
            ensure_process_running(process, name=process_name, log_path=log_path)
        returncode, _output = run_sourced_capture_optional(
            prefix,
            ["ros2", "service", "type", service_name],
        )
        if returncode == 0:
            return
        time.sleep(0.25)

    raise TimeoutError(f"Timed out waiting for service: {service_name}")


def wait_for_topic_publishers_exact(
    prefix: str,
    expected_publishers_by_topic: dict[str, set[str]],
    timeout: float,
    *,
    processes_by_node: dict[str, tuple[subprocess.Popen[Any], Path]] | None = None,
) -> None:
    deadline = time.monotonic() + timeout
    last_publishers: dict[str, list[str]] = {}
    while time.monotonic() < deadline:
        if processes_by_node:
            for node, (process, log_path) in processes_by_node.items():
                ensure_process_running(process, name=node, log_path=log_path)

        last_publishers = {
            topic: topic_publisher_nodes(prefix, topic)
            for topic in sorted(expected_publishers_by_topic)
        }
        if all(set(last_publishers[topic]) == expected for topic, expected in expected_publishers_by_topic.items()):
            return
        time.sleep(0.25)

    formatted = ", ".join(f"{topic}={publishers}" for topic, publishers in last_publishers.items())
    expected = {
        topic: sorted(nodes)
        for topic, nodes in expected_publishers_by_topic.items()
    }
    raise TimeoutError(f"Timed out waiting for exact bag publishers: {formatted}; expected={expected}")


def wait_for_frontend(
    prefix: str,
    process: subprocess.Popen[Any],
    timeout: float,
    *,
    lidar_topic: str,
    imu_topic: str,
    odom_topic: str,
) -> None:
    deadline = time.monotonic() + timeout
    last_node_info = ""
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"Frontend exited early with code {process.returncode}")

        returncode, node_info = run_sourced_capture_optional(prefix, ["ros2", "node", "info", "/lio_mapping"])
        last_node_info = node_info
        has_lidar_sub = f"{lidar_topic}: sensor_msgs/msg/PointCloud2" in node_info
        has_imu_sub = f"{imu_topic}: sensor_msgs/msg/Imu" in node_info
        has_odom_pub = f"{odom_topic}: nav_msgs/msg/Odometry" in node_info
        if returncode == 0 and has_lidar_sub and has_imu_sub and has_odom_pub:
            return
        time.sleep(0.25)

    raise TimeoutError(
        "Timed out waiting for /lio_mapping readiness "
        f"(lidar_sub={lidar_topic}, imu_sub={imu_topic}, odom_pub={odom_topic}).\n"
        f"Last node info:\n{last_node_info}"
    )


def count_pattern(text: str, pattern: str) -> int:
    return len(re.findall(pattern, text))


def parse_frontend_log(log_path: Path) -> dict[str, int]:
    text = log_path.read_text(encoding="utf-8", errors="ignore") if log_path.exists() else ""
    return {
        "motion_gate_rejections": count_pattern(text, r"Motion quality gate rejected"),
        "large_lio_jumps": count_pattern(text, r"Large LIO state jump"),
        "no_effective_points": count_pattern(text, r"No Effective Points"),
        "no_point_skips": count_pattern(text, r"No point, skip this scan"),
        "imu_init_rejections": count_pattern(text, r"IMU initialization rejected"),
        "imu_init_done": count_pattern(text, r"IMU Initial Done"),
        "estimator_resets": count_pattern(text, r"Resetting estimator state"),
        "imu_loopbacks": count_pattern(text, r"IMU loopback|IMU timestamps must be in ascending order"),
    }


def parse_bag_topic_types(bag_info_text: str) -> dict[str, str]:
    topic_types: dict[str, str] = {}
    topic_pattern = re.compile(r"Topic:\s+(?P<topic>\S+)\s+\|\s+Type:\s+(?P<type>[^|]+)")
    for match in topic_pattern.finditer(bag_info_text):
        topic = match.group("topic").strip()
        topic_type = match.group("type").strip()
        topic_types[topic] = topic_type
    return topic_types


def select_topic(
    *,
    topic_types: dict[str, str],
    expected_type: str,
    override: str | None,
    label: str,
) -> str:
    if override:
        return override

    candidates = sorted(topic for topic, topic_type in topic_types.items() if topic_type == expected_type)
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise RuntimeError(f"No {expected_type} topic found in input bag. Use --{label}-topic.")

    formatted = ", ".join(candidates)
    raise RuntimeError(f"Multiple {expected_type} topics found: {formatted}. Use --{label}-topic.")


def resolve_config_path(config: str | None) -> Path | None:
    if not config:
        return None

    config_path = expand_path(config)
    if not config_path.is_file():
        raise FileNotFoundError(f"Config YAML not found: {config_path}")
    return config_path


def load_yaml_mapping(path: Path, *, label: str) -> dict[str, Any]:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("python3-yaml is required to read ROS 2 bag metadata") from exc

    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"{label} must contain a YAML mapping: {path}")
    return data


def bag_start_time_seconds(bag_path: Path) -> float | None:
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.is_file():
        return None

    metadata = load_yaml_mapping(metadata_path, label="Bag metadata")
    info = metadata.get("rosbag2_bagfile_information")
    if not isinstance(info, dict):
        return None
    starting_time = info.get("starting_time")
    if not isinstance(starting_time, dict):
        return None
    nanoseconds = starting_time.get("nanoseconds_since_epoch")
    if not isinstance(nanoseconds, int):
        return None
    return nanoseconds / 1.0e9


def build_bag_play_schedule(input_bags: list[Path]) -> list[tuple[Path, float, float | None]]:
    bag_start_times = {bag: bag_start_time_seconds(bag) for bag in input_bags}
    valid_start_times = [start for start in bag_start_times.values() if start is not None]
    if not valid_start_times:
        return [(bag, 0.0, None) for bag in input_bags]

    first_start = min(valid_start_times)
    schedule = []
    for bag in input_bags:
        start_time = bag_start_times[bag]
        start_offset = max(0.0, start_time - first_start) if start_time is not None else 0.0
        schedule.append((bag, start_offset, start_time))
    return sorted(schedule, key=lambda item: (item[1], str(item[0])))


def analyze_odom_via_sourced_python(
    prefix: str,
    odom_dir: Path,
    topic: str,
    large_step_threshold: float,
    frozen_eps: float,
) -> dict[str, Any]:
    output = run_sourced_capture(
        prefix,
        [
            sys.executable,
            str(Path(__file__).resolve()),
            "__analyze_odom",
            "--bag",
            str(odom_dir),
            "--topic",
            topic,
            "--large-step-threshold",
            str(large_step_threshold),
            "--frozen-eps",
            str(frozen_eps),
        ],
        timeout=120.0,
    )
    for line in reversed(output.splitlines()):
        line = line.strip()
        if line.startswith("{") and line.endswith("}"):
            return json.loads(line)
    raise RuntimeError(f"Could not find JSON metrics in analyzer output:\n{output}")


def analyze_input_via_sourced_python(
    prefix: str,
    input_bags: list[Path],
    lidar_topic: str,
    imu_topic: str,
) -> dict[str, Any]:
    output = run_sourced_capture(
        prefix,
        [
            sys.executable,
            str(Path(__file__).resolve()),
            "__analyze_input",
            "--lidar-topic",
            lidar_topic,
            "--imu-topic",
            imu_topic,
            *[item for bag in input_bags for item in ("--bag", str(bag))],
        ],
        timeout=120.0,
    )
    for line in reversed(output.splitlines()):
        line = line.strip()
        if line.startswith("{") and line.endswith("}"):
            return json.loads(line)
    raise RuntimeError(f"Could not find JSON input metrics in analyzer output:\n{output}")


def add_coverage_metrics(metrics: dict[str, Any], input_metrics: dict[str, Any]) -> None:
    lidar_duration = float(input_metrics.get("input_lidar_header_duration_s", 0.0) or 0.0)
    odom_duration = float(metrics.get("odom_header_duration_s", 0.0) or 0.0)
    lidar_count = int(input_metrics.get("input_lidar_count", 0) or 0)
    syncable_lidar_count = int(input_metrics.get("input_lidar_syncable_count", 0) or 0)
    odom_count = int(metrics.get("odom_count", 0) or 0)

    if lidar_duration > 0.0 and odom_duration >= 0.0:
        metrics["odom_coverage_ratio"] = round(odom_duration / lidar_duration, 6)
    if lidar_count > 0:
        metrics["odom_lidar_count_ratio"] = round(odom_count / lidar_count, 6)
    if syncable_lidar_count > 0:
        metrics["odom_syncable_lidar_count_ratio"] = round(odom_count / syncable_lidar_count, 6)
        metrics["odom_missing_syncable_lidar_count"] = max(syncable_lidar_count - odom_count, 0)
    if lidar_count > 0:
        metrics["odom_missing_lidar_count"] = max(lidar_count - odom_count, 0)

    if "odom_header_first_s" in metrics and "input_lidar_header_first_s" in input_metrics:
        metrics["odom_first_lidar_latency_s"] = round(
            metrics["odom_header_first_s"] - input_metrics["input_lidar_header_first_s"],
            6,
        )
    if "odom_header_last_s" in metrics and "input_lidar_header_last_s" in input_metrics:
        metrics["odom_tail_gap_s"] = round(
            input_metrics["input_lidar_header_last_s"] - metrics["odom_header_last_s"],
            6,
        )


def replay_status_and_reasons(
    metrics: dict[str, Any],
    log_metrics: dict[str, int],
    exit_codes: dict[str, Any],
) -> tuple[str, list[str]]:
    play_exit_codes = exit_codes.get("play", [])
    if not isinstance(play_exit_codes, list):
        play_exit_codes = [play_exit_codes]

    frozen_ratio = float(metrics.get("odom_frozen_ratio", 0.0))
    frozen_by_gate = (
        metrics.get("odom_count", 0) > 1 and
        frozen_ratio >= 0.98 and
        (
            log_metrics.get("motion_gate_rejections", 0) > 0 or
            log_metrics.get("no_effective_points", 0) > 0
        )
    )
    coverage_ratio = metrics.get("odom_coverage_ratio")
    syncable_ratio = metrics.get("odom_syncable_lidar_count_ratio")

    fail_reasons: list[str] = []
    if metrics.get("odom_count", 0) <= 0:
        fail_reasons.append("no odometry samples were recorded")
    if metrics.get("odom_nonfinite_samples", 0) > 0:
        fail_reasons.append(f"non-finite odometry samples: {metrics['odom_nonfinite_samples']}")
    if frozen_by_gate:
        fail_reasons.append("odometry is almost fully frozen while frontend reports rejected/ineffective frames")
    if coverage_ratio is not None and coverage_ratio < 0.7:
        fail_reasons.append(f"raw odometry coverage below 0.70: {coverage_ratio}")
    if syncable_ratio is not None and syncable_ratio < 0.7:
        fail_reasons.append(f"syncable LiDAR count coverage below 0.70: {syncable_ratio}")
    if log_metrics.get("large_lio_jumps", 0) > 0:
        fail_reasons.append(f"frontend reported large LIO jumps: {log_metrics['large_lio_jumps']}")
    if metrics.get("odom_large_steps", 0) > 0:
        fail_reasons.append(f"odometry steps exceeded threshold: {metrics['odom_large_steps']}")
    if any(code != 0 for code in play_exit_codes):
        fail_reasons.append(f"bag play exited non-zero: {play_exit_codes}")
    if metrics.get("endpoint_monitor_violations", 0) > 0:
        fail_reasons.append(f"unexpected input topic publishers: {metrics['endpoint_monitor_violations']}")
    if exit_codes.get("record_odom", 0) not in (0, None):
        fail_reasons.append(f"odometry recorder exited non-zero: {exit_codes.get('record_odom')}")

    if fail_reasons:
        return "FAIL", fail_reasons

    return "PASS", ["all hard replay checks passed"]


def replay_quality_warnings(metrics: dict[str, Any], log_metrics: dict[str, int]) -> list[str]:
    coverage_ratio = metrics.get("odom_coverage_ratio")
    syncable_ratio = metrics.get("odom_syncable_lidar_count_ratio")
    warnings: list[str] = []

    if metrics.get("odom_frozen_ratio", 0.0) > 0.5:
        warnings.append(f"odometry frozen ratio above 0.50: {metrics['odom_frozen_ratio']}")
    if coverage_ratio is not None and coverage_ratio < 0.9:
        warnings.append(f"raw odometry coverage below 0.90: {coverage_ratio}")
    if syncable_ratio is not None and syncable_ratio < 0.9:
        warnings.append(f"syncable LiDAR count coverage below 0.90: {syncable_ratio}")
    elif syncable_ratio is not None and syncable_ratio < 0.995:
        warnings.append(
            "syncable LiDAR count coverage below 0.995; the processed frame set "
            f"may be load-dependent and the report not reproducible: {syncable_ratio}"
        )
    if log_metrics.get("imu_init_rejections", 0) > 0:
        warnings.append(f"IMU initialization windows rejected: {log_metrics['imu_init_rejections']}")
    if log_metrics.get("no_point_skips", 0) > 0:
        warnings.append("frontend skipped empty scans")
    if log_metrics.get("no_effective_points", 0) > 0:
        warnings.append("frontend reported ineffective point frames")

    return warnings


def replay_status(metrics: dict[str, Any], log_metrics: dict[str, int], exit_codes: dict[str, Any]) -> str:
    return replay_status_and_reasons(metrics, log_metrics, exit_codes)[0]


class ResourceMonitor:
    """Sample CPU and memory of a process tree from a background thread.

    Used to characterize how much compute the SLAM frontend consumes during
    replay. The root pid and all of its descendants are sampled together, so the
    figures cover the whole `spark_lio_mapping` launch tree. Results are reported
    as average and peak over the sampling window. Sampling is best-effort: if
    `psutil` is unavailable the monitor is inert and reports that.
    """

    def __init__(self, root_pid: int, interval: float = RESOURCE_SAMPLE_INTERVAL_S) -> None:
        self._root_pid = root_pid
        self._interval = interval
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="resource-monitor", daemon=True)
        self._procs: dict[int, Any] = {}
        self._cpu_cores_samples: list[float] = []
        self._rss_bytes_samples: list[int] = []
        self._threads_samples: list[int] = []
        self._sys_cpu_samples: list[float] = []
        self._load1_samples: list[float] = []
        self._started_at: float | None = None
        self._stopped_at: float | None = None
        self._available = psutil is not None

    def start(self) -> None:
        if not self._available:
            return
        self._started_at = time.monotonic()
        self._thread.start()

    def stop(self) -> None:
        if self._started_at is None:
            return
        self._stop.set()
        self._thread.join(timeout=self._interval * 4.0 + 2.0)
        self._stopped_at = time.monotonic()

    def _tree_processes(self) -> list[Any]:
        try:
            root = psutil.Process(self._root_pid)
            return [root, *root.children(recursive=True)]
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return []

    def _run(self) -> None:
        # The first cpu_percent() call for each object only primes a baseline and
        # returns 0.0, so prime the tree and the system counter before sampling.
        psutil.cpu_percent(None)
        for proc in self._tree_processes():
            try:
                proc.cpu_percent(None)
                self._procs[proc.pid] = proc
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

        while not self._stop.wait(self._interval):
            live: dict[int, Any] = {}
            total_cpu = 0.0
            total_rss = 0
            total_threads = 0
            for proc in self._tree_processes():
                cached = self._procs.get(proc.pid, proc)
                live[proc.pid] = cached
                try:
                    total_cpu += cached.cpu_percent(None)
                    total_rss += cached.memory_info().rss
                    total_threads += cached.num_threads()
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            self._procs = live
            self._cpu_cores_samples.append(total_cpu / 100.0)
            self._rss_bytes_samples.append(total_rss)
            self._threads_samples.append(total_threads)
            self._sys_cpu_samples.append(psutil.cpu_percent(None))
            self._load1_samples.append(os.getloadavg()[0])

    def result(self) -> dict[str, Any]:
        if not self._available:
            return {"resource_monitoring": "unavailable (psutil not installed)"}
        if not self._cpu_cores_samples:
            return {"resource_monitoring": "no samples captured"}

        ncpu = os.cpu_count() or 1
        duration = 0.0
        if self._started_at is not None and self._stopped_at is not None:
            duration = max(0.0, self._stopped_at - self._started_at)

        def average(values: list[float]) -> float:
            return sum(values) / len(values) if values else 0.0

        cpu_cores_avg = average(self._cpu_cores_samples)
        cpu_cores_peak = max(self._cpu_cores_samples)
        rss_mb = [value / (1024.0 * 1024.0) for value in self._rss_bytes_samples]
        return {
            "resource_sample_count": len(self._cpu_cores_samples),
            "resource_monitor_duration_s": round(duration, 3),
            "resource_num_cpus": ncpu,
            "frontend_cpu_cores_avg": round(cpu_cores_avg, 3),
            "frontend_cpu_cores_peak": round(cpu_cores_peak, 3),
            "frontend_cpu_util_pct_avg": round(100.0 * cpu_cores_avg / ncpu, 2),
            "frontend_cpu_util_pct_peak": round(100.0 * cpu_cores_peak / ncpu, 2),
            "frontend_rss_mb_avg": round(average(rss_mb), 2),
            "frontend_rss_mb_peak": round(max(rss_mb), 2),
            "frontend_threads_peak": max(self._threads_samples) if self._threads_samples else 0,
            "system_cpu_util_pct_avg": round(average(self._sys_cpu_samples), 2),
            "system_cpu_util_pct_peak": round(max(self._sys_cpu_samples), 2) if self._sys_cpu_samples else 0.0,
            "system_load1_peak": round(max(self._load1_samples), 2) if self._load1_samples else 0.0,
        }


class EndpointMonitor:
    def __init__(
        self,
        prefix: str,
        expected_publishers_by_topic: dict[str, set[str]],
        watched_processes: list[subprocess.Popen[Any]],
        interval: float = DEFAULT_ENDPOINT_MONITOR_INTERVAL_S,
    ) -> None:
        self._prefix = prefix
        self._expected = expected_publishers_by_topic
        self._watched_processes = watched_processes
        self._interval = interval
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="endpoint-monitor", daemon=True)
        self._violations: list[dict[str, Any]] = []

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=self._interval * 4.0 + 2.0)

    def _run(self) -> None:
        while not self._stop.wait(self._interval):
            if any(process.poll() is not None for process in self._watched_processes):
                return
            for topic, expected_publishers in self._expected.items():
                publishers = set(topic_publisher_nodes(self._prefix, topic))
                unexpected_publishers = publishers - expected_publishers
                if not unexpected_publishers:
                    continue
                self._violations.append(
                    {
                        "topic": topic,
                        "expected": sorted(expected_publishers),
                        "actual": sorted(publishers),
                        "wall_time_s": round(time.time(), 6),
                    }
                )

    def result(self) -> dict[str, Any]:
        return {
            "endpoint_monitor_violations": len(self._violations),
            "endpoint_monitor_first_violations": self._violations[:10],
        }


def write_report(
    output_dir: Path,
    input_bags: list[Path],
    bag_info: str,
    replay_details: dict[str, Any],
    metrics: dict[str, Any],
    log_metrics: dict[str, int],
    exit_codes: dict[str, Any],
    resource_metrics: dict[str, Any],
) -> None:
    status, status_reasons = replay_status_and_reasons(metrics, log_metrics, exit_codes)
    quality_warnings = replay_quality_warnings(metrics, log_metrics)
    metrics["status"] = status
    metrics["status_reasons"] = status_reasons
    metrics["quality_warnings"] = quality_warnings

    report_lines = [
        "# s-slam Replay Report",
        "",
        f"- Status: `{status}`",
        f"- Generated at: `{datetime.now().isoformat(timespec='seconds')}`",
        f"- Output: `{output_dir}`",
        "- Input bags:",
    ]
    report_lines.extend(f"  - `{bag}`" for bag in input_bags)
    report_lines.extend(
        [
            "- Status reasons:",
            *[f"  - {reason}" for reason in status_reasons],
        ]
    )
    report_lines.extend(["- Quality warnings:"])
    if quality_warnings:
        report_lines.extend(f"  - {warning}" for warning in quality_warnings)
    else:
        report_lines.append("  - none")
    report_lines.extend(
        [
            "",
            "## Replay Config",
            "",
            "| Field | Value |",
            "|---|---|",
        ]
    )
    for key, value in replay_details.items():
        report_lines.append(f"| `{key}` | `{value}` |")

    report_lines.extend(
        [
            "",
            "## Input Coverage Metrics",
            "",
            "| Metric | Value |",
            "|---|---:|",
        ]
    )
    input_metric_order = (
        "input_lidar_count",
        "input_lidar_header_duration_s",
        "input_lidar_rate_hz",
        "input_imu_count",
        "input_imu_header_duration_s",
        "input_imu_rate_hz",
        "input_lidar_syncable_count",
        "input_lidar_unsyncable_head_count",
        "input_lidar_unsyncable_tail_count",
        "input_lidar_scan_duration_min_s",
        "input_lidar_scan_duration_median_s",
        "input_lidar_scan_duration_max_s",
        "odom_first_lidar_latency_s",
        "odom_tail_gap_s",
        "odom_coverage_ratio",
        "odom_lidar_count_ratio",
        "odom_syncable_lidar_count_ratio",
        "odom_missing_lidar_count",
        "odom_missing_syncable_lidar_count",
    )
    for key in input_metric_order:
        if key in metrics:
            report_lines.append(f"| `{key}` | `{metrics[key]}` |")
    if "input_lidar_time_field_modes" in metrics:
        report_lines.append(
            f"| `input_lidar_time_field_modes` | `{json.dumps(metrics['input_lidar_time_field_modes'], sort_keys=True)}` |"
        )

    report_lines.extend(
        [
            "",
            "## Odometry Metrics",
            "",
            "| Metric | Value |",
            "|---|---:|",
        ]
    )

    metric_order = (
        "odom_count",
        "odom_duration_s",
        "odom_header_duration_s",
        "odom_rate_hz",
        "odom_max_pos_norm_m",
        "odom_final_pos_norm_m",
        "odom_max_step_m",
        "odom_max_speed_m_s",
        "odom_frozen_steps",
        "odom_frozen_ratio",
        "odom_large_steps",
        "odom_nonfinite_samples",
        "odom_sequence_hash",
        "endpoint_monitor_violations",
    )
    for key in metric_order:
        if key in metrics:
            report_lines.append(f"| `{key}` | `{metrics[key]}` |")

    report_lines.extend(
        [
            "",
            "## Frontend Log Signals",
            "",
            "These counts are diagnostic log signals. Use `large_lio_jumps`, "
            "`motion_gate_rejections`, reset/loopback counts, and odometry metrics for "
            "regression decisions; shutdown-bound warning counts can vary without "
            "changing the odometry sequence.",
            "",
            "| Signal | Count |",
            "|---|---:|",
        ]
    )
    for key, value in log_metrics.items():
        report_lines.append(f"| `{key}` | `{value}` |")

    report_lines.append("")
    report_lines.append("## Resource Usage (Frontend)")
    report_lines.append("")
    if "resource_sample_count" in resource_metrics:
        rm = resource_metrics
        report_lines.extend(
            [
                f"Sampled the `spark_lio_mapping` frontend process tree "
                f"{rm['resource_sample_count']} times over "
                f"{rm['resource_monitor_duration_s']} s during bag playback "
                f"(host has {rm['resource_num_cpus']} CPUs).",
                "",
                "| Metric | Average | Peak |",
                "|---|---:|---:|",
                f"| CPU (cores) | `{rm['frontend_cpu_cores_avg']}` | `{rm['frontend_cpu_cores_peak']}` |",
                f"| CPU (% of {rm['resource_num_cpus']} cores) | "
                f"`{rm['frontend_cpu_util_pct_avg']}` | `{rm['frontend_cpu_util_pct_peak']}` |",
                f"| Resident memory (MB) | `{rm['frontend_rss_mb_avg']}` | `{rm['frontend_rss_mb_peak']}` |",
                f"| Threads | — | `{rm['frontend_threads_peak']}` |",
                "",
                "| System context | Average | Peak |",
                "|---|---:|---:|",
                f"| System CPU (% all cores) | "
                f"`{rm['system_cpu_util_pct_avg']}` | `{rm['system_cpu_util_pct_peak']}` |",
                f"| Load average (1 min) | — | `{rm['system_load1_peak']}` |",
            ]
        )
    else:
        note = resource_metrics.get("resource_monitoring", "not captured")
        report_lines.append(f"Resource monitoring {note}.")

    report_lines.extend(
        [
            "",
            "## Process Exit Codes",
            "",
            "```json",
            json.dumps(exit_codes, indent=2, sort_keys=True),
            "```",
            "",
            "## Input Bag Info",
            "",
            "```text",
            bag_info.strip(),
            "```",
        ]
    )

    (output_dir / "report.md").write_text("\n".join(report_lines) + "\n", encoding="utf-8")


def run_replay(args: argparse.Namespace) -> int:
    input_path = expand_path(args.input)
    output_dir = expand_path(args.output)
    # Keep the lock file handle alive for the whole replay. Concurrent replays
    # share ROS topic names and can kill or mix each other's bag players.
    _run_lock = acquire_run_lock(DEFAULT_RUN_LOCK_TIMEOUT_S)
    ros_setup = expand_path(args.ros_setup)
    workspace_setup = expand_path(args.workspace_setup)
    config_path = resolve_config_path(args.config)
    input_bags = discover_input_bags(input_path)

    prepare_output_dir(output_dir)
    logs_dir = output_dir / "logs"
    odom_dir = output_dir / "odom"
    prefix = source_prefix(ros_setup, workspace_setup)

    bag_infos = []
    bag_topic_types: dict[Path, dict[str, str]] = {}
    for bag in input_bags:
        bag_info = run_sourced_capture(prefix, ["ros2", "bag", "info", str(bag)])
        bag_infos.append(f"### {bag}\n{bag_info}")
        bag_topic_types[bag] = parse_bag_topic_types(bag_info)
    bag_info_text = "\n\n".join(bag_infos)
    (output_dir / "input_info.txt").write_text(bag_info_text, encoding="utf-8")
    topic_types = parse_bag_topic_types(bag_info_text)
    run_id = f"{os.getpid()}_{int(time.time() * 1000)}"
    lidar_topic = select_topic(
        topic_types=topic_types,
        expected_type="sensor_msgs/msg/PointCloud2",
        override=args.lidar_topic,
        label="lidar",
    )
    imu_topic = select_topic(
        topic_types=topic_types,
        expected_type="sensor_msgs/msg/Imu",
        override=args.imu_topic,
        label="imu",
    )

    if not args.no_cleanup:
        cleanup_old_processes()
    wait_for_no_publishers(
        prefix,
        [lidar_topic, imu_topic, args.odom_topic],
        DEFAULT_CLEANUP_TIMEOUT_S,
    )

    frontend_args = ["ros2", "launch", *shlex.split(args.frontend_launch)]
    if config_path is not None:
        frontend_args.append(f"config_path:={config_path}")
    frontend_args.extend(
        [
            f"lidar_topic:={lidar_topic}",
            f"imu_topic:={imu_topic}",
            f"map_frame:={args.map_frame}",
            f"base_frame:={args.base_frame}",
            f"lidar_frame:={args.lidar_frame}",
            f"imu_frame:={args.imu_frame}",
            f"visualization_frame:={args.visualization_frame}",
        ]
    )
    frontend = start_sourced_process(prefix, frontend_args, logs_dir / "frontend.log")
    bag_play_schedule = build_bag_play_schedule(input_bags)

    recorder: subprocess.Popen[Any] | None = None
    play_processes: list[subprocess.Popen[Any]] = []
    endpoint_monitor: EndpointMonitor | None = None
    exit_codes: dict[str, Any] = {}
    drain_wait_s = 0.0
    monitor = ResourceMonitor(frontend.pid)
    try:
        wait_for_frontend(
            prefix,
            frontend,
            args.startup_timeout,
            lidar_topic=lidar_topic,
            imu_topic=imu_topic,
            odom_topic=args.odom_topic,
        )
        time.sleep(args.startup_wait)

        recorder_args = ["ros2", "bag", "record", args.odom_topic, "-o", str(odom_dir)]
        recorder = start_sourced_process(prefix, recorder_args, logs_dir / "record_odom.log")
        wait_for_topic_subscription(
            prefix,
            args.odom_topic,
            DEFAULT_RECORD_START_TIMEOUT_S,
            expected_nodes={"/rosbag2_recorder"},
        )
        time.sleep(args.record_start_wait)

        player_nodes = [f"s_slam_replay_player_{run_id}_{index}" for index, _item in enumerate(bag_play_schedule)]
        play_topics: list[list[str]] = []
        play_log_paths: list[Path] = []
        topic_to_nodes: dict[str, list[str]] = {lidar_topic: [], imu_topic: []}
        for index, (bag, start_offset, _start_time) in enumerate(bag_play_schedule):
            player_node = player_nodes[index]
            topics_for_bag = [
                topic for topic in (lidar_topic, imu_topic) if topic in bag_topic_types.get(bag, {})
            ]
            if not topics_for_bag:
                raise RuntimeError(f"Bag {bag} does not contain {lidar_topic} or {imu_topic}")
            play_topics.append(topics_for_bag)
            for topic in topics_for_bag:
                topic_to_nodes[topic].append(player_node)

        for topic, nodes in topic_to_nodes.items():
            if len(nodes) != 1:
                raise RuntimeError(
                    f"Input topic {topic} must be published by exactly one input bag, "
                    f"but matched players={nodes}."
                )

        for index, (bag, _start_offset, _start_time) in enumerate(bag_play_schedule):
            player_node = player_nodes[index]
            topics_for_bag = play_topics[index]
            play_log_path = logs_dir / f"play_{index}.log"
            play_log_paths.append(play_log_path)
            play_args = [
                "ros2",
                "bag",
                "play",
                str(bag),
                "--topics",
                *topics_for_bag,
                "--read-ahead-queue-size",
                str(args.read_ahead_queue_size),
                "--start-paused",
                "--disable-keyboard-controls",
                "--remap",
                f"__node:={player_node}",
            ]
            if index == 0:
                play_args.extend(["--clock", str(DEFAULT_REPLAY_CLOCK_HZ)])
            if args.replay_rate != 1.0:
                play_args.extend(["--rate", str(args.replay_rate)])
            play_processes.append(
                start_sourced_process(
                    prefix,
                    play_args,
                    play_log_path,
                )
            )

        for player_node in player_nodes:
            player_index = player_nodes.index(player_node)
            wait_for_service(
                prefix,
                f"/{player_node}/resume",
                DEFAULT_PLAYER_READY_TIMEOUT_S,
                process=play_processes[player_index],
                process_name=player_node,
                log_path=play_log_paths[player_index],
            )
        expected_input_publishers_by_topic = {
            topic: {f"/{nodes[0]}"}
            for topic, nodes in topic_to_nodes.items()
        }
        wait_for_topic_publishers_exact(
            prefix,
            expected_input_publishers_by_topic,
            DEFAULT_PLAYER_READY_TIMEOUT_S,
            processes_by_node={
                player_node: (play_processes[index], play_log_paths[index])
                for index, player_node in enumerate(player_nodes)
            },
        )

        play_wall_start = time.time() + DEFAULT_PLAY_START_LEAD_S
        monitor.start()
        endpoint_monitor = EndpointMonitor(
            prefix,
            expected_input_publishers_by_topic,
            play_processes,
        )
        endpoint_monitor.start()
        resume_calls: list[dict[str, Any]] = [{} for _item in bag_play_schedule]
        resume_errors: list[BaseException] = []

        def resume_player(index: int, start_offset: float) -> None:
            target_time = play_wall_start + start_offset / max(args.replay_rate, 1.0e-6)
            try:
                run_sourced_capture(
                    prefix,
                    [
                        "ros2",
                        "service",
                        "call",
                        f"/{player_nodes[index]}/resume",
                        "rosbag2_interfaces/srv/Resume",
                    ],
                    timeout=10.0 + start_offset / max(args.replay_rate, 1.0e-6),
                    start_at_epoch=target_time,
                )
                resume_calls[index] = {
                    "node": player_nodes[index],
                    "target_wall_time_s": round(target_time, 6),
                    "returned_wall_time_s": round(time.time(), 6),
                }
            except BaseException as exc:
                resume_errors.append(exc)

        resume_threads = [
            threading.Thread(
                target=resume_player,
                args=(index, start_offset),
                name=f"resume-{index}",
            )
            for index, (_bag, start_offset, _start_time) in enumerate(bag_play_schedule)
        ]
        for thread in resume_threads:
            thread.start()
        for thread in resume_threads:
            thread.join()
        if resume_errors:
            raise RuntimeError(f"Failed to resume bag player: {resume_errors[0]}")

        exit_codes["play"] = [process.wait() for process in play_processes]
        if endpoint_monitor is not None:
            endpoint_monitor.stop()
        time.sleep(args.settle_time)
        # Let the frontend chew through any buffered backlog before teardown;
        # a fixed settle window truncates the processed frame set in a
        # load-dependent way and makes the report non-reproducible.
        drain_wait_s = wait_for_topic_drain(prefix, args.odom_topic)
        monitor.stop()
    finally:
        if endpoint_monitor is not None:
            endpoint_monitor.stop()
        monitor.stop()
        if recorder is not None:
            exit_codes["record_odom"] = terminate_process(recorder, "record_odom")
        exit_codes["frontend"] = terminate_process(frontend, "frontend")
        for process in play_processes:
            if process.poll() is None:
                terminate_process(process, "bag_play")

    metrics = analyze_odom_via_sourced_python(
        prefix,
        odom_dir,
        args.odom_topic,
        args.large_step_threshold,
        args.frozen_eps,
    )
    input_metrics = analyze_input_via_sourced_python(prefix, input_bags, lidar_topic, imu_topic)
    metrics.update(input_metrics)
    add_coverage_metrics(metrics, input_metrics)
    log_metrics = parse_frontend_log(logs_dir / "frontend.log")
    metrics.update(log_metrics)
    resource_metrics = monitor.result()
    metrics.update(resource_metrics)
    metrics["replay_drain_wait_s"] = round(drain_wait_s, 3)
    endpoint_metrics = (
        endpoint_monitor.result()
        if endpoint_monitor is not None
        else {
            "endpoint_monitor_violations": 0,
            "endpoint_monitor_first_violations": [],
        }
    )
    metrics.update(endpoint_metrics)
    status, status_reasons = replay_status_and_reasons(metrics, log_metrics, exit_codes)
    quality_warnings = replay_quality_warnings(metrics, log_metrics)
    metrics["status"] = status
    metrics["status_reasons"] = status_reasons
    metrics["quality_warnings"] = quality_warnings

    manifest = {
        "input": str(input_path),
        "input_bags": [str(path) for path in input_bags],
        "bag_play_schedule": [
            {
                "bag": str(bag),
                "start_offset_s": round(start_offset, 6),
                "start_time_s": start_time,
            }
            for bag, start_offset, start_time in bag_play_schedule
        ],
        "output": str(output_dir),
        "config": str(config_path) if config_path is not None else None,
        "lidar_topic": lidar_topic,
        "imu_topic": imu_topic,
        "odom_topic": args.odom_topic,
        "frontend_launch": args.frontend_launch,
        "frontend_args": frontend_args,
        "run_id": run_id,
        "player_nodes": player_nodes,
        "play_topics": {
            str(bag): topics for (bag, _start_offset, _start_time), topics in zip(bag_play_schedule, play_topics)
        },
        "resume_calls": resume_calls,
        "topic_types": topic_types,
        "frames": {
            "map_frame": args.map_frame,
            "base_frame": args.base_frame,
            "lidar_frame": args.lidar_frame,
            "imu_frame": args.imu_frame,
            "visualization_frame": args.visualization_frame,
        },
        "ros_setup": str(ros_setup),
        "workspace_setup": str(workspace_setup),
        "exit_codes": exit_codes,
        "endpoint_monitor": endpoint_metrics,
        "clock": {
            "publisher_node": player_nodes[0] if player_nodes else None,
            "rate_hz": DEFAULT_REPLAY_CLOCK_HZ,
        },
    }
    (output_dir / "run_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (output_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_report(
        output_dir,
        input_bags,
        bag_info_text,
        {
            "config": str(config_path) if config_path is not None else "launch default",
            "lidar_topic": lidar_topic,
            "imu_topic": imu_topic,
            "bag_play_offsets_s": ", ".join(
                f"{bag.name}:{start_offset:.3f}" for bag, start_offset, _start_time in bag_play_schedule
            ),
            "map_frame": args.map_frame,
            "base_frame": args.base_frame,
            "lidar_frame": args.lidar_frame,
            "imu_frame": args.imu_frame,
            "visualization_frame": args.visualization_frame,
            "frontend_launch": args.frontend_launch,
        },
        metrics,
        log_metrics,
        exit_codes,
        resource_metrics,
    )

    print(f"Replay complete: {output_dir}")
    print(f"Report: {output_dir / 'report.md'}")
    print(
        "Summary: "
        f"status={status}, "
        f"final={metrics['odom_final_pos_norm_m']:.4f} m, "
        f"max={metrics['odom_max_pos_norm_m']:.4f} m, "
        f"max_step={metrics['odom_max_step_m']:.4f} m, "
        f"coverage={metrics.get('odom_coverage_ratio', 0.0):.3f}, "
        f"syncable_coverage={metrics.get('odom_syncable_lidar_count_ratio', 0.0):.3f}, "
        f"imu_init_rejections={log_metrics['imu_init_rejections']}, "
        f"large_lio_jumps={log_metrics['large_lio_jumps']}, "
        f"odom_large_steps={metrics['odom_large_steps']}, "
        f"endpoint_violations={metrics.get('endpoint_monitor_violations', 0)}"
    )
    print("Status reasons: " + "; ".join(status_reasons))
    print("Quality warnings: " + ("; ".join(quality_warnings) if quality_warnings else "none"))
    if "frontend_cpu_cores_peak" in resource_metrics:
        print(
            "Resources: "
            f"cpu_avg={resource_metrics['frontend_cpu_cores_avg']} cores, "
            f"cpu_peak={resource_metrics['frontend_cpu_cores_peak']} cores, "
            f"rss_avg={resource_metrics['frontend_rss_mb_avg']} MB, "
            f"rss_peak={resource_metrics['frontend_rss_mb_peak']} MB"
        )
    return 0


def read_odom_positions(bag_path: Path, topic: str) -> list[tuple[float, float, tuple[float, float, float]]]:
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        available = ", ".join(sorted(topic_types))
        raise RuntimeError(f"Topic {topic!r} not found. Available topics: {available}")
    if topic_types[topic] != "nav_msgs/msg/Odometry":
        raise RuntimeError(
            f"Topic {topic!r} has type {topic_types[topic]!r}, expected nav_msgs/msg/Odometry"
        )

    samples: list[tuple[float, float, tuple[float, float, float]]] = []
    while reader.has_next():
        topic_name, data, timestamp_ns = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, Odometry)
        header_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
        pos = msg.pose.pose.position
        samples.append((timestamp_ns * 1.0e-9, header_time, (pos.x, pos.y, pos.z)))
    return samples


def norm3(values: tuple[float, float, float]) -> float:
    return math.sqrt(values[0] ** 2 + values[1] ** 2 + values[2] ** 2)


def round_or_none(value: float | None, digits: int = 6) -> float | None:
    return round(value, digits) if value is not None else None


def topic_rate(count: int, duration: float | None) -> float:
    if count <= 1 or duration is None or duration <= 0.0:
        return 0.0
    return (count - 1) / duration


def stamp_to_seconds(stamp: Any) -> float:
    return stamp.sec + stamp.nanosec * 1.0e-9


def pointcloud_time_bounds_s(msg: Any, header_time_s: float) -> tuple[float, float, str] | None:
    try:
        import numpy as np
        from sensor_msgs.msg import PointField
    except ImportError:
        return None

    time_field = None
    for field in msg.fields:
        if field.name.lower() in ("timestamp", "time", "t"):
            time_field = field
            break
    if time_field is None or time_field.count != 1:
        return None

    dtype_by_datatype = {
        PointField.INT8: "i1",
        PointField.UINT8: "u1",
        PointField.INT16: "i2",
        PointField.UINT16: "u2",
        PointField.INT32: "i4",
        PointField.UINT32: "u4",
        PointField.FLOAT32: "f4",
        PointField.FLOAT64: "f8",
    }
    dtype_code = dtype_by_datatype.get(time_field.datatype)
    if dtype_code is None:
        return None

    endian = ">" if msg.is_bigendian else "<"
    point_count = msg.width * msg.height
    if point_count <= 0:
        return None

    values = np.ndarray(
        shape=(point_count,),
        dtype=np.dtype(endian + dtype_code),
        buffer=msg.data,
        offset=time_field.offset,
        strides=(msg.point_step,),
    )
    finite_values = values[np.isfinite(values)]
    if finite_values.size == 0:
        return None

    min_value = float(finite_values.min())
    max_value = float(finite_values.max())

    # RoboSense Fairy publishes absolute per-point seconds in `timestamp`.
    # Other datasets commonly publish relative scan offsets in seconds or ms.
    if min_value > header_time_s - 1.0 and max_value < header_time_s + 2.0:
        return min_value, max_value, f"absolute:{time_field.name}"

    if max(abs(min_value), abs(max_value)) <= 1.0:
        scale = 1.0
        mode = f"relative_seconds:{time_field.name}"
    elif max(abs(min_value), abs(max_value)) <= 1000.0:
        scale = 1.0e-3
        mode = f"relative_milliseconds:{time_field.name}"
    else:
        scale = 1.0e-9
        mode = f"relative_nanoseconds:{time_field.name}"
    return header_time_s + min_value * scale, header_time_s + max_value * scale, mode


def read_input_topic_metrics(bag_paths: list[Path], lidar_topic: str, imu_topic: str) -> dict[str, Any]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu, PointCloud2

    topics = {
        lidar_topic: ("input_lidar", PointCloud2),
        imu_topic: ("input_imu", Imu),
    }
    stats: dict[str, dict[str, Any]] = {
        prefix: {
            "count": 0,
            "storage_first": None,
            "storage_last": None,
            "header_first": None,
            "header_last": None,
        }
        for prefix, _msg_type in topics.values()
    }
    lidar_time_bounds: list[tuple[float, float]] = []
    lidar_scan_durations: list[float] = []
    lidar_time_modes: dict[str, int] = {}

    for bag_path in bag_paths:
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        reader.open(storage_options, converter_options)

        while reader.has_next():
            topic_name, data, timestamp_ns = reader.read_next()
            if topic_name not in topics:
                continue

            prefix, msg_type = topics[topic_name]
            msg = deserialize_message(data, msg_type)
            storage_time = timestamp_ns * 1.0e-9
            header_time = stamp_to_seconds(msg.header.stamp)
            entry = stats[prefix]

            if entry["count"] == 0:
                entry["storage_first"] = storage_time
                entry["header_first"] = header_time
            entry["storage_last"] = storage_time
            entry["header_last"] = header_time
            entry["count"] += 1

            if topic_name == lidar_topic:
                bounds = pointcloud_time_bounds_s(msg, header_time)
                if bounds is not None:
                    scan_begin_s, scan_end_s, mode = bounds
                    lidar_time_bounds.append((scan_begin_s, scan_end_s))
                    lidar_scan_durations.append(max(0.0, scan_end_s - scan_begin_s))
                    lidar_time_modes[mode] = lidar_time_modes.get(mode, 0) + 1

    metrics: dict[str, Any] = {}
    for prefix, entry in stats.items():
        count = int(entry["count"])
        storage_duration = (
            entry["storage_last"] - entry["storage_first"]
            if entry["storage_first"] is not None and entry["storage_last"] is not None
            else None
        )
        header_duration = (
            entry["header_last"] - entry["header_first"]
            if entry["header_first"] is not None and entry["header_last"] is not None
            else None
        )

        metrics[f"{prefix}_count"] = count
        metrics[f"{prefix}_storage_duration_s"] = round_or_none(storage_duration)
        metrics[f"{prefix}_header_duration_s"] = round_or_none(header_duration)
        metrics[f"{prefix}_rate_hz"] = round(topic_rate(count, header_duration), 6)
        if entry["storage_first"] is not None:
            metrics[f"{prefix}_storage_first_s"] = round(entry["storage_first"], 6)
        if entry["storage_last"] is not None:
            metrics[f"{prefix}_storage_last_s"] = round(entry["storage_last"], 6)
        if entry["header_first"] is not None:
            metrics[f"{prefix}_header_first_s"] = round(entry["header_first"], 6)
        if entry["header_last"] is not None:
            metrics[f"{prefix}_header_last_s"] = round(entry["header_last"], 6)

    imu_first_s = stats["input_imu"]["header_first"]
    imu_last_s = stats["input_imu"]["header_last"]
    if lidar_time_bounds:
        metrics["input_lidar_time_field_modes"] = dict(sorted(lidar_time_modes.items()))
        metrics["input_lidar_scan_duration_min_s"] = round(min(lidar_scan_durations), 6)
        metrics["input_lidar_scan_duration_median_s"] = round(median(lidar_scan_durations), 6)
        metrics["input_lidar_scan_duration_max_s"] = round(max(lidar_scan_durations), 6)
        if imu_first_s is not None and imu_last_s is not None:
            syncable_count = 0
            unsyncable_head_count = 0
            unsyncable_tail_count = 0
            for scan_begin_s, scan_end_s in lidar_time_bounds:
                if scan_end_s > imu_last_s + 1.0e-6:
                    unsyncable_tail_count += 1
                elif scan_begin_s < imu_first_s - 1.0e-6:
                    unsyncable_head_count += 1
                else:
                    syncable_count += 1
            metrics["input_lidar_syncable_count"] = syncable_count
            metrics["input_lidar_unsyncable_head_count"] = unsyncable_head_count
            metrics["input_lidar_unsyncable_tail_count"] = unsyncable_tail_count
    return metrics


def analyze_odom_bag(args: argparse.Namespace) -> int:
    bag_path = expand_path(args.bag)
    if not has_metadata(bag_path):
        raise FileNotFoundError(f"Odometry bag metadata.yaml not found: {bag_path}")

    samples = read_odom_positions(bag_path, args.topic)
    if not samples:
        raise RuntimeError("No odometry samples found")

    first_t = samples[0][0]
    last_t = samples[-1][0]
    first_header_t = samples[0][1]
    last_header_t = samples[-1][1]
    duration = max(0.0, last_t - first_t)
    header_duration = max(0.0, last_header_t - first_header_t)
    rate = (len(samples) - 1) / duration if duration > 0.0 and len(samples) > 1 else 0.0

    first = samples[0][2]
    rel_positions = [
        (pos[0] - first[0], pos[1] - first[1], pos[2] - first[2])
        for _storage_t, _header_t, pos in samples
    ]
    sequence_hash = hashlib.sha256()
    for (_storage_t, header_t, _pos), rel_pos in zip(samples, rel_positions):
        sequence_hash.update(f"{header_t:.9f},".encode("ascii"))
        sequence_hash.update(
            f"{rel_pos[0]:.9f},{rel_pos[1]:.9f},{rel_pos[2]:.9f}\n".encode("ascii")
        )
    max_norm = max(norm3(pos) for pos in rel_positions)
    final_norm = norm3(rel_positions[-1])

    max_step = 0.0
    max_speed = 0.0
    frozen_steps = 0
    large_steps = 0
    nonfinite_samples = 0
    step_count = max(len(samples) - 1, 0)
    for _storage_t, _header_t, pos in samples:
        if not all(math.isfinite(value) for value in pos):
            nonfinite_samples += 1

    for (t0, _ht0, p0), (t1, _ht1, p1) in zip(samples, samples[1:]):
        step = norm3((p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]))
        dt = t1 - t0
        max_step = max(max_step, step)
        if dt > 0.0:
            max_speed = max(max_speed, step / dt)
        if step <= args.frozen_eps:
            frozen_steps += 1
        if step > args.large_step_threshold:
            large_steps += 1

    metrics = {
        "odom_count": len(samples),
        "odom_duration_s": round(duration, 6),
        "odom_header_duration_s": round(header_duration, 6),
        "odom_storage_first_s": round(first_t, 6),
        "odom_storage_last_s": round(last_t, 6),
        "odom_header_first_s": round(first_header_t, 6),
        "odom_header_last_s": round(last_header_t, 6),
        "odom_rate_hz": round(rate, 6),
        "odom_first": [round(value, 6) for value in first],
        "odom_last": [round(value, 6) for value in samples[-1][2]],
        "odom_max_pos_norm_m": round(max_norm, 6),
        "odom_final_pos_norm_m": round(final_norm, 6),
        "odom_max_step_m": round(max_step, 6),
        "odom_max_speed_m_s": round(max_speed, 6),
        "odom_frozen_steps": frozen_steps,
        "odom_frozen_ratio": round(frozen_steps / step_count, 6) if step_count > 0 else 0.0,
        "odom_large_steps": large_steps,
        "odom_large_step_threshold_m": args.large_step_threshold,
        "odom_nonfinite_samples": nonfinite_samples,
        "odom_sequence_hash": sequence_hash.hexdigest(),
    }
    print(json.dumps(metrics, sort_keys=True))
    return 0


def analyze_input_bags(args: argparse.Namespace) -> int:
    bag_paths = [expand_path(path) for path in args.bag]
    for bag_path in bag_paths:
        if not has_metadata(bag_path):
            raise FileNotFoundError(f"ROS 2 bag metadata.yaml not found: {bag_path}")

    metrics = read_input_topic_metrics(bag_paths, args.lidar_topic, args.imu_topic)
    print(json.dumps(metrics, sort_keys=True))
    return 0


def parse_args(argv: list[str]) -> argparse.Namespace:
    if argv and argv[0] == "__analyze_odom":
        parser = argparse.ArgumentParser(description="Analyze a recorded /odometry bag")
        parser.add_argument("__command")
        parser.add_argument("--bag", required=True)
        parser.add_argument("--topic", default="/odometry")
        parser.add_argument("--large-step-threshold", type=float, default=0.5)
        parser.add_argument("--frozen-eps", type=float, default=1.0e-4)
        return parser.parse_args(argv)

    if argv and argv[0] == "__analyze_input":
        parser = argparse.ArgumentParser(description="Analyze input LiDAR/IMU bag coverage")
        parser.add_argument("__command")
        parser.add_argument("--bag", action="append", required=True)
        parser.add_argument("--lidar-topic", required=True)
        parser.add_argument("--imu-topic", required=True)
        return parser.parse_args(argv)

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="Input ROS 2 bag directory")
    parser.add_argument("--output", required=True, help="Output directory for odom bag and report")
    parser.add_argument("--config", help="spark_fast_lio YAML config for the input sensor/dataset")
    parser.add_argument("--lidar-topic", help="Override auto-detected PointCloud2 topic")
    parser.add_argument("--imu-topic", help="Override auto-detected Imu topic")
    parser.add_argument("--odom-topic", default="/odometry", help="Frontend odometry topic to record")
    parser.add_argument(
        "--frontend-launch",
        default=DEFAULT_FRONTEND_LAUNCH,
        help="Advanced: arguments passed after `ros2 launch`; must accept config_path/lidar_topic/imu_topic",
    )
    parser.add_argument("--map-frame", default="odom", help="Override map/world frame")
    parser.add_argument("--base-frame", default="base_link", help="Override body/base frame")
    parser.add_argument("--lidar-frame", default="rslidar", help="Override LiDAR frame")
    parser.add_argument("--imu-frame", default="rslidar_imu", help="Override IMU frame")
    parser.add_argument("--visualization-frame", default="base", help="Override frame used by visualization topics")
    parser.add_argument("--ros-setup", default="/opt/ros/humble/setup.bash")
    parser.add_argument(
        "--workspace-setup",
        default=str(repo_root() / "install" / "setup.bash"),
    )
    parser.add_argument("--read-ahead-queue-size", type=int, default=1000)
    parser.add_argument("--replay-rate", type=float, default=1.0)
    parser.add_argument("--startup-timeout", type=float, default=20.0)
    parser.add_argument("--startup-wait", type=float, default=2.0)
    parser.add_argument("--record-start-wait", type=float, default=2.0)
    parser.add_argument("--settle-time", type=float, default=2.0)
    parser.add_argument("--large-step-threshold", type=float, default=0.5)
    parser.add_argument("--frozen-eps", type=float, default=1.0e-4)
    parser.add_argument("--no-cleanup", action="store_true", help="Do not kill old replay/frontend processes")
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args(sys.argv[1:])
    if getattr(args, "__command", None) == "__analyze_odom":
        return analyze_odom_bag(args)
    if getattr(args, "__command", None) == "__analyze_input":
        return analyze_input_bags(args)
    return run_replay(args)


if __name__ == "__main__":
    raise SystemExit(main())
