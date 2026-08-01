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
from html import escape as html_escape
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
    "odom_tum.txt",
    "benchmark_ape.json",
    "benchmark_timeseries.json",
    "metrics.json",
    "plots",
    "report.html",
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
DEFAULT_RTE_DELTA_S = 5.0
DEFAULT_PROGRESS_INTERVAL_S = 10.0
DEFAULT_REPLAY_FRAMES = {
    "map_frame": "odom",
    "base_frame": "base_link",
    "lidar_frame": "rslidar",
    "imu_frame": "rslidar_imu",
    "visualization_frame": "base",
}


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


def console(message: str) -> None:
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {message}", flush=True)


def format_duration(seconds: float | None) -> str:
    if seconds is None or not math.isfinite(seconds):
        return "n/a"
    seconds = max(0.0, seconds)
    minutes, whole_seconds = divmod(int(round(seconds)), 60)
    hours, minutes = divmod(minutes, 60)
    if hours:
        return f"{hours:d}h{minutes:02d}m{whole_seconds:02d}s"
    if minutes:
        return f"{minutes:d}m{whole_seconds:02d}s"
    return f"{whole_seconds:d}s"


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


def ros_parameters_from_config(config_path: Path | None) -> dict[str, Any]:
    if config_path is None:
        return {}

    config = load_yaml_mapping(config_path, label="Config YAML")
    wildcard_node = config.get("/**")
    if isinstance(wildcard_node, dict):
        parameters = wildcard_node.get("ros__parameters")
        if isinstance(parameters, dict):
            return parameters
    return {}


def nested_config_value(mapping: dict[str, Any], dotted_key: str) -> Any:
    current: Any = mapping
    for part in dotted_key.split("."):
        if not isinstance(current, dict) or part not in current:
            return None
        current = current[part]
    return current


def resolve_replay_frames(args: argparse.Namespace, config_path: Path | None) -> dict[str, str]:
    parameters = ros_parameters_from_config(config_path)
    frames: dict[str, str] = {}
    for name, default in DEFAULT_REPLAY_FRAMES.items():
        cli_value = getattr(args, name)
        if cli_value is not None:
            frames[name] = cli_value
            continue

        config_key = f"common.{name}"
        config_value = nested_config_value(parameters, config_key)
        if config_value is None:
            frames[name] = default
            continue
        if not isinstance(config_value, str) or not config_value.strip():
            raise RuntimeError(f"{config_path}: {config_key} must be a non-empty string")
        frames[name] = config_value
    return frames


def normalize_xyz_q_xyzw_transform(value: Any, *, source: str) -> tuple[float, float, float, float, float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 7:
        raise RuntimeError(f"{source} must be [x, y, z, qx, qy, qz, qw], got {value!r}")
    try:
        transform = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"{source} must contain numeric values, got {value!r}") from exc
    if not all(math.isfinite(item) for item in transform):
        raise RuntimeError(f"{source} must contain finite values, got {value!r}")
    qx, qy, qz, qw = transform[3:]
    if math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) <= 1.0e-12:
        raise RuntimeError(f"{source} quaternion must have non-zero norm, got {value!r}")
    return transform  # type: ignore[return-value]


def resolve_estimate_frame_to_gt_transform(
    config_path: Path | None,
    visualization_frame: str,
) -> tuple[tuple[float, float, float, float, float, float, float] | None, str]:
    if config_path is None:
        return None, "none"
    if visualization_frame.strip().lower() != "lidar":
        return None, "not_applicable"

    config_key = "sensor.T_base_lidar_t_xyz_q_xyzw"
    config_value = nested_config_value(ros_parameters_from_config(config_path), config_key)
    if config_value is None:
        return None, "none"
    transform = normalize_xyz_q_xyzw_transform(
        config_value,
        source=f"{config_path}: {config_key}",
    )
    return transform, f"config:{config_key}"


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


def odom_bag_size_mb(odom_dir: Path) -> float:
    total_bytes = 0
    if odom_dir.is_dir():
        for db_path in odom_dir.glob("*.db3"):
            if db_path.is_file():
                total_bytes += db_path.stat().st_size
    return total_bytes / (1024.0 * 1024.0)


def wait_for_play_processes_with_progress(
    play_processes: list[subprocess.Popen[Any]],
    *,
    play_wall_start: float,
    odom_dir: Path,
    progress_interval_s: float,
) -> list[int]:
    if progress_interval_s <= 0.0:
        return [process.wait() for process in play_processes]

    exit_codes: list[int | None] = [None for _process in play_processes]
    next_progress = time.monotonic()
    while True:
        for index, process in enumerate(play_processes):
            if exit_codes[index] is None:
                exit_codes[index] = process.poll()
        if all(code is not None for code in exit_codes):
            return [int(code) for code in exit_codes if code is not None]

        now = time.monotonic()
        if now >= next_progress:
            elapsed_s = max(0.0, time.time() - play_wall_start)
            running = sum(1 for code in exit_codes if code is None)
            console(
                "Replay running: "
                f"elapsed={format_duration(elapsed_s)}, "
                f"odom={odom_bag_size_mb(odom_dir):.1f} MB, "
                f"players_running={running}/{len(play_processes)}"
            )
            next_progress = now + progress_interval_s
        time.sleep(1.0)


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


def benchmark_ape_via_sourced_python(
    prefix: str,
    odom_dir: Path,
    topic: str,
    gt_tum_path: Path,
    output_dir: Path,
    max_association_dt: float,
    rte_delta_s: float,
    estimate_frame_to_gt_transform: tuple[float, float, float, float, float, float, float] | None,
) -> dict[str, Any]:
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "__benchmark_ape",
        "--bag",
        str(odom_dir),
        "--topic",
        topic,
        "--gt-tum",
        str(gt_tum_path),
        "--output-dir",
        str(output_dir),
        "--max-association-dt",
        str(max_association_dt),
        "--rte-delta-s",
        str(rte_delta_s),
    ]
    if estimate_frame_to_gt_transform is not None:
        command.extend(
            [
                "--estimate-frame-to-gt-frame",
                *[f"{value:.17g}" for value in estimate_frame_to_gt_transform],
            ]
        )
    output = run_sourced_capture(
        prefix,
        command,
        timeout=180.0,
    )
    for line in reversed(output.splitlines()):
        line = line.strip()
        if line.startswith("{") and line.endswith("}"):
            return json.loads(line)
    raise RuntimeError(f"Could not find JSON benchmark metrics in analyzer output:\n{output}")


def discover_gt_tum(input_path: Path, input_bags: list[Path], requested_gt_tum: str | None) -> Path | None:
    if requested_gt_tum:
        gt_path = expand_path(requested_gt_tum)
        if not gt_path.is_file():
            raise FileNotFoundError(f"GT TUM file not found: {gt_path}")
        return gt_path

    candidates: list[Path] = []
    if input_path.is_dir():
        candidates.append(input_path / "gt-tum.txt")
        candidates.append(input_path.parent / "gt-tum.txt")
    for bag in input_bags:
        candidates.append(bag / "gt-tum.txt")
        candidates.append(bag.parent / "gt-tum.txt")

    seen: set[Path] = set()
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        if resolved.is_file():
            return resolved
    return None


def add_coverage_metrics(metrics: dict[str, Any], input_metrics: dict[str, Any]) -> None:
    lidar_duration = float(
        input_metrics.get(
            "input_lidar_effective_duration_s",
            input_metrics.get("input_lidar_header_duration_s", 0.0),
        )
        or 0.0
    )
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

    lidar_first_s = input_metrics.get(
        "input_lidar_effective_first_s",
        input_metrics.get("input_lidar_header_first_s"),
    )
    lidar_last_s = input_metrics.get(
        "input_lidar_effective_last_s",
        input_metrics.get("input_lidar_header_last_s"),
    )

    if "odom_header_first_s" in metrics and lidar_first_s is not None:
        metrics["odom_first_lidar_latency_s"] = round(
            metrics["odom_header_first_s"] - lidar_first_s,
            6,
        )
    if "odom_header_last_s" in metrics and lidar_last_s is not None:
        metrics["odom_tail_gap_s"] = round(
            lidar_last_s - metrics["odom_header_last_s"],
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


def finite_float(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def format_report_value(value: Any, suffix: str = "") -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        rendered = f"{value:.6g}"
    else:
        rendered = str(value)
    return rendered + suffix


def format_axis_tick(value: float) -> str:
    abs_value = abs(value)
    if value == 0.0:
        return "0"
    if abs_value >= 1000.0 or abs_value < 0.001:
        return f"{value:.2e}"
    if abs_value >= 10.0:
        return f"{value:.2f}"
    return f"{value:.3f}".rstrip("0").rstrip(".")


def downsample_points(points: list[tuple[float, float]], max_points: int = 1600) -> list[tuple[float, float]]:
    if len(points) <= max_points:
        return points
    step = max(1, math.ceil(len(points) / max_points))
    sampled = points[::step]
    if sampled[-1] != points[-1]:
        sampled.append(points[-1])
    return sampled


def svg_chart(
    title: str,
    x_values: list[Any],
    series: list[tuple[str, str, list[Any]]],
    y_label: str,
    image_path: Path | None = None,
    image_href: str | None = None,
    y_domain: tuple[float, float] | None = None,
) -> str:
    cleaned_series: list[tuple[str, str, list[tuple[float, float]]]] = []
    all_x: list[float] = []
    all_y: list[float] = []
    for label, color, y_values in series:
        points: list[tuple[float, float]] = []
        for raw_x, raw_y in zip(x_values, y_values):
            x_value = finite_float(raw_x)
            y_value = finite_float(raw_y)
            if x_value is None or y_value is None:
                continue
            points.append((x_value, y_value))
        if not points:
            continue
        points = downsample_points(points)
        cleaned_series.append((label, color, points))
        all_x.extend(point[0] for point in points)
        all_y.extend(point[1] for point in points)

    title_html = html_escape(title)
    if not cleaned_series:
        return (
            '<article class="chart empty">'
            f"<h3>{title_html}</h3>"
            "<p>No finite samples available for this chart.</p>"
            "</article>"
        )

    x_min = min(all_x)
    x_max = max(all_x)
    if y_domain is not None:
        y_min, y_max = y_domain
    else:
        y_min = min(all_y)
        y_max = max(all_y)
    if math.isclose(x_min, x_max):
        x_min -= 0.5
        x_max += 0.5
    if math.isclose(y_min, y_max):
        pad = max(abs(y_min) * 0.05, 1.0)
        y_min -= pad
        y_max += pad
    else:
        pad = (y_max - y_min) * 0.08
        y_min -= pad
        y_max += pad

    width = 920
    height = 260
    left = 72
    right = 18
    top = 20
    bottom = 46
    plot_w = width - left - right
    plot_h = height - top - bottom

    def sx(value: float) -> float:
        return left + (value - x_min) / (x_max - x_min) * plot_w

    def sy(value: float) -> float:
        return top + (y_max - value) / (y_max - y_min) * plot_h

    svg_lines = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
            f'role="img" aria-label="{title_html}">'
        ),
        (
            "<style>"
            ".plot-bg{fill:#fff}.grid{stroke:#e7ebf1;stroke-width:1}"
            ".axis{stroke:#9aa4b2;stroke-width:1.2}"
            ".tick,.axis-label{fill:#5d6978;font:11px -apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif}"
            "</style>"
        ),
        f'<rect x="0" y="0" width="{width}" height="{height}" class="plot-bg" />',
    ]
    for index in range(6):
        x_tick = x_min + (x_max - x_min) * index / 5.0
        x_px = sx(x_tick)
        svg_lines.append(f'<line x1="{x_px:.2f}" y1="{top}" x2="{x_px:.2f}" y2="{top + plot_h}" class="grid" />')
        svg_lines.append(
            f'<text x="{x_px:.2f}" y="{height - 18}" text-anchor="middle" class="tick">'
            f"{html_escape(format_axis_tick(x_tick))}</text>"
        )
    for index in range(5):
        y_tick = y_min + (y_max - y_min) * index / 4.0
        y_px = sy(y_tick)
        svg_lines.append(f'<line x1="{left}" y1="{y_px:.2f}" x2="{left + plot_w}" y2="{y_px:.2f}" class="grid" />')
        svg_lines.append(
            f'<text x="{left - 10}" y="{y_px + 4:.2f}" text-anchor="end" class="tick">'
            f"{html_escape(format_axis_tick(y_tick))}</text>"
        )
    svg_lines.extend(
        [
            f'<line x1="{left}" y1="{top + plot_h}" x2="{left + plot_w}" y2="{top + plot_h}" class="axis" />',
            f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_h}" class="axis" />',
            f'<text x="{left + plot_w / 2.0:.2f}" y="{height - 3}" text-anchor="middle" class="axis-label">time (s)</text>',
            (
                f'<text transform="translate(16 {top + plot_h / 2.0:.2f}) rotate(-90)" '
                f'text-anchor="middle" class="axis-label">{html_escape(y_label)}</text>'
            ),
        ]
    )
    for label, color, points in cleaned_series:
        point_text = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in points)
        svg_lines.append(
            f'<polyline points="{point_text}" fill="none" stroke="{html_escape(color)}" '
            'stroke-width="1.0" stroke-linejoin="round" stroke-linecap="round" />'
        )
    svg_lines.append("</svg>")

    legend = "".join(
        '<span class="legend-item">'
        f'<span class="legend-swatch" style="background:{html_escape(color)}"></span>'
        f"{html_escape(label)}</span>"
        for label, color, _points in cleaned_series
    )
    svg_text = "\n".join(svg_lines)
    if image_path is not None:
        image_path.parent.mkdir(parents=True, exist_ok=True)
        image_path.write_text('<?xml version="1.0" encoding="UTF-8"?>\n' + svg_text + "\n", encoding="utf-8")
        href = image_href if image_href is not None else image_path.name
        href_html = html_escape(href, quote=True)
        return (
            '<article class="chart">'
            f"<h3>{title_html}</h3>"
            f'<a class="chart-link" href="{href_html}" title="Open {title_html}">'
            f'<img class="chart-image" src="{href_html}" alt="{title_html}" loading="lazy" />'
            "</a>"
            f'<div class="legend">{legend}</div>'
            "</article>"
        )

    return (
        '<article class="chart">'
        f"<h3>{title_html}</h3>"
        + svg_text
        + f'<div class="legend">{legend}</div>'
        "</article>"
    )


def finite_xy_points(x_values: list[Any], y_values: list[Any]) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for raw_x, raw_y in zip(x_values, y_values):
        x_value = finite_float(raw_x)
        y_value = finite_float(raw_y)
        if x_value is not None and y_value is not None:
            points.append((x_value, y_value))
    return points


def finite_xyz_points(data: dict[str, Any]) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for raw_x, raw_y, raw_z in zip(
        data.get("x_m", []),
        data.get("y_m", []),
        data.get("z_m", []),
    ):
        x_value = finite_float(raw_x)
        y_value = finite_float(raw_y)
        z_value = finite_float(raw_z)
        if x_value is not None and y_value is not None and z_value is not None:
            points.append((x_value, y_value, z_value))
    return points


def downsample_xyz_points(
    points: list[tuple[float, float, float]],
    max_points: int = 1600,
) -> list[tuple[float, float, float]]:
    if len(points) <= max_points:
        return points
    step = max(1, math.ceil(len(points) / max_points))
    sampled = points[::step]
    if sampled[-1] != points[-1]:
        sampled.append(points[-1])
    return sampled


def spatial_chart_article(
    title: str,
    svg_lines: list[str],
    legend_items: list[tuple[str, str]],
    image_path: Path | None,
    image_href: str | None,
) -> str:
    title_html = html_escape(title)
    legend = "".join(
        '<span class="legend-item">'
        f'<span class="legend-swatch" style="background:{html_escape(color)}"></span>'
        f"{html_escape(label)}</span>"
        for label, color in legend_items
    )
    svg_text = "\n".join(svg_lines)
    if image_path is not None:
        image_path.parent.mkdir(parents=True, exist_ok=True)
        image_path.write_text('<?xml version="1.0" encoding="UTF-8"?>\n' + svg_text + "\n", encoding="utf-8")
        href = image_href if image_href is not None else image_path.name
        href_html = html_escape(href, quote=True)
        return (
            '<article class="chart">'
            f"<h3>{title_html}</h3>"
            f'<a class="chart-link" href="{href_html}" title="Open {title_html}">'
            f'<img class="chart-image" src="{href_html}" alt="{title_html}" loading="lazy" />'
            "</a>"
            f'<div class="legend">{legend}</div>'
            "</article>"
        )
    return (
        '<article class="chart">'
        f"<h3>{title_html}</h3>"
        + svg_text
        + f'<div class="legend">{legend}</div>'
        "</article>"
    )


def svg_trajectory_xy(
    title: str,
    estimate: dict[str, Any],
    groundtruth: dict[str, Any],
    image_path: Path | None = None,
    image_href: str | None = None,
) -> str:
    series = [
        ("ground truth", "#2ca02c", finite_xy_points(groundtruth.get("x_m", []), groundtruth.get("y_m", []))),
        ("estimate", "#d62728", finite_xy_points(estimate.get("x_m", []), estimate.get("y_m", []))),
    ]
    series = [(label, color, downsample_points(points)) for label, color, points in series if points]
    title_html = html_escape(title)
    if not series:
        return (
            '<article class="chart empty">'
            f"<h3>{title_html}</h3>"
            "<p>No finite samples available for this chart.</p>"
            "</article>"
        )

    width = 920
    height = 460
    left = 72
    right = 22
    top = 24
    bottom = 54
    all_x = [x for _label, _color, points in series for x, _y in points]
    all_y = [y for _label, _color, points in series for _x, y in points]
    x_min = min(all_x)
    x_max = max(all_x)
    y_min = min(all_y)
    y_max = max(all_y)
    if math.isclose(x_min, x_max):
        x_min -= 0.5
        x_max += 0.5
    if math.isclose(y_min, y_max):
        y_min -= 0.5
        y_max += 0.5

    plot_w = width - left - right
    plot_h = height - top - bottom
    x_center = (x_min + x_max) * 0.5
    y_center = (y_min + y_max) * 0.5
    data_scale = max((x_max - x_min) / plot_w, (y_max - y_min) / plot_h)
    if data_scale <= 0.0:
        data_scale = 1.0
    x_half = data_scale * plot_w * 0.5 * 1.08
    y_half = data_scale * plot_h * 0.5 * 1.08
    x_min = x_center - x_half
    x_max = x_center + x_half
    y_min = y_center - y_half
    y_max = y_center + y_half

    def sx(value: float) -> float:
        return left + (value - x_min) / (x_max - x_min) * plot_w

    def sy(value: float) -> float:
        return top + (y_max - value) / (y_max - y_min) * plot_h

    svg_lines = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
            f'role="img" aria-label="{title_html}">'
        ),
        (
            "<style>"
            ".plot-bg{fill:#fff}.grid{stroke:#e7ebf1;stroke-width:1}"
            ".axis{stroke:#9aa4b2;stroke-width:1.2}"
            ".tick,.axis-label{fill:#5d6978;font:11px -apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif}"
            ".marker{stroke:#111827;stroke-width:1.2}"
            "</style>"
        ),
        f'<rect x="0" y="0" width="{width}" height="{height}" class="plot-bg" />',
    ]
    for index in range(6):
        x_tick = x_min + (x_max - x_min) * index / 5.0
        x_px = sx(x_tick)
        svg_lines.append(f'<line x1="{x_px:.2f}" y1="{top}" x2="{x_px:.2f}" y2="{top + plot_h}" class="grid" />')
        svg_lines.append(
            f'<text x="{x_px:.2f}" y="{height - 20}" text-anchor="middle" class="tick">'
            f"{html_escape(format_axis_tick(x_tick))}</text>"
        )
    for index in range(5):
        y_tick = y_min + (y_max - y_min) * index / 4.0
        y_px = sy(y_tick)
        svg_lines.append(f'<line x1="{left}" y1="{y_px:.2f}" x2="{left + plot_w}" y2="{y_px:.2f}" class="grid" />')
        svg_lines.append(
            f'<text x="{left - 10}" y="{y_px + 4:.2f}" text-anchor="end" class="tick">'
            f"{html_escape(format_axis_tick(y_tick))}</text>"
        )
    svg_lines.extend(
        [
            f'<line x1="{left}" y1="{top + plot_h}" x2="{left + plot_w}" y2="{top + plot_h}" class="axis" />',
            f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_h}" class="axis" />',
            f'<text x="{left + plot_w / 2.0:.2f}" y="{height - 4}" text-anchor="middle" class="axis-label">x (m)</text>',
            (
                f'<text transform="translate(16 {top + plot_h / 2.0:.2f}) rotate(-90)" '
                'text-anchor="middle" class="axis-label">y (m)</text>'
            ),
        ]
    )
    for label, color, points in series:
        point_text = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in points)
        svg_lines.append(
            f'<polyline points="{point_text}" fill="none" stroke="{html_escape(color)}" '
            'stroke-width="1.0" stroke-linejoin="round" stroke-linecap="round" />'
        )
        start_x, start_y = points[0]
        end_x, end_y = points[-1]
        svg_lines.append(f'<circle cx="{sx(start_x):.2f}" cy="{sy(start_y):.2f}" r="4" fill="{html_escape(color)}" class="marker" />')
        svg_lines.append(
            f'<rect x="{sx(end_x) - 4:.2f}" y="{sy(end_y) - 4:.2f}" width="8" height="8" '
            f'fill="{html_escape(color)}" class="marker" />'
        )
    svg_lines.append("</svg>")
    return spatial_chart_article(
        title,
        svg_lines,
        [("estimate", "#d62728"), ("ground truth", "#2ca02c")],
        image_path,
        image_href,
    )


def svg_trajectory_xyz(
    title: str,
    estimate: dict[str, Any],
    groundtruth: dict[str, Any],
    image_path: Path | None = None,
    image_href: str | None = None,
) -> str:
    series = [
        ("ground truth", "#2ca02c", downsample_xyz_points(finite_xyz_points(groundtruth))),
        ("estimate", "#d62728", downsample_xyz_points(finite_xyz_points(estimate))),
    ]
    series = [(label, color, points) for label, color, points in series if points]
    title_html = html_escape(title)
    if not series:
        return (
            '<article class="chart empty">'
            f"<h3>{title_html}</h3>"
            "<p>No finite samples available for this chart.</p>"
            "</article>"
        )

    all_x = [x for _label, _color, points in series for x, _y, _z in points]
    all_y = [y for _label, _color, points in series for _x, y, _z in points]
    all_z = [z for _label, _color, points in series for _x, _y, z in points]
    x_center = (min(all_x) + max(all_x)) * 0.5
    y_center = (min(all_y) + max(all_y)) * 0.5
    z_center = (min(all_z) + max(all_z)) * 0.5
    cos30 = math.cos(math.radians(30.0))
    sin30 = math.sin(math.radians(30.0))

    def project(point: tuple[float, float, float]) -> tuple[float, float]:
        x_value, y_value, z_value = point
        x_rel = x_value - x_center
        y_rel = y_value - y_center
        z_rel = z_value - z_center
        return ((x_rel - y_rel) * cos30, (x_rel + y_rel) * sin30 - z_rel)

    projected_series = [
        (label, color, [project(point) for point in points])
        for label, color, points in series
    ]
    projected_axes = [
        ("x", "#9aa4b2", project((x_center + 10.0, y_center, z_center))),
        ("y", "#9aa4b2", project((x_center, y_center + 10.0, z_center))),
        ("z", "#9aa4b2", project((x_center, y_center, z_center + 10.0))),
    ]
    all_u = [u for _label, _color, points in projected_series for u, _v in points]
    all_v = [v for _label, _color, points in projected_series for _u, v in points]
    for _axis, _color, point in projected_axes:
        all_u.append(point[0])
        all_v.append(point[1])
    u_min = min(all_u)
    u_max = max(all_u)
    v_min = min(all_v)
    v_max = max(all_v)
    if math.isclose(u_min, u_max):
        u_min -= 0.5
        u_max += 0.5
    if math.isclose(v_min, v_max):
        v_min -= 0.5
        v_max += 0.5

    width = 920
    height = 460
    left = 32
    right = 32
    top = 24
    bottom = 24
    plot_w = width - left - right
    plot_h = height - top - bottom
    u_center = (u_min + u_max) * 0.5
    v_center = (v_min + v_max) * 0.5
    data_scale = max((u_max - u_min) / plot_w, (v_max - v_min) / plot_h)
    if data_scale <= 0.0:
        data_scale = 1.0
    u_half = data_scale * plot_w * 0.5 * 1.12
    v_half = data_scale * plot_h * 0.5 * 1.12
    u_min = u_center - u_half
    u_max = u_center + u_half
    v_min = v_center - v_half
    v_max = v_center + v_half

    def sx(value: float) -> float:
        return left + (value - u_min) / (u_max - u_min) * plot_w

    def sy(value: float) -> float:
        return top + (v_max - value) / (v_max - v_min) * plot_h

    origin = project((x_center, y_center, z_center))
    svg_lines = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
            f'role="img" aria-label="{title_html}">'
        ),
        (
            "<style>"
            ".plot-bg{fill:#fff}.axis{stroke:#9aa4b2;stroke-width:1.2}"
            ".axis-label{fill:#5d6978;font:11px -apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif}"
            ".marker{stroke:#111827;stroke-width:1.2}"
            "</style>"
        ),
        f'<rect x="0" y="0" width="{width}" height="{height}" class="plot-bg" />',
    ]
    for axis_label, axis_color, axis_point in projected_axes:
        svg_lines.append(
            f'<line x1="{sx(origin[0]):.2f}" y1="{sy(origin[1]):.2f}" '
            f'x2="{sx(axis_point[0]):.2f}" y2="{sy(axis_point[1]):.2f}" '
            f'stroke="{html_escape(axis_color)}" class="axis" />'
        )
        svg_lines.append(
            f'<text x="{sx(axis_point[0]):.2f}" y="{sy(axis_point[1]) - 4:.2f}" '
            f'text-anchor="middle" class="axis-label">{html_escape(axis_label)}</text>'
        )
    for label, color, points in projected_series:
        point_text = " ".join(f"{sx(u):.2f},{sy(v):.2f}" for u, v in points)
        svg_lines.append(
            f'<polyline points="{point_text}" fill="none" stroke="{html_escape(color)}" '
            'stroke-width="1.0" stroke-linejoin="round" stroke-linecap="round" />'
        )
        start_u, start_v = points[0]
        end_u, end_v = points[-1]
        svg_lines.append(f'<circle cx="{sx(start_u):.2f}" cy="{sy(start_v):.2f}" r="4" fill="{html_escape(color)}" class="marker" />')
        svg_lines.append(
            f'<rect x="{sx(end_u) - 4:.2f}" y="{sy(end_v) - 4:.2f}" width="8" height="8" '
            f'fill="{html_escape(color)}" class="marker" />'
        )
    svg_lines.append("</svg>")
    return spatial_chart_article(
        title,
        svg_lines,
        [("estimate", "#d62728"), ("ground truth", "#2ca02c")],
        image_path,
        image_href,
    )


def html_table(rows: list[tuple[str, Any]]) -> str:
    rendered_rows = "\n".join(
        "<tr>"
        f"<th>{html_escape(label)}</th>"
        f"<td>{html_escape(format_report_value(value))}</td>"
        "</tr>"
        for label, value in rows
        if value is not None
    )
    return f"<table>{rendered_rows}</table>" if rendered_rows else "<p>No data.</p>"


def read_benchmark_timeseries(output_dir: Path) -> dict[str, Any] | None:
    path = output_dir / "benchmark_timeseries.json"
    if not path.is_file():
        return None
    try:
        loaded = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None
    return loaded if isinstance(loaded, dict) else None


def write_html_report(
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
    timeseries = read_benchmark_timeseries(output_dir)
    plots_dir = output_dir / "plots"
    if plots_dir.exists():
        shutil.rmtree(plots_dir)
    if timeseries:
        plots_dir.mkdir(parents=True, exist_ok=True)
    status_class = status.lower()

    def card(label: str, value: Any, suffix: str = "") -> str:
        return (
            '<div class="card">'
            f"<span>{html_escape(label)}</span>"
            f"<strong>{html_escape(format_report_value(value, suffix))}</strong>"
            "</div>"
        )

    benchmark_rows = [
        ("GT TUM", metrics.get("benchmark_gt_tum")),
        ("Odometry TUM", metrics.get("benchmark_odom_tum")),
        ("Associated Pairs", metrics.get("benchmark_associated_pairs")),
        ("Max Association dt (s)", metrics.get("benchmark_max_association_dt_s")),
        ("Estimate Frame Transform", metrics.get("benchmark_estimate_frame_transform")),
        ("Estimate Frame Transform Source", metrics.get("benchmark_estimate_frame_transform_source")),
        (
            "Estimate Frame Transform [x y z qx qy qz qw]",
            metrics.get("benchmark_estimate_frame_transform_t_xyz_q_xyzw"),
        ),
        ("ATE RMSE (m)", metrics.get("benchmark_ate_rmse_m", metrics.get("benchmark_ape_rmse_m"))),
        ("ATE P95 (m)", metrics.get("benchmark_ate_p95_m", metrics.get("benchmark_ape_p95_m"))),
        ("ATE Max (m)", metrics.get("benchmark_ate_max_m", metrics.get("benchmark_ape_max_m"))),
        ("ATE Roll RMSE (deg)", metrics.get("benchmark_ate_roll_rmse_deg")),
        ("ATE Pitch RMSE (deg)", metrics.get("benchmark_ate_pitch_rmse_deg")),
        ("ATE Yaw RMSE (deg)", metrics.get("benchmark_ate_yaw_rmse_deg")),
        ("RTE Delta (s)", metrics.get("benchmark_rte_delta_s")),
        ("RTE Count", metrics.get("benchmark_rte_count")),
        ("RTE RMSE (m)", metrics.get("benchmark_rte_rmse_m")),
        ("RTE P95 (m)", metrics.get("benchmark_rte_p95_m")),
        ("RTE Max (m)", metrics.get("benchmark_rte_max_m")),
        ("RTE Roll RMSE (deg)", metrics.get("benchmark_rte_roll_rmse_deg")),
        ("RTE Pitch RMSE (deg)", metrics.get("benchmark_rte_pitch_rmse_deg")),
        ("RTE Yaw RMSE (deg)", metrics.get("benchmark_rte_yaw_rmse_deg")),
    ]

    html_parts = [
        "<!doctype html>",
        '<html lang="en">',
        "<head>",
        '<meta charset="utf-8" />',
        '<meta name="viewport" content="width=device-width, initial-scale=1" />',
        "<title>s-slam Replay Report</title>",
        "<style>",
        """
        :root {
          color-scheme: light;
          --bg: #f6f7f9;
          --panel: #ffffff;
          --ink: #17202a;
          --muted: #5d6978;
          --line: #d9dee7;
          --red: #d62728;
          --green: #2ca02c;
        }
        body {
          margin: 0;
          background: var(--bg);
          color: var(--ink);
          font: 14px/1.45 -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
        }
        main {
          max-width: 1180px;
          margin: 0 auto;
          padding: 28px 20px 56px;
        }
        h1 {
          margin: 0 0 6px;
          font-size: 28px;
          letter-spacing: 0;
        }
        h2 {
          margin: 28px 0 12px;
          font-size: 20px;
          letter-spacing: 0;
        }
        h3 {
          margin: 0 0 10px;
          font-size: 15px;
          letter-spacing: 0;
        }
        .muted, small {
          color: var(--muted);
        }
        .status {
          display: inline-flex;
          align-items: center;
          border-radius: 4px;
          padding: 4px 9px;
          font-weight: 700;
          color: #fff;
          background: #687385;
        }
        .status.pass { background: #18794e; }
        .status.fail { background: #b42318; }
        .grid-cards {
          display: grid;
          grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
          gap: 10px;
          margin: 18px 0 18px;
        }
        .card, .panel, .chart {
          background: var(--panel);
          border: 1px solid var(--line);
          border-radius: 6px;
        }
        .card {
          padding: 12px 14px;
        }
        .card span {
          display: block;
          color: var(--muted);
          font-size: 12px;
          margin-bottom: 4px;
        }
        .card strong {
          font-size: 21px;
        }
        .panel {
          padding: 14px 16px;
          margin: 12px 0;
        }
        table {
          width: 100%;
          border-collapse: collapse;
          background: var(--panel);
          border: 1px solid var(--line);
          border-radius: 6px;
          overflow: hidden;
        }
        th, td {
          border-bottom: 1px solid var(--line);
          padding: 7px 9px;
          text-align: left;
          vertical-align: top;
        }
        th {
          width: 270px;
          color: var(--muted);
          font-weight: 600;
        }
        tr:last-child th, tr:last-child td {
          border-bottom: 0;
        }
        code, pre {
          font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
        }
        pre {
          overflow: auto;
          background: #111827;
          color: #e5e7eb;
          padding: 12px;
          border-radius: 6px;
        }
        .chart-grid {
          display: grid;
          grid-template-columns: repeat(auto-fit, minmax(360px, 1fr));
          gap: 12px;
        }
        .chart {
          padding: 12px;
          overflow: hidden;
        }
        .chart svg {
          width: 100%;
          height: auto;
          display: block;
        }
        .chart-link {
          display: block;
          border: 1px solid var(--line);
          border-radius: 4px;
          overflow: hidden;
          background: #fff;
        }
        .chart-image {
          display: block;
          width: 100%;
          height: auto;
        }
        .plot-bg {
          fill: #fff;
        }
        .grid {
          stroke: #e7ebf1;
          stroke-width: 1;
        }
        .axis {
          stroke: #9aa4b2;
          stroke-width: 1.2;
        }
        .tick, .axis-label {
          fill: #5d6978;
          font-size: 11px;
        }
        .legend {
          display: flex;
          flex-wrap: wrap;
          gap: 12px;
          margin-top: 8px;
          color: var(--muted);
          font-size: 12px;
        }
        .legend-item {
          display: inline-flex;
          gap: 6px;
          align-items: center;
        }
        .legend-swatch {
          width: 18px;
          height: 3px;
          display: inline-block;
        }
        ul {
          margin: 8px 0 0;
          padding-left: 20px;
        }
        a {
          color: #1455d9;
        }
        @media (max-width: 640px) {
          main { padding: 18px 12px 40px; }
          .chart-grid { grid-template-columns: 1fr; }
          th { width: 42%; }
        }
        """,
        "</style>",
        "</head>",
        "<body>",
        "<main>",
        "<header>",
        "<h1>s-slam Replay Report</h1>",
        (
            f'<div><span class="status {html_escape(status_class)}">{html_escape(status)}</span> '
            f'<span class="muted">generated {html_escape(datetime.now().isoformat(timespec="seconds"))}</span></div>'
        ),
        "</header>",
        '<section class="grid-cards">',
        card("ATE RMSE", metrics.get("benchmark_ate_rmse_m", metrics.get("benchmark_ape_rmse_m")), " m"),
        card("RTE RMSE", metrics.get("benchmark_rte_rmse_m"), " m"),
        card("Odom Coverage", metrics.get("odom_coverage_ratio")),
        card("Odom Count", metrics.get("odom_count")),
        "</section>",
        '<section class="panel">',
        "<h2>Run</h2>",
        html_table(
            [
                ("Output", str(output_dir)),
                ("HTML Report", "report.html"),
                ("Metrics JSON", "metrics.json"),
                ("Benchmark Timeseries", "benchmark_timeseries.json" if timeseries else None),
                ("Plot Images", "plots/*.svg" if timeseries else None),
                *[(f"Input Bag {index + 1}", str(bag)) for index, bag in enumerate(input_bags)],
                *[(key, value) for key, value in replay_details.items()],
            ]
        ),
        "</section>",
        '<section class="panel">',
        "<h2>Status</h2>",
        "<strong>Reasons</strong>",
        "<ul>",
        *[f"<li>{html_escape(reason)}</li>" for reason in status_reasons],
        "</ul>",
        "<strong>Quality Warnings</strong>",
        "<ul>",
        *[
            f"<li>{html_escape(warning)}</li>"
            for warning in (quality_warnings if quality_warnings else ["none"])
        ],
        "</ul>",
        "</section>",
        '<section class="panel">',
        "<h2>Benchmark Metrics</h2>",
    ]
    if metrics.get("benchmark_status") == "evaluated":
        html_parts.append(html_table(benchmark_rows))
    else:
        html_parts.append(
            f"<p>Skipped: {html_escape(str(metrics.get('benchmark_skip_reason', 'no ground truth provided')))}</p>"
        )
    html_parts.append("</section>")

    if timeseries:
        time_s = timeseries.get("time_s", [])
        estimate = timeseries.get("estimate", {})
        groundtruth = timeseries.get("groundtruth", {})
        ate_error = timeseries.get("ate_error", {})
        components = [
            ("x", "x_m", "m"),
            ("y", "y_m", "m"),
            ("z", "z_m", "m"),
            ("yaw", "yaw_deg", "deg"),
            ("roll", "roll_deg", "deg"),
            ("pitch", "pitch_deg", "deg"),
        ]

        def plot_path(name: str) -> tuple[Path, str]:
            filename = f"{name}.svg"
            return plots_dir / filename, f"plots/{filename}"

        html_parts.extend(["<h2>ATE Trajectory: Estimate vs Ground Truth</h2>", '<section class="chart-grid">'])
        image_path, image_href = plot_path("ate_trajectory_xy")
        html_parts.append(
            svg_trajectory_xy(
                "ATE trajectory XY",
                estimate,
                groundtruth,
                image_path,
                image_href,
            )
        )
        image_path, image_href = plot_path("ate_trajectory_xyz")
        html_parts.append(
            svg_trajectory_xyz(
                "ATE trajectory XYZ isometric",
                estimate,
                groundtruth,
                image_path,
                image_href,
            )
        )
        html_parts.append("</section>")

        html_parts.extend(["<h2>ATE 6DoF Pose: Estimate vs Ground Truth</h2>", '<section class="chart-grid">'])
        for title, key, unit in components:
            image_path, image_href = plot_path(f"ate_pose_{title}")
            html_parts.append(
                svg_chart(
                    f"ATE pose {title}",
                    time_s,
                    [
                        ("estimate", "#d62728", estimate.get(key, [])),
                        ("ground truth", "#2ca02c", groundtruth.get(key, [])),
                    ],
                    f"{title} ({unit})",
                    image_path,
                    image_href,
                )
            )
        html_parts.append("</section>")

        html_parts.extend(["<h2>ATE 6DoF Error</h2>", '<section class="chart-grid">'])
        for title, key, unit in components:
            image_path, image_href = plot_path(f"ate_error_{title}")
            html_parts.append(
                svg_chart(
                    f"ATE error {title}",
                    time_s,
                    [("estimate - ground truth", "#d62728", ate_error.get(key, []))],
                    f"error ({unit})",
                    image_path,
                    image_href,
                )
            )
        html_parts.append("</section>")

        rte = timeseries.get("rte", {})
        rte_time_s = rte.get("time_s", [])
        rte_estimate = rte.get("estimate_delta", {})
        rte_groundtruth = rte.get("groundtruth_delta", {})
        rte_error = rte.get("error", {})
        if rte_time_s:
            html_parts.extend(["<h2>RTE 6DoF Relative Motion: Estimate vs Ground Truth</h2>", '<section class="chart-grid">'])
            for title, key, unit in components:
                image_path, image_href = plot_path(f"rte_delta_{title}")
                html_parts.append(
                    svg_chart(
                        f"RTE delta {title}",
                        rte_time_s,
                        [
                            ("estimate", "#d62728", rte_estimate.get(key, [])),
                            ("ground truth", "#2ca02c", rte_groundtruth.get(key, [])),
                        ],
                        f"delta {title} ({unit})",
                        image_path,
                        image_href,
                    )
                )
            html_parts.append("</section>")

            html_parts.extend(["<h2>RTE 6DoF Error</h2>", '<section class="chart-grid">'])
            for title, key, unit in components:
                image_path, image_href = plot_path(f"rte_error_{title}")
                html_parts.append(
                    svg_chart(
                        f"RTE error {title}",
                        rte_time_s,
                        [("relative error", "#d62728", rte_error.get(key, []))],
                        f"error ({unit})",
                        image_path,
                        image_href,
                    )
                )
            html_parts.append("</section>")
    else:
        html_parts.extend(
            [
                '<section class="panel">',
                "<h2>Trajectory Charts</h2>",
                "<p>No benchmark timeseries was generated. Provide a GT TUM file to enable ATE/RTE charts.</p>",
                "</section>",
            ]
        )

    html_parts.extend(
        [
            '<section class="panel">',
            "<h2>Odometry Metrics</h2>",
            html_table(
                [
                    ("odom_count", metrics.get("odom_count")),
                    ("odom_rate_hz", metrics.get("odom_rate_hz")),
                    ("odom_header_duration_s", metrics.get("odom_header_duration_s")),
                    ("odom_max_pos_norm_m", metrics.get("odom_max_pos_norm_m")),
                    ("odom_final_pos_norm_m", metrics.get("odom_final_pos_norm_m")),
                    ("odom_path_length_m", metrics.get("odom_path_length_m")),
                    ("odom_max_step_m", metrics.get("odom_max_step_m")),
                    ("odom_large_steps", metrics.get("odom_large_steps")),
                    ("odom_frozen_ratio", metrics.get("odom_frozen_ratio")),
                    ("odom_sequence_hash", metrics.get("odom_sequence_hash")),
                ]
            ),
            "</section>",
            '<section class="panel">',
            "<h2>Frontend Log Signals</h2>",
            html_table([(key, value) for key, value in log_metrics.items()]),
            "</section>",
            '<section class="panel">',
            "<h2>Resource Usage</h2>",
            html_table([(key, value) for key, value in resource_metrics.items()]),
            "</section>",
            '<section class="panel">',
            "<h2>Process Exit Codes</h2>",
            "<pre>",
            html_escape(json.dumps(exit_codes, indent=2, sort_keys=True)),
            "</pre>",
            "</section>",
            '<section class="panel">',
            "<h2>Input Bag Info</h2>",
            "<pre>",
            html_escape(bag_info.strip()),
            "</pre>",
            "</section>",
            "</main>",
            "</body>",
            "</html>",
        ]
    )

    (output_dir / "report.html").write_text("\n".join(html_parts) + "\n", encoding="utf-8")


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
        f"- HTML report: `{output_dir / 'report.html'}`",
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
        "input_lidar_coverage_time_source",
        "input_lidar_effective_duration_s",
        "input_lidar_effective_rate_hz",
        "input_lidar_header_duration_s",
        "input_lidar_header_rate_hz",
        "input_lidar_zero_header_count",
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
        "odom_path_length_m",
        "odom_endpoint_path_ratio",
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

    if metrics.get("benchmark_status") == "evaluated":
        report_lines.extend(
            [
                "",
                "## Benchmark Against Ground Truth",
                "",
                "| Metric | Value |",
                "|---|---:|",
                f"| `gt_tum` | `{metrics['benchmark_gt_tum']}` |",
                f"| `odom_tum` | `{metrics['benchmark_odom_tum']}` |",
                f"| `associated_pairs` | `{metrics['benchmark_associated_pairs']}` |",
                f"| `est_count` | `{metrics['benchmark_est_count']}` |",
                f"| `gt_count` | `{metrics['benchmark_gt_count']}` |",
                f"| `max_abs_time_diff_s` | `{metrics['benchmark_max_abs_time_diff_s']}` |",
                f"| `est_length_m` | `{metrics['benchmark_est_length_m']}` |",
                f"| `gt_length_m` | `{metrics['benchmark_gt_length_m']}` |",
                f"| `estimate_frame_transform` | `{metrics.get('benchmark_estimate_frame_transform', 'none')}` |",
                f"| `estimate_frame_transform_source` | `{metrics.get('benchmark_estimate_frame_transform_source', 'n/a')}` |",
                f"| `estimate_frame_transform_t_xyz_q_xyzw` | `{metrics.get('benchmark_estimate_frame_transform_t_xyz_q_xyzw', 'n/a')}` |",
                f"| `ate_rmse_m` | `{metrics.get('benchmark_ate_rmse_m', metrics['benchmark_ape_rmse_m'])}` |",
                f"| `ate_mean_m` | `{metrics.get('benchmark_ate_mean_m', metrics['benchmark_ape_mean_m'])}` |",
                f"| `ate_median_m` | `{metrics.get('benchmark_ate_median_m', metrics['benchmark_ape_median_m'])}` |",
                f"| `ate_p95_m` | `{metrics.get('benchmark_ate_p95_m', metrics['benchmark_ape_p95_m'])}` |",
                f"| `ate_max_m` | `{metrics.get('benchmark_ate_max_m', metrics['benchmark_ape_max_m'])}` |",
                f"| `ate_roll_rmse_deg` | `{metrics.get('benchmark_ate_roll_rmse_deg', 'n/a')}` |",
                f"| `ate_pitch_rmse_deg` | `{metrics.get('benchmark_ate_pitch_rmse_deg', 'n/a')}` |",
                f"| `ate_yaw_rmse_deg` | `{metrics.get('benchmark_ate_yaw_rmse_deg', 'n/a')}` |",
                f"| `rte_delta_s` | `{metrics.get('benchmark_rte_delta_s', 'n/a')}` |",
                f"| `rte_count` | `{metrics.get('benchmark_rte_count', 'n/a')}` |",
                f"| `rte_rmse_m` | `{metrics.get('benchmark_rte_rmse_m', 'n/a')}` |",
                f"| `rte_p95_m` | `{metrics.get('benchmark_rte_p95_m', 'n/a')}` |",
                f"| `rte_max_m` | `{metrics.get('benchmark_rte_max_m', 'n/a')}` |",
                f"| `rte_roll_rmse_deg` | `{metrics.get('benchmark_rte_roll_rmse_deg', 'n/a')}` |",
                f"| `rte_pitch_rmse_deg` | `{metrics.get('benchmark_rte_pitch_rmse_deg', 'n/a')}` |",
                f"| `rte_yaw_rmse_deg` | `{metrics.get('benchmark_rte_yaw_rmse_deg', 'n/a')}` |",
                f"| `ape_rmse_m` | `{metrics['benchmark_ape_rmse_m']}` |",
                f"| `ape_mean_m` | `{metrics['benchmark_ape_mean_m']}` |",
                f"| `ape_median_m` | `{metrics['benchmark_ape_median_m']}` |",
                f"| `ape_p95_m` | `{metrics['benchmark_ape_p95_m']}` |",
                f"| `ape_max_m` | `{metrics['benchmark_ape_max_m']}` |",
                f"| `endpoint_error_m` | `{metrics['benchmark_endpoint_error_m']}` |",
            ]
        )
    elif metrics.get("benchmark_status") == "skipped":
        report_lines.extend(
            [
                "",
                "## Benchmark Against Ground Truth",
                "",
                f"Skipped: {metrics.get('benchmark_skip_reason', 'no ground truth provided')}.",
            ]
        )
        report_lines.extend(
            [
                "",
                "## Odometry-Only Drift Summary",
                "",
                "No ground truth trajectory was found. The values below are relative "
                "to the first odometry pose; `final_displacement_from_start_m` is a "
                "drift proxy only for static or return-to-start bags.",
                "",
                "| Metric | Value |",
                "|---|---:|",
                f"| `final_displacement_from_start_m` | `{metrics['odom_final_pos_norm_m']}` |",
                f"| `final_displacement_xyz_m` | `{metrics.get('odom_final_displacement_xyz_m', [])}` |",
                f"| `max_displacement_from_start_m` | `{metrics['odom_max_pos_norm_m']}` |",
                f"| `estimated_path_length_m` | `{metrics.get('odom_path_length_m', 0.0)}` |",
                f"| `endpoint_to_path_ratio` | `{metrics.get('odom_endpoint_path_ratio', 0.0)}` |",
                f"| `max_frame_step_m` | `{metrics['odom_max_step_m']}` |",
                f"| `odom_count` | `{metrics['odom_count']}` |",
                f"| `odom_rate_hz` | `{metrics['odom_rate_hz']}` |",
            ]
        )

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
    write_html_report(
        output_dir,
        input_bags,
        bag_info,
        replay_details,
        metrics,
        log_metrics,
        exit_codes,
        resource_metrics,
    )


def run_replay(args: argparse.Namespace) -> int:
    input_path = expand_path(args.input)
    output_dir = expand_path(args.output)
    console(f"Replay requested: input={input_path}")
    console(f"Output directory: {output_dir}")
    # Keep the lock file handle alive for the whole replay. Concurrent replays
    # share ROS topic names and can kill or mix each other's bag players.
    console("Acquiring replay lock")
    _run_lock = acquire_run_lock(DEFAULT_RUN_LOCK_TIMEOUT_S)
    ros_setup = expand_path(args.ros_setup)
    workspace_setup = expand_path(args.workspace_setup)
    config_path = resolve_config_path(args.config)
    replay_frames = resolve_replay_frames(args, config_path)
    for name, value in replay_frames.items():
        setattr(args, name, value)
    estimate_frame_to_gt_transform, estimate_frame_to_gt_transform_source = (
        resolve_estimate_frame_to_gt_transform(config_path, args.visualization_frame)
    )
    input_bags = discover_input_bags(input_path)

    prepare_output_dir(output_dir)
    logs_dir = output_dir / "logs"
    odom_dir = output_dir / "odom"
    prefix = source_prefix(ros_setup, workspace_setup)

    console("Inspecting input bag metadata")
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
    console(f"Selected topics: lidar={lidar_topic}, imu={imu_topic}, odom={args.odom_topic}")

    if not args.no_cleanup:
        console("Cleaning old replay/frontend processes")
        cleanup_old_processes()
    console("Waiting for stale publishers to disappear")
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
    console(f"Starting frontend: {args.frontend_launch}")
    frontend = start_sourced_process(prefix, frontend_args, logs_dir / "frontend.log")
    bag_play_schedule = build_bag_play_schedule(input_bags)
    console(
        "Input bags ready: "
        f"count={len(input_bags)}, "
        f"replay_rate={args.replay_rate:g}x"
    )

    recorder: subprocess.Popen[Any] | None = None
    play_processes: list[subprocess.Popen[Any]] = []
    endpoint_monitor: EndpointMonitor | None = None
    exit_codes: dict[str, Any] = {}
    drain_wait_s = 0.0
    monitor = ResourceMonitor(frontend.pid)
    try:
        console("Waiting for frontend readiness")
        wait_for_frontend(
            prefix,
            frontend,
            args.startup_timeout,
            lidar_topic=lidar_topic,
            imu_topic=imu_topic,
            odom_topic=args.odom_topic,
        )
        console("Frontend ready")
        time.sleep(args.startup_wait)

        recorder_args = ["ros2", "bag", "record", args.odom_topic, "-o", str(odom_dir)]
        console(f"Starting odometry recorder: {odom_dir}")
        recorder = start_sourced_process(prefix, recorder_args, logs_dir / "record_odom.log")
        wait_for_topic_subscription(
            prefix,
            args.odom_topic,
            DEFAULT_RECORD_START_TIMEOUT_S,
            expected_nodes={"/rosbag2_recorder"},
        )
        console("Odometry recorder ready")
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
            console(
                "Starting bag player: "
                f"bag={bag.name}, topics={','.join(topics_for_bag)}, node={player_node}"
            )
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
        console("Bag players are ready")
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
        console("Bag player publishers verified")

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
        console("Bag playback resumed")

        exit_codes["play"] = wait_for_play_processes_with_progress(
            play_processes,
            play_wall_start=play_wall_start,
            odom_dir=odom_dir,
            progress_interval_s=DEFAULT_PROGRESS_INTERVAL_S,
        )
        console(f"Bag playback complete: exit_codes={exit_codes['play']}")
        if endpoint_monitor is not None:
            endpoint_monitor.stop()
        time.sleep(args.settle_time)
        # Let the frontend chew through any buffered backlog before teardown;
        # a fixed settle window truncates the processed frame set in a
        # load-dependent way and makes the report non-reproducible.
        console("Waiting for odometry topic drain")
        drain_wait_s = wait_for_topic_drain(prefix, args.odom_topic)
        console(f"Odometry topic drained: waited={format_duration(drain_wait_s)}")
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

    console("Analyzing recorded odometry")
    metrics = analyze_odom_via_sourced_python(
        prefix,
        odom_dir,
        args.odom_topic,
        args.large_step_threshold,
        args.frozen_eps,
    )
    console("Analyzing input bag coverage")
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
    gt_tum_path = discover_gt_tum(input_path, input_bags, args.gt_tum)
    if gt_tum_path is not None:
        console(f"Benchmarking against ground truth: {gt_tum_path}")
        benchmark_metrics = benchmark_ape_via_sourced_python(
            prefix,
            odom_dir,
            args.odom_topic,
            gt_tum_path,
            output_dir,
            args.max_association_dt,
            args.rte_delta_s,
            estimate_frame_to_gt_transform,
        )
        metrics.update(benchmark_metrics)
        metrics["benchmark_estimate_frame_transform_source"] = estimate_frame_to_gt_transform_source
    else:
        console("Ground truth not found; skipping benchmark")
        metrics["benchmark_status"] = "skipped"
        metrics["benchmark_skip_reason"] = "no gt-tum.txt found near the input bag"
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
        "benchmark_estimate_frame_to_gt_transform": (
            list(estimate_frame_to_gt_transform)
            if estimate_frame_to_gt_transform is not None
            else None
        ),
        "benchmark_estimate_frame_to_gt_transform_source": estimate_frame_to_gt_transform_source,
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
        "gt_tum": str(gt_tum_path) if gt_tum_path is not None else None,
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
    console("Writing replay report")
    write_report(
        output_dir,
        input_bags,
        bag_info_text,
        {
            "config": str(config_path) if config_path is not None else "launch default",
            "lidar_topic": lidar_topic,
            "imu_topic": imu_topic,
            "benchmark_estimate_frame_to_gt_transform": (
                list(estimate_frame_to_gt_transform)
                if estimate_frame_to_gt_transform is not None
                else None
            ),
            "benchmark_estimate_frame_to_gt_transform_source": estimate_frame_to_gt_transform_source,
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
    print(f"Report HTML: {output_dir / 'report.html'}")
    print(f"Report Markdown: {output_dir / 'report.md'}")
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
    if metrics.get("benchmark_status") == "evaluated":
        rte_summary = (
            f", rte_rmse={metrics['benchmark_rte_rmse_m']:.4f} m"
            if "benchmark_rte_rmse_m" in metrics
            else f", rte={metrics.get('benchmark_rte_status', 'skipped')}"
        )
        print(
            "Benchmark: "
            f"ate_rmse={metrics['benchmark_ate_rmse_m']:.4f} m, "
            f"ate_max={metrics['benchmark_ate_max_m']:.4f} m"
            f"{rte_summary}, "
            f"endpoint={metrics['benchmark_endpoint_error_m']:.4f} m, "
            f"pairs={metrics['benchmark_associated_pairs']}"
        )
    else:
        print(f"Benchmark: skipped ({metrics.get('benchmark_skip_reason', 'no ground truth')})")
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
    span = max_value - min_value

    # RoboSense Fairy publishes absolute per-point seconds in `timestamp`.
    # Other datasets commonly publish relative scan offsets in seconds or ms.
    if min_value > header_time_s - 1.0 and max_value < header_time_s + 2.0:
        return min_value, max_value, f"absolute:{time_field.name}"
    # Some RoboSense SDK warmup frames have header.stamp=0 while the per-point
    # `timestamp` field is still a valid absolute LiDAR-clock second. Detect that
    # by the scan-like span instead of anchoring only to the bad header stamp.
    if min_value > 1000.0 and 0.001 <= span <= 1.0:
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
    lidar_zero_header_count = 0

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
                if header_time <= 0.0:
                    lidar_zero_header_count += 1
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
        header_rate_hz = round(topic_rate(count, header_duration), 6)
        metrics[f"{prefix}_header_rate_hz"] = header_rate_hz
        metrics[f"{prefix}_rate_hz"] = header_rate_hz
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
        lidar_effective_first_s = min(scan_begin_s for scan_begin_s, _scan_end_s in lidar_time_bounds)
        lidar_effective_last_s = max(scan_end_s for _scan_begin_s, scan_end_s in lidar_time_bounds)
        lidar_effective_duration_s = lidar_effective_last_s - lidar_effective_first_s
        metrics["input_lidar_coverage_time_source"] = "point_timestamp"
        metrics["input_lidar_effective_first_s"] = round(lidar_effective_first_s, 6)
        metrics["input_lidar_effective_last_s"] = round(lidar_effective_last_s, 6)
        metrics["input_lidar_effective_duration_s"] = round(lidar_effective_duration_s, 6)
        metrics["input_lidar_effective_rate_hz"] = round(
            topic_rate(len(lidar_time_bounds), lidar_effective_duration_s),
            6,
        )
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
    else:
        metrics["input_lidar_coverage_time_source"] = "header_stamp"
    metrics["input_lidar_zero_header_count"] = lidar_zero_header_count
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
    path_length = 0.0
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
        path_length += step
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
        "odom_final_displacement_xyz_m": [round(value, 6) for value in rel_positions[-1]],
        "odom_max_pos_norm_m": round(max_norm, 6),
        "odom_final_pos_norm_m": round(final_norm, 6),
        "odom_path_length_m": round(path_length, 6),
        "odom_endpoint_path_ratio": round(final_norm / path_length, 6) if path_length > 0.0 else 0.0,
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


def benchmark_ape(args: argparse.Namespace) -> int:
    import numpy as np
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message

    odom_bag = expand_path(args.bag)
    gt_tum = expand_path(args.gt_tum)
    output_dir = expand_path(args.output_dir)
    odom_tum = output_dir / "odom_tum.txt"
    benchmark_json = output_dir / "benchmark_ape.json"
    timeseries_json = output_dir / "benchmark_timeseries.json"

    if not has_metadata(odom_bag):
        raise FileNotFoundError(f"Odometry bag metadata.yaml not found: {odom_bag}")
    if not gt_tum.is_file():
        raise FileNotFoundError(f"GT TUM file not found: {gt_tum}")
    if args.rte_delta_s <= 0.0:
        raise ValueError("--rte-delta-s must be positive")

    def quat_xyzw_to_rot(qx: float, qy: float, qz: float, qw: float):
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1.0e-12:
            return np.eye(3, dtype=np.float64)
        x = qx / norm
        y = qy / norm
        z = qz / norm
        w = qw / norm
        return np.asarray(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

    def rot_to_rpy_deg(rotation_matrix: Any):
        sy = math.sqrt(
            float(rotation_matrix[0, 0] * rotation_matrix[0, 0] + rotation_matrix[1, 0] * rotation_matrix[1, 0])
        )
        if sy >= 1.0e-9:
            roll = math.atan2(float(rotation_matrix[2, 1]), float(rotation_matrix[2, 2]))
            pitch = math.atan2(float(-rotation_matrix[2, 0]), sy)
            yaw = math.atan2(float(rotation_matrix[1, 0]), float(rotation_matrix[0, 0]))
        else:
            roll = math.atan2(float(-rotation_matrix[1, 2]), float(rotation_matrix[1, 1]))
            pitch = math.atan2(float(-rotation_matrix[2, 0]), sy)
            yaw = 0.0
        return np.asarray(
            [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)],
            dtype=np.float64,
        )

    def rot_angle_deg(rotation_matrix: Any) -> float:
        cosine = (float(np.trace(rotation_matrix)) - 1.0) * 0.5
        cosine = max(-1.0, min(1.0, cosine))
        return math.degrees(math.acos(cosine))

    def relative_pose(rot0: Any, pos0: Any, rot1: Any, pos1: Any) -> tuple[Any, Any]:
        rel_rot = rot0.T @ rot1
        rel_pos = rot0.T @ (pos1 - pos0)
        return rel_rot, rel_pos

    def rounded_array(values: Any, digits: int = 6) -> list[float]:
        return [round(float(value), digits) for value in np.asarray(values, dtype=np.float64).reshape(-1)]

    def apply_estimate_frame_transform(xyz: Any, rotations: Any, transform: list[float]):
        t_gt_est = np.asarray(transform[:3], dtype=np.float64)
        r_gt_est = quat_xyzw_to_rot(*transform[3:])
        # T_gt_est maps coordinates from the odometry child frame into the GT
        # body frame. Pose rows are T_world_est, so compare T_world_gt =
        # T_world_est * inverse(T_gt_est).
        r_est_gt = r_gt_est.T
        t_est_gt = -r_est_gt @ t_gt_est
        transformed_xyz = xyz + np.einsum("nij,j->ni", rotations, t_est_gt)
        transformed_rotations = np.einsum("nij,jk->nik", rotations, r_est_gt)
        return transformed_xyz, transformed_rotations

    def pose_component_dict(xyz: Any, rpy_deg: Any) -> dict[str, list[float]]:
        return {
            "x_m": rounded_array(xyz[:, 0]),
            "y_m": rounded_array(xyz[:, 1]),
            "z_m": rounded_array(xyz[:, 2]),
            "roll_deg": rounded_array(rpy_deg[:, 0]),
            "pitch_deg": rounded_array(rpy_deg[:, 1]),
            "yaw_deg": rounded_array(rpy_deg[:, 2]),
        }

    def error_component_dict(xyz: Any, rpy_deg: Any, trans_norm: Any, rot_angle: Any) -> dict[str, list[float]]:
        result = pose_component_dict(xyz, rpy_deg)
        result["trans_norm_m"] = rounded_array(trans_norm)
        result["rot_angle_deg"] = rounded_array(rot_angle)
        return result

    def add_scalar_stats(target: dict[str, Any], prefix: str, unit: str, values: Any) -> None:
        arr = np.asarray(values, dtype=np.float64).reshape(-1)
        if arr.size == 0:
            return
        target[f"{prefix}_rmse_{unit}"] = round(float(np.sqrt(np.mean(arr * arr))), 6)
        target[f"{prefix}_mean_{unit}"] = round(float(np.mean(arr)), 6)
        target[f"{prefix}_median_{unit}"] = round(float(np.median(arr)), 6)
        target[f"{prefix}_std_{unit}"] = round(float(np.std(arr)), 6)
        target[f"{prefix}_p95_{unit}"] = round(float(np.percentile(arr, 95)), 6)
        target[f"{prefix}_max_{unit}"] = round(float(np.max(arr)), 6)

    def add_component_stats(
        target: dict[str, Any],
        prefix: str,
        unit: str,
        names: tuple[str, str, str],
        values: Any,
    ) -> None:
        arr = np.asarray(values, dtype=np.float64)
        if arr.size == 0:
            return
        for index, name in enumerate(names):
            component = arr[:, index]
            target[f"{prefix}_{name}_rmse_{unit}"] = round(float(np.sqrt(np.mean(component * component))), 6)
            target[f"{prefix}_{name}_mean_{unit}"] = round(float(np.mean(component)), 6)
            target[f"{prefix}_{name}_max_abs_{unit}"] = round(float(np.max(np.abs(component))), 6)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(odom_bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if args.topic not in topic_types:
        available = ", ".join(sorted(topic_types))
        raise RuntimeError(f"Topic {args.topic!r} not found. Available topics: {available}")
    if topic_types[args.topic] != "nav_msgs/msg/Odometry":
        raise RuntimeError(
            f"Topic {args.topic!r} has type {topic_types[args.topic]!r}, expected nav_msgs/msg/Odometry"
        )

    est_rows: list[tuple[float, float, float, float, float, float, float, float]] = []
    while reader.has_next():
        topic_name, data, _timestamp_ns = reader.read_next()
        if topic_name != args.topic:
            continue
        msg = deserialize_message(data, Odometry)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        row = (stamp, pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w)
        if stamp > 0.0 and all(math.isfinite(value) for value in row):
            est_rows.append(row)

    if not est_rows:
        raise RuntimeError("No valid odometry samples found for benchmark")
    odom_tum.write_text(
        "\n".join(
            "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f" % row
            for row in est_rows
        )
        + "\n",
        encoding="utf-8",
    )

    gt_rows: list[tuple[float, float, float, float, float, float, float, float]] = []
    for line in gt_tum.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        fields = line.split()
        if len(fields) < 8:
            continue
        try:
            values = tuple(float(item) for item in fields[:8])
        except ValueError:
            continue
        if len(values) == 8 and all(math.isfinite(value) for value in values):
            gt_rows.append(values)  # type: ignore[arg-type]
    if not gt_rows:
        raise RuntimeError(f"No valid TUM poses found in {gt_tum}")

    est = np.asarray(est_rows, dtype=np.float64)
    gt = np.asarray(gt_rows, dtype=np.float64)
    gt_times = gt[:, 0]
    pairs: list[tuple[int, int]] = []
    time_diffs: list[float] = []
    for est_index, stamp in enumerate(est[:, 0]):
        gt_index = int(np.searchsorted(gt_times, stamp))
        candidates: list[int] = []
        if gt_index < len(gt_times):
            candidates.append(gt_index)
        if gt_index > 0:
            candidates.append(gt_index - 1)
        if not candidates:
            continue
        best = min(candidates, key=lambda index: abs(gt_times[index] - stamp))
        time_diff = float(abs(gt_times[best] - stamp))
        if time_diff <= args.max_association_dt:
            pairs.append((est_index, best))
            time_diffs.append(time_diff)

    if len(pairs) < 3:
        raise RuntimeError(
            f"Not enough associated poses for APE: {len(pairs)} "
            f"(max dt {args.max_association_dt}s)"
        )

    est_pair_indices = [est_index for est_index, _gt_index in pairs]
    gt_pair_indices = [gt_index for _est_index, gt_index in pairs]
    est_assoc = est[est_pair_indices]
    gt_assoc = gt[gt_pair_indices]
    est_xyz = est_assoc[:, 1:4].copy()
    gt_xyz = gt_assoc[:, 1:4]
    est_rot = np.stack([quat_xyzw_to_rot(*row[4:8]) for row in est_assoc], axis=0)
    gt_rot = np.stack([quat_xyzw_to_rot(*row[4:8]) for row in gt_assoc], axis=0)

    if args.estimate_frame_to_gt_frame is not None:
        est_xyz, est_rot = apply_estimate_frame_transform(
            est_xyz,
            est_rot,
            args.estimate_frame_to_gt_frame,
        )

    est_mean = est_xyz.mean(axis=0)
    gt_mean = gt_xyz.mean(axis=0)
    est_centered = est_xyz - est_mean
    gt_centered = gt_xyz - gt_mean
    covariance = est_centered.T @ gt_centered / est_xyz.shape[0]
    u_matrix, _singular_values, vt_matrix = np.linalg.svd(covariance)
    reflection_guard = np.eye(3)
    if np.linalg.det(vt_matrix.T @ u_matrix.T) < 0.0:
        reflection_guard[2, 2] = -1.0
    rotation = vt_matrix.T @ reflection_guard @ u_matrix.T
    translation = gt_mean - rotation @ est_mean
    aligned_est_xyz = (rotation @ est_xyz.T).T + translation

    aligned_est_rot = np.einsum("ij,njk->nik", rotation, est_rot)
    est_rpy_deg = np.rad2deg(
        np.unwrap(np.deg2rad(np.stack([rot_to_rpy_deg(item) for item in aligned_est_rot])), axis=0)
    )
    gt_rpy_deg = np.rad2deg(np.unwrap(np.deg2rad(np.stack([rot_to_rpy_deg(item) for item in gt_rot])), axis=0))

    ate_trans = aligned_est_xyz - gt_xyz
    errors = np.linalg.norm(ate_trans, axis=1)
    ate_rpy_error = np.stack(
        [rot_to_rpy_deg(gt_rot[index].T @ aligned_est_rot[index]) for index in range(len(pairs))],
        axis=0,
    )
    ate_rot_angle = np.asarray(
        [rot_angle_deg(gt_rot[index].T @ aligned_est_rot[index]) for index in range(len(pairs))],
        dtype=np.float64,
    )
    est_steps = np.linalg.norm(np.diff(est_xyz, axis=0), axis=1)
    gt_steps = np.linalg.norm(np.diff(gt_xyz, axis=0), axis=1)

    assoc_times = est_assoc[:, 0]
    positive_periods = np.diff(assoc_times)
    positive_periods = positive_periods[positive_periods > 0.0]
    nominal_period = float(np.median(positive_periods)) if positive_periods.size else 0.0
    rte_match_dt = max(float(args.max_association_dt), nominal_period * 0.5)

    def compute_rte_series(est_rot_for_rte: Any) -> dict[str, Any]:
        rte_times: list[float] = []
        rte_est_delta_pos: list[Any] = []
        rte_gt_delta_pos: list[Any] = []
        rte_err_pos: list[Any] = []
        rte_est_delta_rpy: list[Any] = []
        rte_gt_delta_rpy: list[Any] = []
        rte_err_rpy: list[Any] = []
        rte_err_norm: list[float] = []
        rte_err_angle: list[float] = []
        for start_index, stamp in enumerate(assoc_times):
            target_time = stamp + args.rte_delta_s
            end_index = int(np.searchsorted(assoc_times, target_time))
            candidates: list[int] = []
            if end_index < len(assoc_times):
                candidates.append(end_index)
            if end_index > start_index + 1:
                candidates.append(end_index - 1)
            if not candidates:
                continue
            best = min(candidates, key=lambda index: abs(float(assoc_times[index] - target_time)))
            if best <= start_index:
                continue
            if abs(float(assoc_times[best] - target_time)) > rte_match_dt:
                continue

            est_rel_rot, est_rel_pos = relative_pose(
                est_rot_for_rte[start_index],
                aligned_est_xyz[start_index],
                est_rot_for_rte[best],
                aligned_est_xyz[best],
            )
            gt_rel_rot, gt_rel_pos = relative_pose(
                gt_rot[start_index],
                gt_xyz[start_index],
                gt_rot[best],
                gt_xyz[best],
            )
            rel_err_rot = gt_rel_rot.T @ est_rel_rot
            rel_err_pos = gt_rel_rot.T @ (est_rel_pos - gt_rel_pos)

            rte_times.append(float(assoc_times[start_index] - assoc_times[0]))
            rte_est_delta_pos.append(est_rel_pos)
            rte_gt_delta_pos.append(gt_rel_pos)
            rte_err_pos.append(rel_err_pos)
            rte_est_delta_rpy.append(rot_to_rpy_deg(est_rel_rot))
            rte_gt_delta_rpy.append(rot_to_rpy_deg(gt_rel_rot))
            rte_err_rpy.append(rot_to_rpy_deg(rel_err_rot))
            rte_err_norm.append(float(np.linalg.norm(rel_err_pos)))
            rte_err_angle.append(rot_angle_deg(rel_err_rot))

        return {
            "time_s": rte_times,
            "est_delta_pos": np.asarray(rte_est_delta_pos, dtype=np.float64).reshape((-1, 3)),
            "gt_delta_pos": np.asarray(rte_gt_delta_pos, dtype=np.float64).reshape((-1, 3)),
            "err_pos": np.asarray(rte_err_pos, dtype=np.float64).reshape((-1, 3)),
            "est_delta_rpy": np.asarray(rte_est_delta_rpy, dtype=np.float64).reshape((-1, 3)),
            "gt_delta_rpy": np.asarray(rte_gt_delta_rpy, dtype=np.float64).reshape((-1, 3)),
            "err_rpy": np.asarray(rte_err_rpy, dtype=np.float64).reshape((-1, 3)),
            "err_norm": np.asarray(rte_err_norm, dtype=np.float64),
            "err_angle": np.asarray(rte_err_angle, dtype=np.float64),
        }

    rte = compute_rte_series(aligned_est_rot)
    rte_est_delta_pos_arr = rte["est_delta_pos"]
    rte_gt_delta_pos_arr = rte["gt_delta_pos"]
    rte_err_pos_arr = rte["err_pos"]
    rte_est_delta_rpy_arr = rte["est_delta_rpy"]
    rte_gt_delta_rpy_arr = rte["gt_delta_rpy"]
    rte_err_rpy_arr = rte["err_rpy"]
    rte_err_norm_arr = rte["err_norm"]
    rte_err_angle_arr = rte["err_angle"]

    metrics = {
        "benchmark_status": "evaluated",
        "benchmark_gt_tum": str(gt_tum),
        "benchmark_odom_tum": str(odom_tum),
        "benchmark_ape_json": str(benchmark_json),
        "benchmark_timeseries_json": str(timeseries_json),
        "benchmark_alignment": (
            "fixed_frame_transform_plus_se3_no_scale"
            if args.estimate_frame_to_gt_frame is not None
            else "se3_no_scale"
        ),
        "benchmark_estimate_frame_transform": (
            "gt_from_estimate"
            if args.estimate_frame_to_gt_frame is not None
            else "none"
        ),
        "benchmark_estimate_frame_transform_t_xyz_q_xyzw": (
            rounded_array(args.estimate_frame_to_gt_frame)
            if args.estimate_frame_to_gt_frame is not None
            else None
        ),
        "benchmark_max_association_dt_s": round(args.max_association_dt, 6),
        "benchmark_associated_pairs": len(pairs),
        "benchmark_est_count": int(est.shape[0]),
        "benchmark_gt_count": int(gt.shape[0]),
        "benchmark_mean_abs_time_diff_s": round(float(np.mean(time_diffs)), 6),
        "benchmark_max_abs_time_diff_s": round(float(np.max(time_diffs)), 6),
        "benchmark_est_length_m": round(float(np.sum(est_steps)), 6),
        "benchmark_gt_length_m": round(float(np.sum(gt_steps)), 6),
        "benchmark_endpoint_error_m": round(float(errors[-1]), 6),
        "benchmark_se3_rotation_row_major": [round(float(value), 9) for value in rotation.reshape(-1)],
        "benchmark_se3_translation": [round(float(value), 9) for value in translation],
        "benchmark_rte_delta_s": round(float(args.rte_delta_s), 6),
        "benchmark_rte_pair_dt_tolerance_s": round(rte_match_dt, 6),
        "benchmark_rte_count": int(rte_err_norm_arr.size),
    }
    add_scalar_stats(metrics, "benchmark_ate", "m", errors)
    add_scalar_stats(metrics, "benchmark_ate_rot_angle", "deg", ate_rot_angle)
    add_component_stats(metrics, "benchmark_ate", "m", ("x", "y", "z"), ate_trans)
    add_component_stats(metrics, "benchmark_ate", "deg", ("roll", "pitch", "yaw"), ate_rpy_error)

    # Backward-compatible aliases for existing scripts: the previous APE values
    # were translational ATE after the same SE(3) no-scale alignment.
    for suffix in ("rmse_m", "mean_m", "median_m", "std_m", "p95_m", "max_m"):
        metrics[f"benchmark_ape_{suffix}"] = metrics[f"benchmark_ate_{suffix}"]

    if rte_err_norm_arr.size > 0:
        metrics["benchmark_rte_status"] = "evaluated"
        add_scalar_stats(metrics, "benchmark_rte", "m", rte_err_norm_arr)
        add_scalar_stats(metrics, "benchmark_rte_rot_angle", "deg", rte_err_angle_arr)
        add_component_stats(metrics, "benchmark_rte", "m", ("x", "y", "z"), rte_err_pos_arr)
        add_component_stats(metrics, "benchmark_rte", "deg", ("roll", "pitch", "yaw"), rte_err_rpy_arr)
    else:
        metrics["benchmark_rte_status"] = "skipped"
        metrics["benchmark_rte_skip_reason"] = (
            f"no associated pose pairs found at {args.rte_delta_s}s +/- {rte_match_dt}s"
        )

    timeseries = {
        "time_s": rounded_array(assoc_times - assoc_times[0]),
        "orientation_convention": "roll/pitch/yaw degrees from ZYX yaw-pitch-roll extraction",
        "alignment": (
            "fixed_frame_transform_plus_se3_no_scale"
            if args.estimate_frame_to_gt_frame is not None
            else "se3_no_scale"
        ),
        "estimate_frame_transform": (
            "gt_from_estimate"
            if args.estimate_frame_to_gt_frame is not None
            else "none"
        ),
        "estimate_frame_transform_t_xyz_q_xyzw": (
            rounded_array(args.estimate_frame_to_gt_frame)
            if args.estimate_frame_to_gt_frame is not None
            else None
        ),
        "estimate": pose_component_dict(aligned_est_xyz, est_rpy_deg),
        "groundtruth": pose_component_dict(gt_xyz, gt_rpy_deg),
        "ate_error": error_component_dict(ate_trans, ate_rpy_error, errors, ate_rot_angle),
        "rte": {
            "delta_s": round(float(args.rte_delta_s), 6),
            "time_s": rounded_array(rte["time_s"]),
            "estimate_delta": pose_component_dict(rte_est_delta_pos_arr, rte_est_delta_rpy_arr),
            "groundtruth_delta": pose_component_dict(rte_gt_delta_pos_arr, rte_gt_delta_rpy_arr),
            "error": error_component_dict(rte_err_pos_arr, rte_err_rpy_arr, rte_err_norm_arr, rte_err_angle_arr),
        },
    }
    timeseries_json.write_text(json.dumps(timeseries, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    benchmark_json.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
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

    if argv and argv[0] == "__benchmark_ape":
        parser = argparse.ArgumentParser(description="Benchmark odometry against a TUM ground-truth file")
        parser.add_argument("__command")
        parser.add_argument("--bag", required=True)
        parser.add_argument("--topic", default="/odometry")
        parser.add_argument("--gt-tum", required=True)
        parser.add_argument("--output-dir", required=True)
        parser.add_argument("--max-association-dt", type=float, default=0.05)
        parser.add_argument("--rte-delta-s", type=float, default=DEFAULT_RTE_DELTA_S)
        parser.add_argument(
            "--estimate-frame-to-gt-frame",
            nargs=7,
            type=float,
            metavar="VALUE",
            help=(
                "Fixed [x y z qx qy qz qw] transform from the odometry "
                "child frame into the ground-truth body frame"
            ),
        )
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
    parser.add_argument("--map-frame", help="Override map/world frame from the selected config")
    parser.add_argument("--base-frame", help="Override body/base frame from the selected config")
    parser.add_argument("--lidar-frame", help="Override LiDAR frame from the selected config")
    parser.add_argument("--imu-frame", help="Override IMU frame from the selected config")
    parser.add_argument(
        "--visualization-frame",
        help="Override visualization frame from the selected config",
    )
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
    parser.add_argument(
        "--gt-tum",
        help="Advanced override for non-standard ground-truth TUM locations. By default, run.py auto-detects gt-tum.txt near the input bag.",
    )
    parser.add_argument("--max-association-dt", type=float, default=0.05)
    parser.add_argument(
        "--rte-delta-s",
        type=float,
        default=DEFAULT_RTE_DELTA_S,
        help="Relative trajectory error window, in seconds, for ground-truth benchmark reports",
    )
    parser.add_argument("--no-cleanup", action="store_true", help="Do not kill old replay/frontend processes")
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args(sys.argv[1:])
    if getattr(args, "__command", None) == "__analyze_odom":
        return analyze_odom_bag(args)
    if getattr(args, "__command", None) == "__analyze_input":
        return analyze_input_bags(args)
    if getattr(args, "__command", None) == "__benchmark_ape":
        return benchmark_ape(args)
    return run_replay(args)


if __name__ == "__main__":
    raise SystemExit(main())
