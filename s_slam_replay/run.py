#!/usr/bin/env python3
"""Run an end-to-end s-slam offline replay regression."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any


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
DEFAULT_PLAY_START_LEAD_S = 3.0


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


def run_sourced_capture(prefix: str, argv: list[str], timeout: float = 30.0) -> str:
    result = subprocess.run(
        ["bash", "-lc", sourced_command(prefix, argv)],
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


def wait_for_topic_subscription(prefix: str, topic: str, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    last_counts = (0, 0)
    while time.monotonic() < deadline:
        last_counts = topic_endpoint_counts(prefix, topic)
        if last_counts[1] > 0:
            return
        time.sleep(0.25)

    raise TimeoutError(
        f"Timed out waiting for a subscriber on {topic}: "
        f"publishers={last_counts[0]} subscriptions={last_counts[1]}"
    )


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


def write_report(
    output_dir: Path,
    input_bags: list[Path],
    bag_info: str,
    replay_details: dict[str, Any],
    metrics: dict[str, Any],
    log_metrics: dict[str, int],
    exit_codes: dict[str, Any],
) -> None:
    play_exit_codes = exit_codes.get("play", [])
    if not isinstance(play_exit_codes, list):
        play_exit_codes = [play_exit_codes]

    status = "PASS"
    if (
        metrics.get("odom_count", 0) <= 0 or
        metrics.get("odom_nonfinite_samples", 0) > 0 or
        any(code != 0 for code in play_exit_codes) or
        exit_codes.get("record_odom", 0) not in (0, None)
    ):
        status = "FAIL"
    elif log_metrics.get("large_lio_jumps", 0) > 0 or metrics.get("odom_large_steps", 0) > 0:
        status = "CHECK"

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
            "## Odometry Metrics",
            "",
            "| Metric | Value |",
            "|---|---:|",
        ]
    )

    metric_order = (
        "odom_count",
        "odom_duration_s",
        "odom_rate_hz",
        "odom_max_pos_norm_m",
        "odom_final_pos_norm_m",
        "odom_max_step_m",
        "odom_max_speed_m_s",
        "odom_frozen_steps",
        "odom_large_steps",
        "odom_nonfinite_samples",
    )
    for key in metric_order:
        if key in metrics:
            report_lines.append(f"| `{key}` | `{metrics[key]}` |")

    report_lines.extend(
        [
            "",
            "## Frontend Log Signals",
            "",
            "| Signal | Count |",
            "|---|---:|",
        ]
    )
    for key, value in log_metrics.items():
        report_lines.append(f"| `{key}` | `{value}` |")

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
    ros_setup = expand_path(args.ros_setup)
    workspace_setup = expand_path(args.workspace_setup)
    config_path = resolve_config_path(args.config)
    input_bags = discover_input_bags(input_path)

    prepare_output_dir(output_dir)
    logs_dir = output_dir / "logs"
    odom_dir = output_dir / "odom"
    prefix = source_prefix(ros_setup, workspace_setup)

    bag_infos = []
    for bag in input_bags:
        bag_infos.append(f"### {bag}\n{run_sourced_capture(prefix, ['ros2', 'bag', 'info', str(bag)])}")
    bag_info_text = "\n\n".join(bag_infos)
    (output_dir / "input_info.txt").write_text(bag_info_text, encoding="utf-8")
    topic_types = parse_bag_topic_types(bag_info_text)
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
    exit_codes: dict[str, Any] = {}
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
        wait_for_topic_subscription(prefix, args.odom_topic, DEFAULT_RECORD_START_TIMEOUT_S)
        time.sleep(args.record_start_wait)

        play_wall_start = time.time() + DEFAULT_PLAY_START_LEAD_S
        for index, (bag, start_offset, _start_time) in enumerate(bag_play_schedule):
            play_args = [
                "ros2",
                "bag",
                "play",
                str(bag),
                "--read-ahead-queue-size",
                str(args.read_ahead_queue_size),
            ]
            if args.replay_rate != 1.0:
                play_args.extend(["--rate", str(args.replay_rate)])
            play_processes.append(
                start_sourced_process(
                    prefix,
                    play_args,
                    logs_dir / f"play_{index}.log",
                    start_at_epoch=play_wall_start + start_offset / max(args.replay_rate, 1.0e-6),
                )
            )

        exit_codes["play"] = [process.wait() for process in play_processes]
        time.sleep(args.settle_time)
    finally:
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
    log_metrics = parse_frontend_log(logs_dir / "frontend.log")
    metrics.update(log_metrics)

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
    )

    print(f"Replay complete: {output_dir}")
    print(f"Report: {output_dir / 'report.md'}")
    print(
        "Summary: "
        f"final={metrics['odom_final_pos_norm_m']:.4f} m, "
        f"max={metrics['odom_max_pos_norm_m']:.4f} m, "
        f"max_step={metrics['odom_max_step_m']:.4f} m, "
        f"large_lio_jumps={log_metrics['large_lio_jumps']}, "
        f"odom_large_steps={metrics['odom_large_steps']}"
    )
    return 0


def read_odom_positions(bag_path: Path, topic: str) -> list[tuple[float, tuple[float, float, float]]]:
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

    samples: list[tuple[float, tuple[float, float, float]]] = []
    while reader.has_next():
        topic_name, data, timestamp_ns = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, Odometry)
        pos = msg.pose.pose.position
        samples.append((timestamp_ns * 1.0e-9, (pos.x, pos.y, pos.z)))
    return samples


def norm3(values: tuple[float, float, float]) -> float:
    return math.sqrt(values[0] ** 2 + values[1] ** 2 + values[2] ** 2)


def analyze_odom_bag(args: argparse.Namespace) -> int:
    bag_path = expand_path(args.bag)
    if not has_metadata(bag_path):
        raise FileNotFoundError(f"Odometry bag metadata.yaml not found: {bag_path}")

    samples = read_odom_positions(bag_path, args.topic)
    if not samples:
        raise RuntimeError("No odometry samples found")

    first_t = samples[0][0]
    last_t = samples[-1][0]
    duration = max(0.0, last_t - first_t)
    rate = (len(samples) - 1) / duration if duration > 0.0 and len(samples) > 1 else 0.0

    first = samples[0][1]
    rel_positions = [
        (pos[0] - first[0], pos[1] - first[1], pos[2] - first[2])
        for _, pos in samples
    ]
    max_norm = max(norm3(pos) for pos in rel_positions)
    final_norm = norm3(rel_positions[-1])

    max_step = 0.0
    max_speed = 0.0
    frozen_steps = 0
    large_steps = 0
    nonfinite_samples = 0
    for _, pos in samples:
        if not all(math.isfinite(value) for value in pos):
            nonfinite_samples += 1

    for (t0, p0), (t1, p1) in zip(samples, samples[1:]):
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
        "odom_rate_hz": round(rate, 6),
        "odom_first": [round(value, 6) for value in first],
        "odom_last": [round(value, 6) for value in samples[-1][1]],
        "odom_max_pos_norm_m": round(max_norm, 6),
        "odom_final_pos_norm_m": round(final_norm, 6),
        "odom_max_step_m": round(max_step, 6),
        "odom_max_speed_m_s": round(max_speed, 6),
        "odom_frozen_steps": frozen_steps,
        "odom_large_steps": large_steps,
        "odom_large_step_threshold_m": args.large_step_threshold,
        "odom_nonfinite_samples": nonfinite_samples,
    }
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
    return run_replay(args)


if __name__ == "__main__":
    raise SystemExit(main())
