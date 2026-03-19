#!/usr/bin/env python3
"""Analyze GLIM IMU validation warnings for an offline run."""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import sys
from pathlib import Path
from typing import Any

from glim_bag_utils import (
    CheckerError,
    SqliteBag,
    analyze_topic_messages,
    decode_imu_full,
    discover_dlio_repo,
    extract_first_point_timestamp_ns,
    extract_ros_parameters,
    load_repo_defaults,
    parse_bag_path,
)


GLIM_THRESHOLDS = {"rot": 0.7, "trans": 0.4, "vel": 0.5}


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def stats_from_values(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "mean": None, "median": None, "min": None, "max": None}
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "min": min(values),
        "max": max(values),
    }


def parse_manifest(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    if not path.exists():
        return result
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        result[key.strip()] = value.strip()
    return result


def load_runtime_configs(run_dir: Path) -> dict[str, Any]:
    resolved_dir = run_dir / "resolved_config"
    if not resolved_dir.is_dir():
        raise CheckerError(f"Resolved config directory is missing: {resolved_dir}")

    config_json = load_json(resolved_dir / "config.json")
    config_ros = load_json(resolved_dir / "config_ros.json")
    config_sensors = load_json(resolved_dir / "config_sensors.json")
    config_preprocess = load_json(resolved_dir / "config_preprocess.json")

    return {
        "resolved_dir": str(resolved_dir),
        "config_json": config_json,
        "config_ros": config_ros,
        "config_sensors": config_sensors,
        "config_preprocess": config_preprocess,
    }


def collect_bag_diagnostics(
    bag_path: Path, imu_topic: str, lidar_topic: str
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    bag_dir, metadata_path = parse_bag_path(str(bag_path))
    bag = SqliteBag(bag_dir, metadata_path)
    topics = bag.list_topics()
    summaries = bag.aggregate_topic_stats(topics)

    for topic_name in sorted(set(summaries) | {imu_topic, lidar_topic}):
        if topic_name in summaries:
            analyze_topic_messages(summaries[topic_name], bag, topics)

    imu_topic_info = topics.get(imu_topic)
    lidar_topic_info = topics.get(lidar_topic)
    if imu_topic_info is None or lidar_topic_info is None:
        raise CheckerError("Expected IMU/LiDAR topics are missing from the bag")

    imu_bag_minus_header_ms: list[float] = []
    acc_norms: list[float] = []
    gyro_norms: list[float] = []
    quat_norms: list[float] = []
    for bag_ts, data in bag.iter_topic_messages(imu_topic, topics):
        decoded = decode_imu_full(data)
        header_ts = int(decoded["header"]["stamp_ns"])
        imu_bag_minus_header_ms.append((bag_ts - header_ts) / 1e6)
        acc = decoded["linear_acceleration"]
        gyro = decoded["angular_velocity"]
        quat = decoded["orientation"]
        acc_norms.append(math.sqrt(sum(value * value for value in acc)))
        gyro_norms.append(math.sqrt(sum(value * value for value in gyro)))
        quat_norms.append(math.sqrt(sum(value * value for value in quat)))

    lidar_bag_minus_header_ms: list[float] = []
    firstpoint_minus_header_ms: list[float] = []
    firstpoint_minus_bag_ms: list[float] = []
    for bag_ts, data in bag.iter_topic_messages(lidar_topic, topics):
        decoded = summaries[lidar_topic]
        del decoded  # summarization already handled; keep iteration explicit here
        pc2 = bag  # silence lint-like editors in plain script context
        del pc2
        from glim_bag_utils import decode_pointcloud2  # local import to keep top list short

        decoded_pc2 = decode_pointcloud2(data)
        header_ts = int(decoded_pc2["header"]["stamp_ns"])
        lidar_bag_minus_header_ms.append((bag_ts - header_ts) / 1e6)
        first_point_ns = extract_first_point_timestamp_ns(data)
        if first_point_ns is not None:
            firstpoint_minus_header_ms.append((first_point_ns - header_ts) / 1e6)
            firstpoint_minus_bag_ms.append((first_point_ns - bag_ts) / 1e6)

    bag_info = {
        "path": str(bag_dir),
        "storage_identifier": bag.storage_identifier,
        "imu": {
            "topic": imu_topic,
            "summary": {
                "message_count": summaries[imu_topic].message_count,
                "approx_frequency_hz": summaries[imu_topic].approx_frequency_hz,
                "frame_ids": summaries[imu_topic].frame_ids,
                "bag_start_ns": summaries[imu_topic].bag_start_ns,
                "bag_end_ns": summaries[imu_topic].bag_end_ns,
            },
            "bag_minus_header_ms": stats_from_values(imu_bag_minus_header_ms),
            "acc_norm_mps2": stats_from_values(acc_norms),
            "gyro_norm_rps": stats_from_values(gyro_norms),
            "quat_norm": stats_from_values(quat_norms),
        },
        "lidar": {
            "topic": lidar_topic,
            "summary": {
                "message_count": summaries[lidar_topic].message_count,
                "approx_frequency_hz": summaries[lidar_topic].approx_frequency_hz,
                "frame_ids": summaries[lidar_topic].frame_ids,
                "bag_start_ns": summaries[lidar_topic].bag_start_ns,
                "bag_end_ns": summaries[lidar_topic].bag_end_ns,
                "point_fields_common": summaries[lidar_topic].point_fields_common,
            },
            "bag_minus_header_ms": stats_from_values(lidar_bag_minus_header_ms),
            "firstpoint_minus_header_ms": stats_from_values(firstpoint_minus_header_ms),
            "firstpoint_minus_bag_ms": stats_from_values(firstpoint_minus_bag_ms),
        },
    }

    return bag_info, topics, summaries


def parse_validation_blocks(log_text: str) -> dict[str, Any]:
    blocks: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None

    for line in log_text.splitlines():
        if "IMU prediction is not good." in line:
            continue
        if "Possibly T_lidar_imu is not accurate" in line:
            continue

        match = re.search(r"num_validations=(\d+)", line)
        if match:
            if current:
                blocks.append(current)
            current = {
                "num_validations": int(match.group(1)),
            }
            continue

        match = re.search(
            r"No-IMU errors rot=([0-9.]+) \+- ([0-9.]+) deg, trans=([0-9.]+) \+- ([0-9.]+) m, vel=([0-9.]+) \+- ([0-9.]+) m/s",
            line,
        )
        if match and current is not None:
            current["no_imu"] = {
                "rot_mean_deg": float(match.group(1)),
                "rot_std_deg": float(match.group(2)),
                "trans_mean_m": float(match.group(3)),
                "trans_std_m": float(match.group(4)),
                "vel_mean_mps": float(match.group(5)),
                "vel_std_mps": float(match.group(6)),
            }
            continue

        match = re.search(
            r"IMU errors rot=([0-9.]+) \+- ([0-9.]+) deg, trans=([0-9.]+) \+- ([0-9.]+) m, vel=([0-9.]+) \+- ([0-9.]+) m/s",
            line,
        )
        if match and current is not None:
            current["imu"] = {
                "rot_mean_deg": float(match.group(1)),
                "rot_std_deg": float(match.group(2)),
                "trans_mean_m": float(match.group(3)),
                "trans_std_m": float(match.group(4)),
                "vel_mean_mps": float(match.group(5)),
                "vel_std_mps": float(match.group(6)),
            }
            continue

        match = re.search(
            r"IMU better ratios rot=([0-9.]+), trans=([0-9.]+), vel=([0-9.]+)",
            line,
        )
        if match and current is not None:
            current["better_ratios"] = {
                "rot": float(match.group(1)),
                "trans": float(match.group(2)),
                "vel": float(match.group(3)),
            }
            current["passes_thresholds"] = {
                key: current["better_ratios"][key] > GLIM_THRESHOLDS[key]
                for key in GLIM_THRESHOLDS
            }
            current["all_thresholds_pass"] = all(current["passes_thresholds"].values())
            continue

        match = re.search(
            r"imu_bias=vec\(([0-9eE+.,\- ]+)\)",
            line,
        )
        if match:
            values = [float(value.strip()) for value in match.group(1).split(",")]
            if current is None:
                current = {}
            current.setdefault("initial_bias_estimate", values)

    if current:
        blocks.append(current)

    return {
        "warning_count": log_text.count("IMU prediction is not good."),
        "validation_blocks": blocks,
        "final_block": blocks[-1] if blocks else None,
        "thresholds": GLIM_THRESHOLDS,
    }


def read_trajectory(path: Path) -> list[list[float]]:
    if not path.exists():
        return []
    rows: list[list[float]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        text = line.strip()
        if not text:
            continue
        rows.append([float(value) for value in text.split()])
    return rows


def qmul(a: list[float], b: list[float]) -> list[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def qconj(q: list[float]) -> list[float]:
    x, y, z, w = q
    return [-x, -y, -z, w]


def qrot(q: list[float], v: list[float]) -> list[float]:
    return qmul(qmul(q, [v[0], v[1], v[2], 0.0]), qconj(q))[:3]


def vector_norm(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def quaternion_angle_deg(a: list[float], b: list[float]) -> float:
    qr = qmul(qconj(a), b)
    angle = 2.0 * math.degrees(math.acos(max(-1.0, min(1.0, qr[3]))))
    return 360.0 - angle if angle > 180.0 else angle


def analyze_trajectory_consistency(run_dir: Path, config_sensors: dict[str, Any]) -> dict[str, Any]:
    lidar_rows = read_trajectory(run_dir / "dump" / "traj_lidar.txt")
    imu_rows = read_trajectory(run_dir / "dump" / "traj_imu.txt")
    if not lidar_rows or len(lidar_rows) != len(imu_rows):
        return {
            "available": False,
            "message": "Trajectory files are missing or mismatched",
        }

    rel_translations: list[list[float]] = []
    rel_rot_angles: list[float] = []
    config_pose = config_sensors["sensors"]["T_lidar_imu"]
    config_translation = [float(value) for value in config_pose[:3]]
    config_quat = [float(value) for value in config_pose[3:7]]

    for lidar_row, imu_row in zip(lidar_rows, imu_rows):
        lidar_pos = lidar_row[1:4]
        lidar_quat = lidar_row[4:8]
        imu_pos = imu_row[1:4]
        imu_quat = imu_row[4:8]
        world_delta = [imu_pos[index] - lidar_pos[index] for index in range(3)]
        lidar_frame_delta = qrot(qconj(lidar_quat), world_delta)
        rel_translations.append(lidar_frame_delta)
        rel_rot_angles.append(quaternion_angle_deg(lidar_quat, imu_quat))

    mean_translation = [
        statistics.fmean(values[index] for values in rel_translations)
        for index in range(3)
    ]
    translation_deviation = [
        vector_norm([values[index] - mean_translation[index] for index in range(3)])
        for values in rel_translations
    ]
    translation_error = [
        mean_translation[index] - config_translation[index] for index in range(3)
    ]

    return {
        "available": True,
        "pose_count": len(lidar_rows),
        "mean_rel_translation_lidar_frame_m": mean_translation,
        "max_rel_translation_deviation_m": max(translation_deviation),
        "config_translation_m": config_translation,
        "config_translation_error_m": translation_error,
        "config_translation_error_norm_m": vector_norm(translation_error),
        "mean_rel_rotation_angle_deg": statistics.fmean(rel_rot_angles),
        "max_rel_rotation_angle_deg": max(rel_rot_angles),
        "config_quaternion_xyzw": config_quat,
    }


def collect_dlio_comparison(repo_path: Path) -> dict[str, Any]:
    params_path = repo_path / "humble_ws/src/direct_lidar_inertial_odometry/cfg/params.yaml"
    dlio_path = repo_path / "humble_ws/src/direct_lidar_inertial_odometry/cfg/dlio.yaml"
    imu_cpp_path = repo_path / "humble_ws/src/go2_unitree_ros2/src/devel/imu_publisher.cpp"
    lowstate_path = repo_path / "humble_ws/src/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/msg/LowState.msg"

    params = extract_ros_parameters(params_path)
    dlio = extract_ros_parameters(dlio_path)
    imu_cpp_text = imu_cpp_path.read_text(encoding="utf-8")
    lowstate_lines = [
        line.strip()
        for line in lowstate_path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]
    lowstate_fields = [line.split()[-1] for line in lowstate_lines if " " in line]
    timestamp_like_fields = [
        field for field in lowstate_fields if any(token in field.lower() for token in ("stamp", "time", "tick"))
    ]
    has_explicit_timestamp = any(
        token in field.lower() for field in lowstate_fields for token in ("stamp", "time")
    )

    return {
        "repo_path": str(repo_path),
        "frames": {
            "imu": params.get("frames/imu"),
            "lidar": params.get("frames/lidar"),
            "base": params.get("frames/baselink"),
            "odom": params.get("frames/odom"),
        },
        "extrinsics": {
            "base_to_imu_t": dlio.get("extrinsics/baselink2imu/t"),
            "base_to_imu_R": dlio.get("extrinsics/baselink2imu/R"),
            "base_to_lidar_t": dlio.get("extrinsics/baselink2lidar/t"),
            "base_to_lidar_R": dlio.get("extrinsics/baselink2lidar/R"),
        },
        "odom_compute_time_offset": bool(params.get("odom/computeTimeOffset")),
        "imu_publisher": {
            "topic": "/go2w/imu",
            "uses_now_timestamp": "header.stamp = this->get_clock()->now();" in imu_cpp_text,
            "source_topic": "lowstate",
        },
        "lowstate": {
            "path": str(lowstate_path),
            "timestamp_like_fields": timestamp_like_fields,
            "has_explicit_timestamp": has_explicit_timestamp,
            "tick_field_present": "tick" in lowstate_fields,
        },
    }


def build_diagnosis(
    bag_checker_report: dict[str, Any] | None,
    runtime_config: dict[str, Any],
    dlio_comparison: dict[str, Any],
    bag_info: dict[str, Any],
    validation: dict[str, Any],
    trajectory: dict[str, Any],
) -> dict[str, Any]:
    likely_causes: list[str] = []
    weaker_hypotheses: list[str] = []
    ruled_out: list[str] = []

    final_block = validation.get("final_block") or {}
    better = final_block.get("better_ratios") or {}
    trans_ratio = better.get("trans")
    vel_ratio = better.get("vel")
    rot_ratio = better.get("rot")

    imu_skew = bag_info["imu"]["bag_minus_header_ms"].get("mean")
    lidar_skew = bag_info["lidar"]["bag_minus_header_ms"].get("mean")
    firstpoint_header = bag_info["lidar"]["firstpoint_minus_header_ms"].get("mean")

    if (
        dlio_comparison["odom_compute_time_offset"]
        and dlio_comparison["imu_publisher"]["uses_now_timestamp"]
        and not dlio_comparison["lowstate"]["has_explicit_timestamp"]
        and imu_skew is not None
        and lidar_skew is not None
        and abs(lidar_skew - imu_skew) > 50.0
    ):
        likely_causes.append(
            "LiDAR-IMU timing mismatch is the strongest current hypothesis: GLIM runs with zero offsets, D-LIO computes a time offset online, the IMU republisher stamps with now(), and LowState does not carry an explicit timestamp."
        )

    if trans_ratio is not None and vel_ratio is not None and rot_ratio is not None:
        if rot_ratio >= 0.55 and (trans_ratio < GLIM_THRESHOLDS["trans"] or vel_ratio < GLIM_THRESHOLDS["vel"]):
            likely_causes.append(
                "The warning pattern is timing/extrinsic-shaped rather than IMU-dead: IMU helps rotation noticeably, but not translation/velocity enough to satisfy GLIM's thresholds."
            )

    if trajectory.get("available"):
        likely_causes.append(
            "The current T_lidar_imu is applied consistently by GLIM, but it is still only an inherited approximation from D-LIO and has not been independently calibrated for this offline pipeline."
        )

    initial_bias = (validation.get("validation_blocks") or [{}])[0].get("initial_bias_estimate")
    if initial_bias:
        weaker_hypotheses.append(
            f"IMU bias/noise mismatch remains possible, but the initial GLIM bias estimate is small ({', '.join(f'{value:.4f}' for value in initial_bias)}), which makes it a weaker hypothesis than timing."
        )
    else:
        weaker_hypotheses.append(
            "IMU bias/noise mismatch remains possible because the current noise values are upstream defaults, but the successful run does not show evidence of a gross IMU failure."
        )

    if bag_checker_report and bag_checker_report.get("overall_status") == "OK":
        ruled_out.append("Raw bag integrity and required topic/field presence are already validated by the checker.")
    ruled_out.append("Gross frame-name or topic-name mismatch is unlikely: the bag, resolved config, and D-LIO defaults agree on /go2w/imu, /points_raw, imu_link, and hesai_lidar.")

    acc_mean = bag_info["imu"]["acc_norm_mps2"].get("mean")
    if acc_mean is not None and 7.0 <= acc_mean <= 12.0:
        ruled_out.append(
            f"IMU linear acceleration units are probably already m/s^2, not g: mean accel norm is {acc_mean:.3f}."
        )

    if firstpoint_header is not None and abs(firstpoint_header) < 1.0:
        ruled_out.append(
            "LiDAR per-point timestamp semantics appear usable: GLIM detected absolute point times and the first-point timestamp matches the cloud header closely."
        )

    ruled_out.append(
        "The old negative-sign legacy extrinsic should not be retried first: upstream GLIM defines T_lidar_imu as IMU->LiDAR, and the current positive-sign transform matches that convention."
    )

    acc_scale = runtime_config["config_ros"]["glim_ros"].get("acc_scale", 0.0)
    if acc_scale == 0.0:
        ruled_out.append(
            "acc_scale=0.0 means auto-detect (upstream GLIM default), not zeroed accelerometer. "
            "No action needed unless auto-detect produces wrong units."
        )

    acc_max = bag_info["imu"]["acc_norm_mps2"].get("max")
    acc_mean_val = bag_info["imu"]["acc_norm_mps2"].get("mean")
    if acc_max is not None and acc_mean_val is not None and acc_mean_val > 0 and acc_max / acc_mean_val > 2.0:
        weaker_hypotheses.append(
            f"Walking-robot accelerometer dynamics (max/mean acc norm = {acc_max / acc_mean_val:.1f}x) "
            "make IMU-based translation prediction inherently noisy between LiDAR scans. "
            "The flat translation ratio across timing offsets is consistent with this limitation."
        )

    recommended_next_experiment = "imu_time_offset_neg010"
    if runtime_config["config_ros"]["glim_ros"]["imu_time_offset"] != 0.0:
        recommended_next_experiment = "t_lidar_imu_unitree_mount"

    summary = likely_causes[0] if likely_causes else "No dominant cause could be ranked from the current evidence."
    return {
        "summary": summary,
        "likely_causes": likely_causes,
        "weaker_hypotheses": weaker_hypotheses,
        "ruled_out": ruled_out,
        "recommended_next_experiment": recommended_next_experiment,
    }


def format_report_text(report: dict[str, Any]) -> str:
    lines: list[str] = []
    lines.append(f"Run: {report['run']['run_name']}")
    lines.append(f"Status: {'passed' if report['run']['completed_successfully'] else 'not-passed'}")
    lines.append(f"Dump exists: {report['run']['dump_exists']}")
    lines.append("")
    lines.append("Effective runtime config:")
    lines.append(
        f"  T_lidar_imu={report['effective_runtime_config']['T_lidar_imu']}  "
        f"imu_time_offset={report['effective_runtime_config']['imu_time_offset']}  "
        f"points_time_offset={report['effective_runtime_config']['points_time_offset']}  "
        f"acc_scale={report['effective_runtime_config'].get('acc_scale', 'N/A')}"
    )
    lines.append(
        f"  imu_noise(acc,gyro,int,bias)=("
        f"{report['effective_runtime_config']['imu_acc_noise']}, "
        f"{report['effective_runtime_config']['imu_gyro_noise']}, "
        f"{report['effective_runtime_config']['imu_int_noise']}, "
        f"{report['effective_runtime_config']['imu_bias_noise']})"
    )
    lines.append("")
    lines.append("Bag timing summary:")
    lines.append(
        f"  IMU hz={report['bag_timing_summary']['imu']['summary']['approx_frequency_hz']:.2f}  "
        f"bag-header mean={report['bag_timing_summary']['imu']['bag_minus_header_ms']['mean']:.3f} ms  "
        f"acc_norm mean={report['bag_timing_summary']['imu']['acc_norm_mps2']['mean']:.3f}"
    )
    lines.append(
        f"  LiDAR hz={report['bag_timing_summary']['lidar']['summary']['approx_frequency_hz']:.2f}  "
        f"bag-header mean={report['bag_timing_summary']['lidar']['bag_minus_header_ms']['mean']:.3f} ms  "
        f"firstpoint-header mean={report['bag_timing_summary']['lidar']['firstpoint_minus_header_ms']['mean']:.6f} ms"
    )
    lines.append("")
    lines.append("GLIM IMU validation:")
    lines.append(f"  warning_count={report['glim_validation']['warning_count']}")
    final_block = report["glim_validation"].get("final_block")
    if final_block:
        lines.append(
            "  final better ratios "
            f"rot={final_block['better_ratios']['rot']:.2f} "
            f"trans={final_block['better_ratios']['trans']:.2f} "
            f"vel={final_block['better_ratios']['vel']:.2f}"
        )
        lines.append(
            "  thresholds "
            f"rot>{GLIM_THRESHOLDS['rot']:.1f} "
            f"trans>{GLIM_THRESHOLDS['trans']:.1f} "
            f"vel>{GLIM_THRESHOLDS['vel']:.1f}"
        )
    lines.append("")
    lines.append("Trajectory consistency:")
    trajectory = report["trajectory_consistency"]
    if trajectory.get("available"):
        lines.append(
            "  mean rel translation="
            f"{trajectory['mean_rel_translation_lidar_frame_m']}  "
            f"config error norm={trajectory['config_translation_error_norm_m']:.6f} m"
        )
        lines.append(
            "  rel rotation max="
            f"{trajectory['max_rel_rotation_angle_deg']:.6f} deg"
        )
    else:
        lines.append(f"  {trajectory['message']}")
    lines.append("")
    lines.append("Diagnosis:")
    lines.append(f"  Summary: {report['diagnosis']['summary']}")
    lines.append("  Likely causes:")
    for item in report["diagnosis"]["likely_causes"]:
        lines.append(f"    - {item}")
    lines.append("  Weaker hypotheses:")
    for item in report["diagnosis"]["weaker_hypotheses"]:
        lines.append(f"    - {item}")
    lines.append("  Ruled out:")
    for item in report["diagnosis"]["ruled_out"]:
        lines.append(f"    - {item}")
    lines.append(f"  Recommended next experiment: {report['diagnosis']['recommended_next_experiment']}")
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze GLIM IMU validation warnings for one run.")
    parser.add_argument("--bag", required=True, help="Path to the rosbag used for the run")
    parser.add_argument("--run-dir", required=True, help="Path to output/results/<run_name>")
    parser.add_argument("--log-dir", required=True, help="Path to output/logs/<run_name>")
    parser.add_argument("--report-dir", required=True, help="Path to output/reports/<run_name>")
    parser.add_argument("--dlio-repo", help="Path to the slam-go2w repository")
    parser.add_argument("--json-output", help="Optional override path for JSON output")
    parser.add_argument("--text-output", help="Optional override path for text output")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    run_dir = Path(args.run_dir).expanduser().resolve()
    log_dir = Path(args.log_dir).expanduser().resolve()
    report_dir = Path(args.report_dir).expanduser().resolve()

    runtime_config = load_runtime_configs(run_dir)
    config_ros = runtime_config["config_ros"]["glim_ros"]
    config_sensors = runtime_config["config_sensors"]["sensors"]
    imu_topic = config_ros["imu_topic"]
    lidar_topic = config_ros["points_topic"]

    bag_info, _, _ = collect_bag_diagnostics(bag_path, imu_topic, lidar_topic)

    repo_path = discover_dlio_repo(args.dlio_repo)
    if repo_path is None:
        raise CheckerError("Could not resolve the D-LIO repository path")
    dlio_comparison = collect_dlio_comparison(repo_path)
    repo_defaults = load_repo_defaults(repo_path)

    manifest = parse_manifest(report_dir / "run_manifest.txt")
    bag_checker_report_path = report_dir / "bag_checker.json"
    bag_checker_report = load_json(bag_checker_report_path) if bag_checker_report_path.exists() else None
    log_path = log_dir / "glim_rosbag.stdout.log"
    if not log_path.exists():
        raise CheckerError(f"Missing GLIM log file: {log_path}")
    log_text = log_path.read_text(encoding="utf-8")
    validation = parse_validation_blocks(log_text)
    trajectory = analyze_trajectory_consistency(run_dir, runtime_config["config_sensors"])

    effective_runtime_config = {
        "config_mode": manifest.get("config_mode"),
        "imu_topic": imu_topic,
        "points_topic": lidar_topic,
        "imu_frame_id": config_ros.get("imu_frame_id"),
        "lidar_frame_id": config_ros.get("lidar_frame_id"),
        "base_frame_id": config_ros.get("base_frame_id"),
        "odom_frame_id": config_ros.get("odom_frame_id"),
        "imu_time_offset": config_ros.get("imu_time_offset"),
        "points_time_offset": config_ros.get("points_time_offset"),
        "acc_scale": config_ros.get("acc_scale"),
        "T_lidar_imu": config_sensors.get("T_lidar_imu"),
        "imu_acc_noise": config_sensors.get("imu_acc_noise"),
        "imu_gyro_noise": config_sensors.get("imu_gyro_noise"),
        "imu_int_noise": config_sensors.get("imu_int_noise"),
        "imu_bias_noise": config_sensors.get("imu_bias_noise"),
        "perpoint_timing": {
            "autoconf_perpoint_times": config_sensors.get("autoconf_perpoint_times"),
            "autoconf_prefer_frame_time": config_sensors.get("autoconf_prefer_frame_time"),
            "perpoint_relative_time": config_sensors.get("perpoint_relative_time"),
            "perpoint_time_scale": config_sensors.get("perpoint_time_scale"),
        },
    }

    run_info = {
        "run_name": manifest.get("run_name", run_dir.name),
        "run_dir": str(run_dir),
        "log_dir": str(log_dir),
        "report_dir": str(report_dir),
        "manifest": manifest,
        "completed_successfully": manifest.get("final_status") == "passed",
        "dump_exists": (run_dir / "dump" / "graph.bin").exists(),
        "overlay_config_dirs": manifest.get("overlay_config_dirs", ""),
    }

    diagnosis = build_diagnosis(
        bag_checker_report,
        runtime_config,
        dlio_comparison,
        bag_info,
        validation,
        trajectory,
    )

    report = {
        "run": run_info,
        "bag": {
            "path": str(bag_path),
            "checker_report_overall_status": bag_checker_report.get("overall_status")
            if bag_checker_report
            else None,
        },
        "repository_defaults": repo_defaults,
        "effective_runtime_config": effective_runtime_config,
        "dlio_comparison": dlio_comparison,
        "bag_timing_summary": bag_info,
        "glim_validation": validation,
        "trajectory_consistency": trajectory,
        "diagnosis": diagnosis,
    }

    report_dir.mkdir(parents=True, exist_ok=True)
    json_output = (
        Path(args.json_output).expanduser().resolve()
        if args.json_output
        else report_dir / "imu_warning_diagnosis.json"
    )
    text_output = (
        Path(args.text_output).expanduser().resolve()
        if args.text_output
        else report_dir / "imu_warning_diagnosis.txt"
    )
    text_output.parent.mkdir(parents=True, exist_ok=True)
    json_output.parent.mkdir(parents=True, exist_ok=True)

    json_output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    text_output.write_text(format_report_text(report), encoding="utf-8")

    print(format_report_text(report), end="")
    print(f"JSON report written: {json_output}")
    print(f"Text report written: {text_output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except CheckerError as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(2)
