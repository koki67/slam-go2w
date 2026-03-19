#!/usr/bin/env python3
"""Check whether a rosbag contains the inputs needed for offline GLIM."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from glim_bag_utils import (
    CheckerError,
    SqliteBag,
    TopicSummary,
    analyze_topic_messages,
    discover_dlio_repo,
    load_repo_defaults,
    load_yaml,
    normalize_topic_name,
    parse_bag_path,
)


STATUS_ORDER = {"OK": 0, "warning": 1, "error": 2}
DEFAULT_THRESHOLDS = {
    "min_imu_hz_warn": 200.0,
    "min_imu_hz_error": 50.0,
    "min_lidar_hz_warn": 5.0,
    "min_lidar_hz_error": 1.0,
    "max_gap_ratio_warn": 5.0,
    "max_gap_ratio_error": 20.0,
}
DEFAULT_REQUIRED_POINT_FIELDS = ["x", "y", "z", "intensity", "timestamp"]
DEFAULT_RECOMMENDED_POINT_FIELDS = ["ring"]


def status_max(*statuses: str) -> str:
    return max(statuses, key=lambda item: STATUS_ORDER[item])


def ns_to_seconds(value: int | None) -> float | None:
    if value is None:
        return None
    return value / 1e9


def ns_to_string(value: int | None) -> str:
    if value is None:
        return "n/a"
    seconds = value / 1e9
    return f"{seconds:.6f}s"


def hz_string(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{value:.2f} Hz"


def flatten_overrides(data: dict[str, Any] | None) -> dict[str, Any]:
    if not data:
        return {}

    flattened: dict[str, Any] = {}
    expected = data.get("expected", {})
    thresholds = data.get("thresholds", {})
    flattened.update(expected if isinstance(expected, dict) else {})
    flattened["thresholds"] = thresholds if isinstance(thresholds, dict) else {}

    for key in (
        "imu_topic",
        "lidar_topic",
        "imu_frame",
        "lidar_frame",
        "required_point_fields",
        "recommended_point_fields",
    ):
        if key in data:
            flattened[key] = data[key]

    if "thresholds" not in flattened:
        flattened["thresholds"] = {}

    return flattened


def merge_expectations(
    repo_defaults: dict[str, Any] | None,
    config_overrides: dict[str, Any],
    args: argparse.Namespace,
) -> dict[str, Any]:
    thresholds = dict(DEFAULT_THRESHOLDS)
    thresholds.update(config_overrides.get("thresholds", {}))

    expectations = {
        "imu_topic": repo_defaults.get("imu_topic") if repo_defaults else None,
        "lidar_topic": repo_defaults.get("lidar_topic") if repo_defaults else None,
        "imu_frame": repo_defaults.get("imu_frame") if repo_defaults else None,
        "lidar_frame": repo_defaults.get("lidar_frame") if repo_defaults else None,
        "required_point_fields": list(DEFAULT_REQUIRED_POINT_FIELDS),
        "recommended_point_fields": list(DEFAULT_RECOMMENDED_POINT_FIELDS),
        "expected_imu_type": "sensor_msgs/msg/Imu",
        "expected_lidar_type": "sensor_msgs/msg/PointCloud2",
        "thresholds": thresholds,
        "source": "repository",
    }

    for key in (
        "imu_topic",
        "lidar_topic",
        "imu_frame",
        "lidar_frame",
        "required_point_fields",
        "recommended_point_fields",
    ):
        if key in config_overrides and config_overrides[key] is not None:
            expectations[key] = config_overrides[key]
            expectations["source"] = "config"

    if args.imu_topic:
        expectations["imu_topic"] = args.imu_topic
        expectations["source"] = "cli"
    if args.lidar_topic:
        expectations["lidar_topic"] = args.lidar_topic
        expectations["source"] = "cli"
    if args.imu_frame:
        expectations["imu_frame"] = args.imu_frame
        expectations["source"] = "cli"
    if args.lidar_frame:
        expectations["lidar_frame"] = args.lidar_frame
        expectations["source"] = "cli"

    expectations["imu_topic"] = normalize_topic_name(expectations["imu_topic"])
    expectations["lidar_topic"] = normalize_topic_name(expectations["lidar_topic"])

    if not expectations["imu_topic"] or not expectations["lidar_topic"]:
        raise CheckerError(
            "Expected IMU/LiDAR topics could not be resolved. "
            "Point the checker at the D-LIO repo or provide overrides."
        )

    return expectations


def make_check(name: str, status: str, message: str) -> dict[str, str]:
    return {"name": name, "status": status, "message": message}


def evaluate_frequency(
    approx_hz: float | None, warn_threshold: float, error_threshold: float, label: str
) -> dict[str, str]:
    if approx_hz is None:
        return make_check(f"{label}_frequency", "warning", "Not enough messages to estimate frequency")
    if approx_hz < error_threshold:
        return make_check(
            f"{label}_frequency",
            "error",
            f"Approximate frequency {approx_hz:.2f} Hz is below the error threshold {error_threshold:.2f} Hz",
        )
    if approx_hz < warn_threshold:
        return make_check(
            f"{label}_frequency",
            "warning",
            f"Approximate frequency {approx_hz:.2f} Hz is below the warning threshold {warn_threshold:.2f} Hz",
        )
    return make_check(f"{label}_frequency", "OK", f"Approximate frequency is {approx_hz:.2f} Hz")


def evaluate_gap_quality(
    monotonic: bool | None,
    non_monotonic_count: int | None,
    gap_stats: dict[str, float | int | None],
    thresholds: dict[str, float],
    label: str,
) -> dict[str, str]:
    if monotonic is False:
        return make_check(
            f"{label}_monotonicity",
            "error",
            f"Found {non_monotonic_count} non-monotonic {label.replace('_', ' ')} timestamp steps",
        )

    median_gap = gap_stats.get("median_s")
    max_gap = gap_stats.get("max_s")
    if median_gap is None or max_gap is None or median_gap <= 0:
        return make_check(
            f"{label}_continuity",
            "warning",
            f"Not enough {label.replace('_', ' ')} samples to assess continuity",
        )

    ratio = max_gap / median_gap
    if ratio > thresholds["max_gap_ratio_error"]:
        return make_check(
            f"{label}_continuity",
            "error",
            f"Largest {label.replace('_', ' ')} gap is {ratio:.1f}x the median gap",
        )
    if ratio > thresholds["max_gap_ratio_warn"]:
        return make_check(
            f"{label}_continuity",
            "warning",
            f"Largest {label.replace('_', ' ')} gap is {ratio:.1f}x the median gap",
        )
    return make_check(
        f"{label}_continuity",
        "OK",
        f"Largest {label.replace('_', ' ')} gap is {ratio:.1f}x the median gap",
    )


def evaluate_expected_frame(
    frame_ids: list[str],
    expected_frame: str | None,
    label: str,
) -> dict[str, str]:
    if not frame_ids:
        return make_check(f"{label}_frame", "warning", "No non-empty frame_id values were found")
    if expected_frame and frame_ids != [expected_frame]:
        return make_check(
            f"{label}_frame",
            "error",
            f"Observed frame_ids {frame_ids} do not match expected '{expected_frame}'",
        )
    return make_check(
        f"{label}_frame",
        "OK",
        f"Observed frame_ids: {', '.join(frame_ids)}",
    )


def evaluate_lidar_fields(
    summary: TopicSummary,
    required_fields: list[str],
    recommended_fields: list[str],
) -> list[dict[str, str]]:
    checks: list[dict[str, str]] = []
    common_fields = set(summary.point_fields_common or [])
    if not common_fields:
        checks.append(
            make_check("lidar_point_fields", "error", "Could not decode PointCloud2 field metadata")
        )
        return checks

    missing_required = [field for field in required_fields if field not in common_fields]
    missing_recommended = [field for field in recommended_fields if field not in common_fields]

    if missing_required:
        checks.append(
            make_check(
                "lidar_required_fields",
                "error",
                f"Missing required PointCloud2 fields: {', '.join(missing_required)}",
            )
        )
    else:
        checks.append(
            make_check(
                "lidar_required_fields",
                "OK",
                f"Required PointCloud2 fields present: {', '.join(required_fields)}",
            )
        )

    if missing_recommended:
        checks.append(
            make_check(
                "lidar_recommended_fields",
                "warning",
                f"Missing recommended PointCloud2 fields: {', '.join(missing_recommended)}",
            )
        )
    else:
        checks.append(
            make_check(
                "lidar_recommended_fields",
                "OK",
                f"Recommended PointCloud2 fields present: {', '.join(recommended_fields)}",
            )
        )

    return checks


def evaluate_input_topic(
    input_name: str,
    topic_name: str,
    expected_type: str,
    expected_frame: str | None,
    summaries: dict[str, TopicSummary],
    thresholds: dict[str, float],
    required_fields: list[str] | None = None,
    recommended_fields: list[str] | None = None,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "topic": topic_name,
        "status": "OK",
        "checks": [],
        "summary": None,
    }

    if topic_name not in summaries:
        result["status"] = "error"
        result["checks"].append(
            make_check("presence", "error", f"Required topic {topic_name} is missing from the bag")
        )
        return result

    summary = summaries[topic_name]
    result["summary"] = {
        "topic": summary.name,
        "type": summary.msg_type,
        "message_count": summary.message_count,
        "bag_start_ns": summary.bag_start_ns,
        "bag_end_ns": summary.bag_end_ns,
        "approx_frequency_hz": summary.approx_frequency_hz,
        "bag_timestamp_monotonic": summary.bag_timestamp_monotonic,
        "bag_timestamp_non_monotonic_count": summary.bag_timestamp_non_monotonic_count,
        "header_timestamp_monotonic": summary.header_timestamp_monotonic,
        "header_timestamp_non_monotonic_count": summary.header_timestamp_non_monotonic_count,
        "header_stamp_all_zero": summary.header_stamp_all_zero,
        "frame_ids": summary.frame_ids,
        "point_fields_common": summary.point_fields_common,
        "point_fields_seen": summary.point_fields_seen,
        "bag_gap_stats": summary.bag_gap_stats,
        "header_gap_stats": summary.header_gap_stats,
        "notes": summary.notes,
    }

    checks = result["checks"]
    checks.append(
        make_check("presence", "OK", f"Found {summary.message_count} messages on {topic_name}")
    )

    if summary.msg_type != expected_type:
        checks.append(
            make_check("type", "error", f"Expected {expected_type} but found {summary.msg_type}")
        )
    else:
        checks.append(make_check("type", "OK", f"Message type is {summary.msg_type}"))

    if summary.message_count == 0:
        checks.append(make_check("message_count", "error", "Topic exists but has no messages"))
    else:
        checks.append(
            make_check("message_count", "OK", f"Topic contains {summary.message_count} messages")
        )

    if input_name == "imu":
        checks.append(
            evaluate_frequency(
                summary.approx_frequency_hz,
                thresholds["min_imu_hz_warn"],
                thresholds["min_imu_hz_error"],
                "imu",
            )
        )
    else:
        checks.append(
            evaluate_frequency(
                summary.approx_frequency_hz,
                thresholds["min_lidar_hz_warn"],
                thresholds["min_lidar_hz_error"],
                "lidar",
            )
        )

    checks.append(
        evaluate_gap_quality(
            summary.header_timestamp_monotonic,
            summary.header_timestamp_non_monotonic_count,
            summary.header_gap_stats,
            thresholds,
            "header_stamp",
        )
    )

    if summary.header_stamp_all_zero:
        checks.append(
            make_check("header_stamps_nonzero", "error", "All decoded header timestamps are zero")
        )
    elif summary.header_stamp_all_zero is False:
        checks.append(
            make_check("header_stamps_nonzero", "OK", "Decoded header timestamps are non-zero")
        )
    else:
        checks.append(
            make_check(
                "header_stamps_nonzero",
                "warning",
                "Header timestamps were not decoded for this topic",
            )
        )

    checks.append(evaluate_expected_frame(summary.frame_ids, expected_frame, input_name))

    if input_name == "lidar":
        checks.extend(
            evaluate_lidar_fields(
                summary,
                required_fields or [],
                recommended_fields or [],
            )
        )

    result["status"] = status_max(*(check["status"] for check in checks))
    return result


def build_report(
    bag: SqliteBag,
    repo_defaults: dict[str, Any] | None,
    expectations: dict[str, Any],
    summaries: dict[str, TopicSummary],
) -> dict[str, Any]:
    bag_start_ns = min(
        (summary.bag_start_ns for summary in summaries.values() if summary.bag_start_ns is not None),
        default=None,
    )
    bag_end_ns = max(
        (summary.bag_end_ns for summary in summaries.values() if summary.bag_end_ns is not None),
        default=None,
    )

    imu_result = evaluate_input_topic(
        "imu",
        expectations["imu_topic"],
        expectations["expected_imu_type"],
        expectations.get("imu_frame"),
        summaries,
        expectations["thresholds"],
    )
    lidar_result = evaluate_input_topic(
        "lidar",
        expectations["lidar_topic"],
        expectations["expected_lidar_type"],
        expectations.get("lidar_frame"),
        summaries,
        expectations["thresholds"],
        expectations["required_point_fields"],
        expectations["recommended_point_fields"],
    )
    overall_status = status_max(imu_result["status"], lidar_result["status"])

    return {
        "bag": {
            "path": str(bag.bag_dir),
            "storage_identifier": bag.storage_identifier,
            "bag_start_ns": bag_start_ns,
            "bag_end_ns": bag_end_ns,
            "bag_duration_s": ns_to_seconds(bag_end_ns - bag_start_ns)
            if bag_start_ns is not None and bag_end_ns is not None
            else None,
            "files": [str(file.path) for file in bag.files],
        },
        "repository_defaults": repo_defaults,
        "expectations": expectations,
        "inputs": {
            "imu": imu_result,
            "lidar": lidar_result,
        },
        "topics": {
            name: {
                "type": summary.msg_type,
                "message_count": summary.message_count,
                "bag_start_ns": summary.bag_start_ns,
                "bag_end_ns": summary.bag_end_ns,
                "approx_frequency_hz": summary.approx_frequency_hz,
                "bag_timestamp_monotonic": summary.bag_timestamp_monotonic,
                "bag_timestamp_non_monotonic_count": summary.bag_timestamp_non_monotonic_count,
                "header_timestamp_monotonic": summary.header_timestamp_monotonic,
                "header_timestamp_non_monotonic_count": summary.header_timestamp_non_monotonic_count,
                "header_stamp_all_zero": summary.header_stamp_all_zero,
                "frame_ids": summary.frame_ids,
                "point_fields_common": summary.point_fields_common,
                "point_fields_seen": summary.point_fields_seen,
                "bag_gap_stats": summary.bag_gap_stats,
                "header_gap_stats": summary.header_gap_stats,
                "notes": summary.notes,
            }
            for name, summary in sorted(summaries.items())
        },
        "overall_status": overall_status,
    }


def print_summary(report: dict[str, Any]) -> None:
    bag_info = report["bag"]
    expectations = report["expectations"]
    repo_defaults = report.get("repository_defaults") or {}
    print(f"Bag: {bag_info['path']}")
    print(f"Storage: {bag_info['storage_identifier']}")
    if bag_info["bag_duration_s"] is not None:
        print(
            "Span: "
            f"{ns_to_string(bag_info['bag_start_ns'])} -> {ns_to_string(bag_info['bag_end_ns'])} "
            f"(duration {bag_info['bag_duration_s']:.3f}s)"
        )
    else:
        print("Span: n/a")
    print(
        f"Expected inputs ({expectations['source']}): "
        f"IMU={expectations['imu_topic']}  LiDAR={expectations['lidar_topic']}"
    )
    if repo_defaults:
        print(
            "Repository frames: "
            f"imu={repo_defaults.get('imu_frame')} "
            f"lidar={repo_defaults.get('lidar_frame')} "
            f"base={repo_defaults.get('base_frame')} "
            f"odom={repo_defaults.get('odom_frame')}"
        )
        print(
            "Current D-LIO record_catmux mode: "
            f"{repo_defaults.get('record_catmux_mode')}"
        )
    print("")
    print("Required inputs:")
    for input_name in ("imu", "lidar"):
        result = report["inputs"][input_name]
        summary = result.get("summary") or {}
        print(
            f"  {input_name.upper():5s} {result['status']:7s} "
            f"{result['topic']}  "
            f"type={summary.get('type', 'n/a')}  "
            f"count={summary.get('message_count', 0)}  "
            f"hz={hz_string(summary.get('approx_frequency_hz'))}"
        )
        frame_ids = summary.get("frame_ids") or []
        if frame_ids:
            print(f"         frames: {', '.join(frame_ids)}")
        point_fields = summary.get("point_fields_common") or []
        if point_fields:
            print(f"         point fields: {', '.join(point_fields)}")
        for check in result["checks"]:
            print(f"         - {check['status']:7s} {check['name']}: {check['message']}")
    print("")
    print("Bag topics:")
    for topic_name, topic in report["topics"].items():
        print(
            f"  {topic_name}  {topic['type']}  "
            f"count={topic['message_count']}  hz={hz_string(topic['approx_frequency_hz'])}"
        )
    print("")
    print(f"Overall status: {report['overall_status']}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check whether a rosbag contains the inputs needed for offline GLIM.",
    )
    parser.add_argument("bag_path", help="Path to a rosbag directory, metadata.yaml, or .db3 file")
    parser.add_argument(
        "--report-path",
        help="Path to write the JSON report. Defaults to <bag>/glim_bag_report.json",
    )
    parser.add_argument("--config", help="YAML config file with topic/frame/threshold overrides")
    parser.add_argument(
        "--dlio-repo",
        help="Path to the slam-go2w repository to derive defaults from",
    )
    parser.add_argument("--imu-topic", help="Override the expected IMU topic name")
    parser.add_argument("--lidar-topic", help="Override the expected LiDAR topic name")
    parser.add_argument("--imu-frame", help="Override the expected IMU frame_id")
    parser.add_argument("--lidar-frame", help="Override the expected LiDAR frame_id")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    repo_path = discover_dlio_repo(args.dlio_repo)
    repo_defaults = load_repo_defaults(repo_path) if repo_path else None
    config_overrides = flatten_overrides(load_yaml(Path(args.config))) if args.config else {}
    expectations = merge_expectations(repo_defaults, config_overrides, args)

    bag_dir, metadata_path = parse_bag_path(args.bag_path)
    bag = SqliteBag(bag_dir, metadata_path)
    topics = bag.list_topics()
    summaries = bag.aggregate_topic_stats(topics)

    topics_to_analyze = set(summaries)
    topics_to_analyze.update([expectations["imu_topic"], expectations["lidar_topic"]])
    for topic_name in sorted(topic for topic in topics_to_analyze if topic in summaries):
        analyze_topic_messages(summaries[topic_name], bag, topics)

    report = build_report(bag, repo_defaults, expectations, summaries)
    print_summary(report)

    if args.report_path:
        report_path = Path(args.report_path).expanduser().resolve()
    else:
        report_path = Path.cwd() / f"{bag_dir.name}_glim_bag_report.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(f"Report written: {report_path}")

    return 1 if report["overall_status"] == "error" else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except CheckerError as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(2)
