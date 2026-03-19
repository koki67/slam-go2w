#!/usr/bin/env python3
"""Compare IMU warning diagnosis reports across GLIM runs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def load_report(path_or_dir: str) -> dict[str, Any]:
    path = Path(path_or_dir).expanduser().resolve()
    if path.is_dir():
        path = path / "imu_warning_diagnosis.json"
    if not path.is_file():
        raise SystemExit(f"missing report: {path_or_dir}")
    return json.loads(path.read_text(encoding="utf-8"))


def format_float(value: float | None, *, digits: int = 2) -> str:
    return "-" if value is None else f"{value:.{digits}f}"


def format_cell(header: str, value: Any) -> str:
    if isinstance(value, float) or value is None:
        digits = 3 if header == "imu_time_offset" else 2
        return format_float(value, digits=digits)
    return str(value)


def compare_reports(reports: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for report in reports:
        final_block = report.get("glim_validation", {}).get("final_block") or {}
        better = final_block.get("better_ratios") or {}
        imu_errors = final_block.get("imu") or {}
        no_imu_errors = final_block.get("no_imu") or {}
        rows.append(
            {
                "run_name": report["run"]["run_name"],
                "imu_time_offset": report["effective_runtime_config"]["imu_time_offset"],
                "T_lidar_imu": report["effective_runtime_config"]["T_lidar_imu"],
                "warning_count": report["glim_validation"]["warning_count"],
                "rot_ratio": better.get("rot"),
                "trans_ratio": better.get("trans"),
                "vel_ratio": better.get("vel"),
                "imu_trans_mean_m": imu_errors.get("trans_mean_m"),
                "imu_vel_mean_mps": imu_errors.get("vel_mean_mps"),
                "noimu_trans_mean_m": no_imu_errors.get("trans_mean_m"),
                "noimu_vel_mean_mps": no_imu_errors.get("vel_mean_mps"),
                "completed_successfully": report["run"]["completed_successfully"],
                "dump_exists": report["run"]["dump_exists"],
            }
        )
    return rows


def print_table(rows: list[dict[str, Any]]) -> None:
    headers = [
        "run_name",
        "imu_time_offset",
        "warning_count",
        "rot_ratio",
        "trans_ratio",
        "vel_ratio",
        "imu_trans_mean_m",
        "imu_vel_mean_mps",
        "completed_successfully",
        "dump_exists",
    ]
    widths = {header: len(header) for header in headers}
    for row in rows:
        for header in headers:
            value = row[header]
            text = format_cell(header, value)
            widths[header] = max(widths[header], len(text))

    header_line = "  ".join(header.ljust(widths[header]) for header in headers)
    print(header_line)
    print("  ".join("-" * widths[header] for header in headers))
    for row in rows:
        values = []
        for header in headers:
            value = row[header]
            text = format_cell(header, value)
            values.append(text.ljust(widths[header]))
        print("  ".join(values))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare GLIM IMU warning diagnosis reports.")
    parser.add_argument("reports", nargs="+", help="Diagnosis JSON paths or report directories")
    parser.add_argument("--output", help="Optional JSON output path for the comparison rows")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    reports = [load_report(path) for path in args.reports]
    rows = compare_reports(reports)
    print_table(rows)

    if args.output:
        output = Path(args.output).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(rows, indent=2, sort_keys=True), encoding="utf-8")
        print(f"\nComparison written: {output}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
