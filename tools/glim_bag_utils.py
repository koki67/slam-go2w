#!/usr/bin/env python3
"""Shared rosbag and D-LIO utilities for the offline GLIM tools."""

from __future__ import annotations

import os
import re
import sqlite3
import statistics
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

import yaml


SUPPORTED_STORAGE = {"sqlite3"}
HEADER_FIRST_TYPES = {
    "geometry_msgs/msg/PoseArray",
    "geometry_msgs/msg/PoseStamped",
    "nav_msgs/msg/Odometry",
    "nav_msgs/msg/Path",
    "sensor_msgs/msg/Image",
}


class CheckerError(RuntimeError):
    """Raised when the checker or diagnostics cannot continue safely."""


def normalize_topic_name(name: str | None) -> str | None:
    if not name:
        return None
    return name if name.startswith("/") else f"/{name}"


def load_yaml(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def nested_get(mapping: dict[str, Any], *keys: str, default: Any = None) -> Any:
    current: Any = mapping
    for key in keys:
        if not isinstance(current, dict):
            return default
        current = current.get(key, default)
        if current is default:
            return default
    return current


def parse_launch_default(path: Path, variable_name: str) -> str:
    text = path.read_text(encoding="utf-8")
    pattern = re.compile(
        rf"{re.escape(variable_name)}\s*=\s*LaunchConfiguration\(\s*'{re.escape(variable_name)}'\s*,\s*default='([^']+)'\s*\)"
    )
    match = pattern.search(text)
    if not match:
        raise CheckerError(f"Could not find {variable_name} default in {path}")
    return match.group(1)


def extract_ros_parameters(path: Path) -> dict[str, Any]:
    data = load_yaml(path)
    if not isinstance(data, dict) or not data:
        raise CheckerError(f"Unexpected YAML structure in {path}")

    root = next(iter(data.values()))
    params = root.get("ros__parameters") if isinstance(root, dict) else None
    if not isinstance(params, dict):
        raise CheckerError(f"Missing ros__parameters in {path}")
    return params


def parse_bag_record_topics(path: Path) -> list[str]:
    data = load_yaml(path)
    if not isinstance(data, dict):
        return []

    windows = data.get("windows", [])
    if not isinstance(windows, list):
        return []

    for window in windows:
        if not isinstance(window, dict):
            continue
        commands = window.get("commands", [])
        if not isinstance(commands, list):
            continue
        for command in commands:
            if not isinstance(command, str) or "ros2 bag record" not in command:
                continue
            topics = [
                normalize_topic_name(match)
                for match in re.findall(r"(\/[A-Za-z0-9_][^\s]*)", command)
            ]
            if " -o " in command or " --output " in command:
                topics = topics[1:]
            return [topic for topic in topics if topic]
    return []


def discover_dlio_repo(user_path: str | None) -> Path | None:
    candidates: list[Path] = []
    if user_path:
        candidates.append(Path(user_path).expanduser())

    env_text = os.environ.get("DLIO_GO2W_REPO")
    if env_text:
        candidates.append(Path(env_text).expanduser())

    script_root = Path(__file__).resolve().parents[1]
    candidates.extend(
        [
            # Unified slam-go2w repo (tools/ is one level below root)
            script_root,
            Path.cwd(),
            Path.cwd().parent,
        ]
    )

    marker = Path("humble_ws/src/direct_lidar_inertial_odometry/launch/dlio.launch.py")
    for candidate in candidates:
        if (candidate / marker).exists():
            return candidate.resolve()
    return None


def load_repo_defaults(repo_path: Path) -> dict[str, Any]:
    launch_path = repo_path / "humble_ws/src/direct_lidar_inertial_odometry/launch/dlio.launch.py"
    params_path = repo_path / "humble_ws/src/direct_lidar_inertial_odometry/cfg/params.yaml"

    # Support the current unified repo layout, the older unified file name,
    # and the legacy layout (humble_ws/src/).
    record_path = repo_path / "catmux/record_dlio.yaml"
    if not record_path.exists():
        record_path = repo_path / "catmux/record_dlio_output.yaml"
    if not record_path.exists():
        record_path = repo_path / "humble_ws/src/record_catmux.yaml"

    params = extract_ros_parameters(params_path)
    imu_topic = normalize_topic_name(parse_launch_default(launch_path, "imu_topic"))
    lidar_topic = normalize_topic_name(parse_launch_default(launch_path, "pointcloud_topic"))
    bag_record_topics = parse_bag_record_topics(record_path)
    bag_records_raw_inputs = bool(
        imu_topic in bag_record_topics and lidar_topic in bag_record_topics
    )

    return {
        "repo_path": str(repo_path),
        "imu_topic": imu_topic,
        "lidar_topic": lidar_topic,
        "imu_frame": params.get("frames/imu"),
        "lidar_frame": params.get("frames/lidar"),
        "base_frame": params.get("frames/baselink"),
        "odom_frame": params.get("frames/odom"),
        "record_catmux_topics": bag_record_topics,
        "record_catmux_mode": "raw_inputs" if bag_records_raw_inputs else "slam_outputs_only",
    }


def parse_bag_path(path_text: str) -> tuple[Path, Path | None]:
    path = Path(path_text).expanduser().resolve()
    if path.is_dir():
        metadata_path = path / "metadata.yaml"
        return path, metadata_path if metadata_path.exists() else None

    if path.is_file():
        if path.name == "metadata.yaml":
            return path.parent, path
        if path.suffix == ".db3":
            metadata_path = path.parent / "metadata.yaml"
            return path.parent, metadata_path if metadata_path.exists() else None

    raise CheckerError(f"Unsupported bag path: {path_text}")


@dataclass
class BagFile:
    path: Path
    label: str


@dataclass
class TopicSummary:
    name: str
    msg_type: str
    message_count: int = 0
    bag_start_ns: int | None = None
    bag_end_ns: int | None = None
    approx_frequency_hz: float | None = None
    bag_timestamp_monotonic: bool | None = None
    bag_timestamp_non_monotonic_count: int | None = None
    header_timestamp_monotonic: bool | None = None
    header_timestamp_non_monotonic_count: int | None = None
    header_stamp_all_zero: bool | None = None
    frame_ids: list[str] = field(default_factory=list)
    point_fields_common: list[str] = field(default_factory=list)
    point_fields_seen: list[str] = field(default_factory=list)
    bag_gap_stats: dict[str, float | int | None] = field(default_factory=dict)
    header_gap_stats: dict[str, float | int | None] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)


class CdrReader:
    def __init__(self, data: bytes) -> None:
        if len(data) < 4:
            raise CheckerError("Serialized message is too short to contain a CDR header")
        self.data = data
        self.base_offset = 4
        self.offset = self.base_offset
        self.endian = "<" if data[1] == 1 else ">"

    def align(self, alignment: int) -> None:
        remainder = (self.offset - self.base_offset) % alignment
        if remainder:
            self.offset += alignment - remainder

    def _read(self, fmt: str, alignment: int) -> tuple[Any, ...]:
        self.align(alignment)
        size = struct.calcsize(fmt)
        end = self.offset + size
        if end > len(self.data):
            raise CheckerError("Serialized message ended unexpectedly")
        values = struct.unpack_from(self.endian + fmt, self.data, self.offset)
        self.offset = end
        return values

    def read_int32(self) -> int:
        return self._read("i", 4)[0]

    def read_uint32(self) -> int:
        return self._read("I", 4)[0]

    def read_uint8(self) -> int:
        return self._read("B", 1)[0]

    def read_bool(self) -> bool:
        return bool(self.read_uint8())

    def read_float64(self) -> float:
        return self._read("d", 8)[0]

    def read_float32(self) -> float:
        return self._read("f", 4)[0]

    def read_string(self) -> str:
        length = self.read_uint32()
        end = self.offset + length
        if end > len(self.data):
            raise CheckerError("Serialized string exceeded message length")
        raw = self.data[self.offset:end]
        self.offset = end
        if raw.endswith(b"\x00"):
            raw = raw[:-1]
        return raw.decode("utf-8", errors="replace")

    def skip(self, size: int, alignment: int = 1) -> None:
        self.align(alignment)
        end = self.offset + size
        if end > len(self.data):
            raise CheckerError("Tried to skip beyond serialized message length")
        self.offset = end


def read_time(reader: CdrReader) -> int:
    sec = reader.read_int32()
    nanosec = reader.read_uint32()
    return sec * 1_000_000_000 + nanosec


def read_header(reader: CdrReader) -> dict[str, Any]:
    return {"stamp_ns": read_time(reader), "frame_id": reader.read_string()}


def read_point_field(reader: CdrReader) -> dict[str, Any]:
    field_name = reader.read_string()
    offset = reader.read_uint32()
    datatype = reader.read_uint8()
    reader.align(4)
    count = reader.read_uint32()
    return {
        "name": field_name,
        "offset": offset,
        "datatype": datatype,
        "count": count,
    }


def decode_pointcloud2(data: bytes) -> dict[str, Any]:
    reader = CdrReader(data)
    header = read_header(reader)
    height = reader.read_uint32()
    width = reader.read_uint32()
    field_count = reader.read_uint32()
    fields = [read_point_field(reader) for _ in range(field_count)]
    is_bigendian = reader.read_bool()
    reader.align(4)
    point_step = reader.read_uint32()
    row_step = reader.read_uint32()
    data_len = reader.read_uint32()
    points_data_offset = reader.offset
    reader.skip(data_len)
    is_dense = reader.read_bool()

    return {
        "header": header,
        "height": height,
        "width": width,
        "fields": fields,
        "field_names": [field["name"] for field in fields],
        "is_bigendian": is_bigendian,
        "point_step": point_step,
        "row_step": row_step,
        "data_size": data_len,
        "points_data_offset": points_data_offset,
        "is_dense": is_dense,
    }


def decode_imu(data: bytes) -> dict[str, Any]:
    reader = CdrReader(data)
    header = read_header(reader)
    for _ in range(4):
        reader.read_float64()
    for _ in range(9):
        reader.read_float64()
    angular_velocity = [reader.read_float64() for _ in range(3)]
    for _ in range(9):
        reader.read_float64()
    linear_acceleration = [reader.read_float64() for _ in range(3)]
    for _ in range(9):
        reader.read_float64()
    return {
        "header": header,
        "angular_velocity": angular_velocity,
        "linear_acceleration": linear_acceleration,
    }


def decode_imu_full(data: bytes) -> dict[str, Any]:
    reader = CdrReader(data)
    header = read_header(reader)
    orientation = [reader.read_float64() for _ in range(4)]
    for _ in range(9):
        reader.read_float64()
    angular_velocity = [reader.read_float64() for _ in range(3)]
    for _ in range(9):
        reader.read_float64()
    linear_acceleration = [reader.read_float64() for _ in range(3)]
    for _ in range(9):
        reader.read_float64()
    return {
        "header": header,
        "orientation": orientation,
        "angular_velocity": angular_velocity,
        "linear_acceleration": linear_acceleration,
    }


def decode_header_only(data: bytes) -> dict[str, Any]:
    reader = CdrReader(data)
    return {"header": read_header(reader)}


def decode_message_header(data: bytes, msg_type: str) -> dict[str, Any] | None:
    if msg_type == "sensor_msgs/msg/PointCloud2":
        return decode_pointcloud2(data)
    if msg_type == "sensor_msgs/msg/Imu":
        return decode_imu(data)
    if msg_type in HEADER_FIRST_TYPES:
        return decode_header_only(data)
    return None


def summarize_gaps(gaps_ns: list[int]) -> dict[str, float | int | None]:
    if not gaps_ns:
        return {
            "count": 0,
            "median_s": None,
            "mean_s": None,
            "max_s": None,
        }
    return {
        "count": len(gaps_ns),
        "median_s": statistics.median(gaps_ns) / 1e9,
        "mean_s": statistics.fmean(gaps_ns) / 1e9,
        "max_s": max(gaps_ns) / 1e9,
    }


class SqliteBag:
    def __init__(self, bag_dir: Path, metadata_path: Path | None) -> None:
        self.bag_dir = bag_dir
        self.metadata_path = metadata_path
        self.metadata = load_yaml(metadata_path) if metadata_path else None
        self.storage_identifier = self._detect_storage_identifier()
        if self.storage_identifier not in SUPPORTED_STORAGE:
            raise CheckerError(
                f"Unsupported storage backend '{self.storage_identifier}'. "
                "This checker currently supports sqlite3 bags."
            )
        self.files = self._resolve_files()

    def _detect_storage_identifier(self) -> str:
        info = nested_get(self.metadata or {}, "rosbag2_bagfile_information", default={})
        if isinstance(info, dict) and info.get("storage_identifier"):
            return str(info["storage_identifier"])
        return "sqlite3"

    def _resolve_files(self) -> list[BagFile]:
        files: list[BagFile] = []
        file_entries = nested_get(
            self.metadata or {},
            "rosbag2_bagfile_information",
            "relative_file_paths",
            default=[],
        )
        if isinstance(file_entries, list) and file_entries:
            for entry in file_entries:
                files.append(BagFile(path=self.bag_dir / str(entry), label=str(entry)))
            return files

        for path in sorted(self.bag_dir.glob("*.db3")):
            files.append(BagFile(path=path, label=path.name))
        if files:
            return files

        raise CheckerError(f"No .db3 bag files found under {self.bag_dir}")

    def list_topics(self) -> dict[str, dict[str, Any]]:
        topics: dict[str, dict[str, Any]] = {}
        for bag_file in self.files:
            with sqlite3.connect(bag_file.path) as conn:
                rows = conn.execute(
                    "SELECT id, name, type, serialization_format, offered_qos_profiles FROM topics"
                )
                for topic_id, name, msg_type, serialization_format, qos in rows:
                    topic = topics.setdefault(
                        name,
                        {
                            "name": name,
                            "type": msg_type,
                            "serialization_format": serialization_format,
                            "offered_qos_profiles": qos,
                            "file_topic_ids": {},
                        },
                    )
                    if topic["type"] != msg_type:
                        raise CheckerError(
                            f"Topic {name} changed type across bag files: "
                            f"{topic['type']} vs {msg_type}"
                        )
                    topic["file_topic_ids"][bag_file.label] = topic_id
        return topics

    def aggregate_topic_stats(self, topics: dict[str, dict[str, Any]]) -> dict[str, TopicSummary]:
        summaries = {
            name: TopicSummary(name=name, msg_type=topic["type"])
            for name, topic in topics.items()
        }
        for bag_file in self.files:
            with sqlite3.connect(bag_file.path) as conn:
                for name, topic in topics.items():
                    topic_id = topic["file_topic_ids"].get(bag_file.label)
                    if topic_id is None:
                        continue
                    row = conn.execute(
                        "SELECT COUNT(*), MIN(timestamp), MAX(timestamp) FROM messages WHERE topic_id = ?",
                        (topic_id,),
                    ).fetchone()
                    if row is None:
                        continue
                    count, min_ts, max_ts = row
                    if not count:
                        continue
                    summary = summaries[name]
                    summary.message_count += int(count)
                    summary.bag_start_ns = (
                        min_ts
                        if summary.bag_start_ns is None
                        else min(summary.bag_start_ns, int(min_ts))
                    )
                    summary.bag_end_ns = (
                        max_ts
                        if summary.bag_end_ns is None
                        else max(summary.bag_end_ns, int(max_ts))
                    )
        for summary in summaries.values():
            if summary.message_count > 1 and summary.bag_start_ns is not None and summary.bag_end_ns is not None:
                duration_ns = summary.bag_end_ns - summary.bag_start_ns
                if duration_ns > 0:
                    summary.approx_frequency_hz = (summary.message_count - 1) / (duration_ns / 1e9)
        return summaries

    def iter_topic_messages(
        self, topic_name: str, topics: dict[str, dict[str, Any]]
    ) -> Iterable[tuple[int, bytes]]:
        topic = topics[topic_name]
        for bag_file in self.files:
            topic_id = topic["file_topic_ids"].get(bag_file.label)
            if topic_id is None:
                continue
            with sqlite3.connect(bag_file.path) as conn:
                rows = conn.execute(
                    "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY id",
                    (topic_id,),
                )
                for timestamp, data in rows:
                    yield int(timestamp), bytes(data)


def analyze_topic_messages(summary: TopicSummary, bag: SqliteBag, topics: dict[str, dict[str, Any]]) -> None:
    bag_gaps: list[int] = []
    header_gaps: list[int] = []
    last_bag_ts: int | None = None
    last_header_ts: int | None = None
    bag_non_monotonic = 0
    header_non_monotonic = 0
    header_zero_count = 0
    frame_ids: set[str] = set()
    point_fields_union: set[str] = set()
    point_fields_intersection: set[str] | None = None
    saw_header = False
    saw_empty_frame = False

    for bag_timestamp, serialized in bag.iter_topic_messages(summary.name, topics):
        if last_bag_ts is not None:
            delta = bag_timestamp - last_bag_ts
            bag_gaps.append(delta)
            if delta < 0:
                bag_non_monotonic += 1
        last_bag_ts = bag_timestamp

        decoded = decode_message_header(serialized, summary.msg_type)
        if decoded is None:
            continue

        header = decoded.get("header")
        if isinstance(header, dict):
            saw_header = True
            stamp_ns = int(header.get("stamp_ns", 0))
            frame_id = str(header.get("frame_id", "")).strip()
            if frame_id:
                frame_ids.add(frame_id)
            elif not saw_empty_frame:
                saw_empty_frame = True
                summary.notes.append("Saw at least one empty frame_id")
            if stamp_ns == 0:
                header_zero_count += 1
            if last_header_ts is not None:
                delta = stamp_ns - last_header_ts
                header_gaps.append(delta)
                if delta < 0:
                    header_non_monotonic += 1
            last_header_ts = stamp_ns

        field_names = decoded.get("field_names")
        if field_names is not None:
            current_fields = set(str(name) for name in field_names)
            point_fields_union.update(current_fields)
            if point_fields_intersection is None:
                point_fields_intersection = set(current_fields)
            else:
                point_fields_intersection &= current_fields

    summary.bag_gap_stats = summarize_gaps(bag_gaps)
    summary.bag_timestamp_non_monotonic_count = bag_non_monotonic
    summary.bag_timestamp_monotonic = bag_non_monotonic == 0

    if saw_header:
        summary.header_gap_stats = summarize_gaps(header_gaps)
        summary.header_timestamp_non_monotonic_count = header_non_monotonic
        summary.header_timestamp_monotonic = header_non_monotonic == 0
        summary.header_stamp_all_zero = header_zero_count == summary.message_count
        summary.frame_ids = sorted(frame_ids)

    if point_fields_union:
        summary.point_fields_seen = sorted(point_fields_union)
        summary.point_fields_common = sorted(point_fields_intersection or set())


def extract_first_point_timestamp_ns(data: bytes) -> int | None:
    decoded = decode_pointcloud2(data)
    fields = decoded["fields"]
    point_step = int(decoded["point_step"])
    data_size = int(decoded["data_size"])
    data_offset = int(decoded["points_data_offset"])
    if point_step <= 0 or data_size < point_step:
        return None

    timestamp_field = next((field for field in fields if field["name"] == "timestamp"), None)
    if timestamp_field is None:
        return None

    field_offset = int(timestamp_field["offset"])
    datatype = int(timestamp_field["datatype"])
    if field_offset < 0 or field_offset >= point_step:
        return None

    endian = ">" if decoded["is_bigendian"] else "<"
    offset = data_offset + field_offset
    if offset >= len(data):
        return None

    if datatype == 8:
        value = struct.unpack_from(endian + "d", data, offset)[0]
    elif datatype == 7:
        value = struct.unpack_from(endian + "f", data, offset)[0]
    elif datatype == 6:
        value = struct.unpack_from(endian + "I", data, offset)[0]
    else:
        return None

    if value > 1e12:
        return int(value)
    return int(value * 1e9)
