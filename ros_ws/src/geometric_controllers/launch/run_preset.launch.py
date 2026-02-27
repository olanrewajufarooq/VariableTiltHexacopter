import os
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


HEX_DESC_PKG = "hexacopter_description"
CTRL_PKG = "geometric_controllers"


def _load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        d = yaml.safe_load(f) or {}
    if not isinstance(d, dict):
        raise ValueError(f"Expected mapping in {path}")
    return d


def _presets_dir() -> str:
    pkg = get_package_share_directory(CTRL_PKG)
    return os.path.join(pkg, "config", "presets")


def _defaults_dir() -> str:
    pkg = get_package_share_directory(CTRL_PKG)
    return os.path.join(pkg, "config", "defaults")


def _deep_update(base: dict, override: dict) -> dict:
    """Recursively merge override into base (returns a new dict)."""
    out = dict(base)
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_update(out[k], v)
        else:
            out[k] = v
    return out


def _load_defaults() -> dict:
    d = _defaults_dir()
    if not os.path.isdir(d):
        return {}
    merged: dict = {}
    for fn in sorted(os.listdir(d)):
        if not fn.endswith(".yaml") or fn.startswith("."):
            continue
        merged = _deep_update(merged, _load_yaml(os.path.join(d, fn)))
    return merged


def _available_presets() -> list[str]:
    d = _presets_dir()
    if not os.path.isdir(d):
        return []
    names: list[str] = []
    for fn in os.listdir(d):
        if fn.endswith(".yaml") and not fn.startswith("."):
            names.append(os.path.splitext(fn)[0])
    names.sort()
    return names


def _load_preset(*, preset: str, preset_file: str) -> dict:
    if preset_file:
        p = os.path.expanduser(preset_file)
        if not os.path.isabs(p):
            p = os.path.join(_presets_dir(), p)
        return _load_yaml(p)

    path = os.path.join(_presets_dir(), f"{preset}.yaml")
    return _load_yaml(path)


def _load_effective_config(*, preset: str, preset_file: str) -> dict:
    defaults = _load_defaults()
    preset_cfg = _load_preset(preset=preset, preset_file=preset_file)
    return _deep_update(defaults, preset_cfg)


def _setup(context, *args, **kwargs):
    preset_name = LaunchConfiguration("preset").perform(context)
    preset_file = LaunchConfiguration("preset_file").perform(context).strip()

    available = _available_presets()
    if not preset_file and preset_name not in available:
        raise ValueError(
            f"Unknown preset '{preset_name}'. Available: {available}"
        )

    cfg = _load_effective_config(preset=preset_name, preset_file=preset_file)
    spawn_cfg = cfg.get("spawn", {}) or {}
    alloc_cfg = cfg.get("allocation", {}) or {}
    ctrl_cfg = cfg.get("controller", {}) or {}
    gains_cfg = ctrl_cfg.get("gains", {}) or {}
    traj_cfg = cfg.get("trajectory", {}) or {}
    node_cfg = cfg.get("node", {}) or {}
    payload_cfg = cfg.get("payload", {}) or {}
    bag_cfg = cfg.get("bag", {}) or {}
    vehicle_cfg = cfg.get("vehicle", {}) or {}

    hex_pkg = get_package_share_directory(HEX_DESC_PKG)
    ctrl_pkg = get_package_share_directory(CTRL_PKG)

    # --- Spawn + bridge -----------------------------------------------------------
    spawn_launch_file = os.path.join(
        hex_pkg, "launch", str(spawn_cfg.get("launch", "spawn_robot.launch.py"))
    )
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_file),
        launch_arguments={
            "start_pos": str(spawn_cfg.get("start_pos", "0.0 0.0 0.3")),
            "world": str(spawn_cfg.get("world", "empty.sdf")),
        }.items(),
    )

    # --- Control allocation params ------------------------------------------------
    hex_params = os.path.join(hex_pkg, "config", "hexacopter_params.yaml")
    hex_all = _load_yaml(hex_params)
    ca_params = hex_all.get("control_allocation_node", {}).get("ros__parameters", {})
    if not isinstance(ca_params, dict):
        ca_params = {}
    ca_params["allocation_method"] = str(alloc_cfg.get("method", "fixed_tilt"))
    ca_params["tilt_angle"] = float(alloc_cfg.get("tilt_angle", 0.57))

    control_allocation_node = Node(
        package=HEX_DESC_PKG,
        executable="control_allocation_node",
        name="control_allocation_node",
        output="screen",
        parameters=[ca_params],
    )

    # --- Trajectory node ----------------------------------------------------------
    traj_params = {
        "rate_hz": float(node_cfg.get("rate_hz", 100.0)),
        "path": str(traj_cfg.get("path", "hover")),
        "path_scale": float(traj_cfg.get("scale", 5.0)),
        "path_period": float(traj_cfg.get("period", 20.0)),
        "path_altitude": float(traj_cfg.get("altitude", 5.0)),
        "path_start_with_hover": bool(traj_cfg.get("start_with_hover", True)),
        "payload_events_enabled": bool(payload_cfg.get("events_enabled", False)),
        "payload_pick_wait_s": float(payload_cfg.get("pick_wait_s", 2.0)),
        "payload_drop_time_s": float(payload_cfg.get("drop_time_s", -1.0)),
    }

    trajectory_node = Node(
        package=CTRL_PKG,
        executable="trajectory_node",
        name="trajectory_node",
        output="screen",
        parameters=[traj_params],
    )

    # --- Control node -------------------------------------------------------------
    ctrl_params = {
        "rate_hz": float(node_cfg.get("rate_hz", 100.0)),
        "gravity": float(vehicle_cfg.get("gravity", 9.8)),
        "mass": float(vehicle_cfg.get("mass", 3.646)),
        "I": [float(x) for x in vehicle_cfg.get("I", [0.1] * 6)],
        "CoG": [float(x) for x in vehicle_cfg.get("CoG", [0.0, 0.0, 0.0])],
        "controller_type": str(ctrl_cfg.get("controller_type", "PD")),
        "adaptation_type": str(ctrl_cfg.get("adaptation_type", "None")),
        "potential_type": str(ctrl_cfg.get("potential_type", "liealgebra")),
        "Kp_att": [float(x) for x in gains_cfg.get("Kp_att", [5.5, 5.5, 5.5])],
        "Kp_pos": [float(x) for x in gains_cfg.get("Kp_pos", [5.5, 5.5, 5.5])],
        "Kd": [float(x) for x in gains_cfg.get("Kd", [2.05] * 6)],
    }

    control_node = Node(
        package=CTRL_PKG,
        executable="control_node",
        name="control_node",
        output="screen",
        parameters=[ctrl_params],
    )

    actions = [spawn_launch, trajectory_node, control_node, control_allocation_node]

    # --- Optional bag recording ---------------------------------------------------
    record_bag_raw = LaunchConfiguration("record_bag").perform(context).lower()
    if record_bag_raw == "auto":
        record_bag = bool(bag_cfg.get("record", True))
    else:
        record_bag = record_bag_raw in ("true", "1", "yes")
    if record_bag:
        bag_root = os.path.expanduser(LaunchConfiguration("bag_root").perform(context))
        os.makedirs(bag_root, exist_ok=True)

        bag_name = LaunchConfiguration("bag_name").perform(context).strip()
        if not bag_name:
            ts = datetime.now().strftime("%Y%m%d-%H%M%S")
            bag_name = f"{ts}_{preset_name}_bag"
        bag_path = os.path.join(bag_root, bag_name)
        topics = bag_cfg.get("topics", [])
        if not topics:
            raise ValueError(f"Preset '{preset_name}' has no bag_topics")

        # Use the CLI (`ros2 bag record`) instead of `rosbag2_transport/record`.
        # Some installations do not ship a `rosbag2_transport` libexec entrypoint.
        bag = ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "-o",
                bag_path,
                *[str(t) for t in topics],
            ],
            output="screen",
        )
        actions.append(bag)

    return actions


def generate_launch_description():
    preset_names = _available_presets()

    declare_preset = DeclareLaunchArgument(
        "preset",
        default_value=preset_names[0] if preset_names else "nominal_demo",
        description=f"Run preset name: {', '.join(preset_names)}",
    )

    declare_preset_file = DeclareLaunchArgument(
        "preset_file",
        default_value="",
        description="Optional preset YAML file path (absolute or relative to config/presets)",
    )
    declare_record_bag = DeclareLaunchArgument(
        "record_bag",
        default_value="auto",
        description="Record a rosbag2 during the run (true/false/auto uses preset)",
    )
    declare_bag_root = DeclareLaunchArgument(
        "bag_root",
        default_value="~/VariableTiltHexacopter/ros_bags/bags",
        description="Directory where bag folders are created",
    )

    declare_bag_name = DeclareLaunchArgument(
        "bag_name",
        default_value="",
        description="Bag folder name (empty => timestamped)",
    )

    return LaunchDescription(
        [
            declare_preset,
            declare_preset_file,
            declare_record_bag,
            declare_bag_root,
            declare_bag_name,
            OpaqueFunction(function=_setup),
        ]
    )
