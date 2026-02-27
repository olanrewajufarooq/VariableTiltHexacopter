# Agent Guide (VariableTiltHexacopter)

This repository is a ROS 2 (Humble) + Gazebo simulation workspace for a variable-tilt hexacopter.
Most code lives under `ros_ws/src/`.

Quick Repo Map
- `ros_ws/`: colcon workspace (`src/`, `build/`, `install/`, `log/`)
- `ros_ws/src/geometric_controllers`: ament_python controllers + pytest tests
- `ros_ws/src/hexacopter_description`: ament_python description/launch/models
- `ros_ws/src/state_estimation`: ament_python state estimation (declares ament lint deps)
- `ros_ws/src/ros2_numpy`: ament_cmake python pkg with ctest/pytest integration
- `bash_scripts/`: experiment helpers (some commands delete output dirs)

Build / Test / Lint
Shell Setup
- ROS 2 Humble is assumed: `source /opt/ros/humble/setup.bash`
- Recommended: work from `ros_ws/` for colcon commands.

Local Shell Helpers (`~/.bashrc`)

This machine already defines convenience functions (paths assume `~/VariableTiltHexacopter`):
```bash
src-hexacopter        # cd ros_ws + source ROS + source overlay
build-hexacopter      # rosdep install + colcon build + source overlay
run-hexacopter        # runs bash_scripts/run_all.sh
record-hexacopter     # records a small set of rosbag topics
```

Install Dependencies (once per machine / when deps change)
```bash
cd ros_ws
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y
```

Build (whole workspace)
```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash
```

Build (single package)
```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select geometric_controllers
source install/local_setup.bash
```

Developer Build Tips
- Faster Python iteration: `colcon build --symlink-install`
- Debug symbols: `colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`
- Clean rebuild (destructive): remove `ros_ws/build/ ros_ws/install/ ros_ws/log/`.

Run Tests (whole workspace)
```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Run Tests (single package)
```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select ros2_numpy --event-handlers console_direct+
colcon test-result --verbose
```

Run a Single Test (ros2_numpy)
`ros2_numpy` registers named ctest targets via `ament_cmake_pytest`:
`pointclouds`, `images`, `occupancygrids`, `geometry`, `quaternions`, `laserscan`.

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select ros2_numpy --ctest-args -R geometry
colcon test-result --verbose
```

Run a Single Test (pytest direct; geometric_controllers)
`geometric_controllers` keeps tests in `ros_ws/src/geometric_controllers/tests/`.
If `colcon test` does not pick them up in your environment, run pytest directly:

```bash
python -m pytest -q ros_ws/src/geometric_controllers/tests/test_controller.py
python -m pytest -q ros_ws/src/geometric_controllers/tests/test_controller.py -k gravity
python -m pytest -q ros_ws/src/geometric_controllers/tests/test_controller.py::test_<name>
```

Lint / Style Checks
No repo-wide config was found for:
- `.pre-commit-config.yaml`, `.editorconfig`, `pyproject.toml`, `.clang-format`
Notes:
- `state_estimation` declares `ament_flake8` and `ament_pep257` as test dependencies, but no dedicated lint test entrypoints were discovered.
- Prefer adding explicit, reproducible lint commands in-package if you introduce new lint tooling.

Common Runtime Commands (ROS 2)
After building + sourcing `ros_ws/install/local_setup.bash`, examples:
- Spawn robot:
  - `ros2 launch hexacopter_description spawn_robot.launch.py`
- Simple control:
  - `ros2 launch hexacopter_description simple_control.launch.py angles:="[...]" motor_speeds:="[...]"`
- Geometric controller (hover / path following):
  - `ros2 launch geometric_controllers hover_control.launch.py`
  - `ros2 launch geometric_controllers path_following.launch.py path:=square`

Helper Scripts
`bash_scripts/run_all.sh` launches controllers, records bags, and plots results.
- It deletes any existing output bag directory (`rm -rf`) before recording.
- It assumes the repo is located at `~/VariableTiltHexacopter`.

Code Style Guidelines

General

- Match existing patterns in the package you touch (ROS 2 conventions differ across ament_python vs ament_cmake).
- Keep changes localized; avoid drive-by refactors in math-heavy code unless required.
- Keep core logic (math/control/estimation utilities) ROS-agnostic: no `rclpy` imports, no parameter access, no publishers/subscribers.
- Keep ROS logic thin: nodes/launch files should only wire parameters, topics, and scheduling around the core logic.

Python Formatting

- Indentation: 4 spaces.
- Keep functions short; prefer small helpers over long procedural blocks.
- Use f-strings for messages.
- Avoid non-ASCII in new code/comments unless a file already uses it.

Imports

- Group imports in this order, separated by a blank line:
  1) stdlib
  2) third-party (numpy/scipy/opencv/gtsam/etc)
  3) ROS 2 (`rclpy`, messages)
  4) local package imports
- Prefer absolute imports within a package (e.g., `from geometric_controllers.utils import Ad`).
- No wildcard imports.

Types and Shapes

- Add type hints for public APIs (entrypoints, node callbacks, library functions used by multiple modules).
- For numpy-heavy math, prioritize explicit shapes over verbose typing:
  - Use `np.ndarray` type hints where helpful, and validate shapes at runtime.
  - Be explicit about column vectors (e.g., `(6, 1)`) vs flat arrays.
- When using matrix log/exp or other ops that can introduce complex rounding artifacts, only coerce to `.real` when justified and documented.

Naming

- Python modules/functions/variables: `snake_case`.
- Classes: `CapWords`.
- ROS nodes / console scripts: `<purpose>_node` and keep the `main()` entrypoint thin.
- Constants: `UPPER_SNAKE_CASE`.

Error Handling

- Raise `ValueError` for invalid inputs, parameter values, or shape mismatches (matches existing utilities).
- Use `assert` for internal invariants only (do not rely on it for validating ROS parameters).
- In ROS nodes, prefer `self.get_logger().error(...)` and fail fast rather than printing and continuing.
- When catching exceptions, either:
  - re-raise with context, or
  - return `None` explicitly and document the contract (several estimators return `None` on optimization failure).

ROS 2 Conventions

- Always source overlays in examples (`source /opt/ros/humble/setup.bash` then `source ros_ws/install/local_setup.bash`).
- Keep launch parameter names stable; validate parameters and log effective values.
- Avoid hardcoding absolute paths in library code; scripts may assume `~/VariableTiltHexacopter` but packages should not.

Tests

- Prefer deterministic unit tests that do not require Gazebo.
- Use `pytest` for new Python tests.
- Name tests `test_*.py` with functions `test_*`.

External Agent Rules

- Cursor rules: none found in `.cursor/rules/` and no `.cursorrules` file.
- Copilot rules: none found in `.github/copilot-instructions.md`.
