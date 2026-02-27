#!/bin/bash

# ──────────────── CONFIGURATION ────────────────
TOP_DIR=~/VariableTiltHexacopter
WORKSPACE=$TOP_DIR/ros_ws
BAG_DIR=$TOP_DIR/ros_bags

DURATION=30            # Duration to run [seconds]
START_SHIFT=2         # Start shift for plotting [seconds]

# Preset-driven run (see geometric_controllers/config/presets/*.yaml)
PRESET=nominal_demo  # nominal_demo, adaptive_demo, nominal_paper, adaptive_paper

# Auto-generated names
TS=$(date +"%Y%m%d-%H%M%S")
BAG_NAME="${TS}_${PRESET}_bag"
BAG_PATH="${BAG_DIR}/bags/${BAG_NAME}"
PLOT_PATH="${BAG_DIR}/plot/${PRESET}/${TS}"

# ──────────────── STEP 1: Source ROS 2 Workspace ────────────────
echo "🔧 Sourcing ROS 2 workspace..."
source /opt/ros/humble/setup.bash
source $WORKSPACE/install/local_setup.bash

# ──────────────── STEP 2: Launch Simulation ────────────────
echo "🚀 Launching simulation preset=$PRESET ..."
ros2 launch geometric_controllers run_preset.launch.py \
    preset:=$PRESET \
    record_bag:=true \
    bag_root:=$BAG_DIR/bags \
    bag_name:=$BAG_NAME &
LAUNCH_PID=$!

# Give simulation some time to start
sleep 5

# ──────────────── STEP 3: Wait for Completion ────────────────
echo "⏳ Letting simulation run for $DURATION seconds..."
sleep $DURATION

# ──────────────── STEP 4: Stop Simulation ────────────────
echo "🛑 Stopping launch..."
kill $LAUNCH_PID


# ──────────────── STEP 5: Plot Results ────────────────
echo "📊 Plotting results into $PLOT_PATH..."
mkdir -p $PLOT_PATH
python3 $BAG_DIR/plot_hexacopter.py -b $BAG_PATH -o $PLOT_PATH -d $DURATION -s $START_SHIFT

echo "✅ All done!"

# ─────────
# Stop Simulation (ensure processes exit)
sleep 2
