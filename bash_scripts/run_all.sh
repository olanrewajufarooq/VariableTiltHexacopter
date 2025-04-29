#!/bin/bash

# ──────────────── CONFIGURATION ────────────────
WORKSPACE=~/VariableTiltHexacopter/ros_ws
BAG_DIR=$WORKSPACE/ros_bags

DURATION=90            # Duration to run [seconds]
START_SHIFT=10         # Start shift for plotting [seconds]

# Controller and Path Settings
PATH_TYPE=hover
CONTROLLER_TYPE=PD
ATT="[0.5,0.5,0.5]"
KP_POS="[0.1,0.1,2.0]"  
KD="[0.0, =0.0,0.005,0.0,0.0,0.5]"

# Auto-generated names
BAG_NAME="geometric_control_${CONTROLLER_TYPE}_${PATH_TYPE}"
BAG_PATH="${BAG_DIR}/${BAG_NAME}"
PLOT_PATH="${BAG_DIR}/plot/${CONTROLLER_TYPE}_${PATH_TYPE}"

# ──────────────── STEP 1: Source ROS 2 Workspace ────────────────
echo "🔧 Sourcing ROS 2 workspace..."
source $WORKSPACE/install/setup.bash

# ──────────────── STEP 2: Launch Simulation ────────────────
echo "🚀 Launching simulation with PATH=$PATH_TYPE CONTROLLER=$CONTROLLER_TYPE..."
ros2 launch geometric_controllers path_following.launch.py \
    path:=$PATH_TYPE \
    controller_type:=$CONTROLLER_TYPE \
    att:=$ATT \
    Kp_pos:=$KP_POS \
    Kd:=$KD &
LAUNCH_PID=$!

# Give simulation some time to start
sleep 5

# ──────────────── STEP 3: Start ROS2 Bag Recording ────────────────
echo "🎥 Recording ROS2 bag to $BAG_PATH..."
mkdir -p $BAG_DIR

# Delete old bag folder if it exists
if [ -d "$BAG_PATH" ]; then
    echo "🗑 Removing existing bag folder: $BAG_PATH"
    rm -rf "$BAG_PATH"
fi

ros2 bag record \
    /model/variable_tilt_hexacopter/odometry \
    /model/variable_tilt_hexacopter/desired_wrench \
    /model/variable_tilt_hexacopter/plot/motor_speed \
    /model/variable_tilt_hexacopter/plot/tilt_angle \
    -o $BAG_PATH &

BAG_PID=$!

# ──────────────── STEP 4: Wait for Completion ────────────────
echo "⏳ Letting simulation run for $DURATION seconds..."
sleep $DURATION

# ──────────────── STEP 5: Kill all background processes ────────────────
echo "🛑 Stopping simulation and recording..."
kill $LAUNCH_PID
sleep 1
kill $BAG_PID

# ──────────────── STEP 6: Plot Results ────────────────
echo "📊 Plotting results into $PLOT_PATH..."
mkdir -p $PLOT_PATH
python3 $BAG_DIR/plot_hexacopter.py -b $BAG_PATH -o $PLOT_PATH -d $DURATION -s $START_SHIFT

echo "✅ All done!"
