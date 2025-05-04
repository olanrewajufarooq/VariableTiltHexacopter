# Hover - PD Control
ros2 launch geometric_controllers path_following.launch.py controller_type:=PD path:=hover path_altitude:=5.0 path_period:=20.0 Kp_att:='[0.05,0.05,0.05]' Kp_pos:='[0.05,0.05,1.28]' Kd:='[0.0,0.0,0.0,0.0,0.0,2.55]'

# Hover - Feedback Linearization
ros2 launch geometric_controllers path_following.launch.py controller_type:=FeedLin path:=hover path_altitude:=5.0 path_period:=20.0 Kp_att:='[0.05,0.05,0.05]' Kp_pos:='[0.05,0.05,1.28]' Kd:='[0.0,0.0,0.0,0.0,0.0,2.55]'

# Take-Off and Land (Slow) - PD Control
ros2 launch geometric_controllers path_following.launch.py controller_type:=PD path:=takeoff_land path_altitude:=5.0 path_period:=120.0 Kp_att:='[0.5,0.5,0.5]' Kp_pos:='[5.5,0.5,5.28]' Kd:='[0.0,0.0,0.0,2.0,0.5,3.55]'

# Take-Off and Land (Slow) - Feedback Linearization
ros2 launch geometric_controllers path_following.launch.py controller_type:=FeedLin path:=takeoff_land path_altitude:=5.0 path_period:=120.0 Kp_att:='[0.5,0.5,0.5]' Kp_pos:='[5.5,0.5,5.28]' Kd:='[0.0,0.0,0.0,6.0,0.5,5.55]'

# Take-Off and Land (Fast) - PD Control
ros2 launch geometric_controllers path_following.launch.py controller_type:=PD path:=takeoff_land path_altitude:=5.0 path_period:=15.0 Kp_att:='[0.5,0.5,0.5]' Kp_pos:='[15.5,15.5,15.28]' Kd:='[0.0,0.0,0.0,10.0,10.5,10.55]'

# Take-Off and Land (Fast) - Feedback Linearization
ros2 launch geometric_controllers path_following.launch.py controller_type:=FeedLin path:=takeoff_land path_altitude:=5.0 path_period:=15.0 Kp_att:='[0.5,0.5,0.5]' Kp_pos:='[15.5,15.5,15.28]' Kd:='[0.0,0.0,0.0,10.0,10.5,10.55]'

# Square - PD Control (Slow)
ros2 launch geometric_controllers path_following.launch.py controller_type:=PD path:=square path_altitude:=5.0 path_period:=120.0 Kp_att:='[5.5,5.5,5.5]' Kp_pos:='[35.5,35.5,65.28]' Kd:='[2.05,2.05,2.05,20.0,20.5,20.55]'

# Square - Feedback Linearization (Slow)
ros2 launch geometric_controllers path_following.launch.py controller_type:=PD path:=square path_altitude:=5.0 path_period:=120.0 Kp_att:='[5.5,5.5,5.5]' Kp_pos:='[35.5,35.5,65.28]' Kd:='[2.05,2.05,2.05,20.0,20.5,20.55]'

# Another - PD Control

# Another - Feedback Linearization
