# software
Use `source ~/ros_humble/install/setup.bash` to source humble on the car.

# SLAM 
- ros2 launch nav2_bringup slam_launch.py params_file:=src/giu_f1t_system/f1tenth_stack/config/f1tenth_online_async_mapping.yaml 
 
- ros2 run nav2_map_server map_saver_cli 

- scp ubuntu@[ip]:/home/ubuntu/giu_f1tenth_ws/software/map_number.pgm ~/path/to/where/to/save/map.pgm

on the host machine not the car !!!!!!

- convert input.png -colorspace Gray output.png /path/to/map.png

go to the python script dir
source ~/miniconda3/etc/profile.d/conda.sh 
~/software/libs/raceline-optimization$ conda activate racingline

conda env list

conda activate raceline

do the centerline opt

python3 main_globaltraj_f110.py --map_name e7_floor5_square --map_path ~/workspaces/racetracks/e7_floor5_square.csv --export_path /tmp/traj_race_cl.csv

- ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=640x480x30 enable_infra1:=false enable_infra2:=false enable_depth:=false enable_gyro:=false enable_accel:=false

- ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    color_width:=320 \
    color_height:=240 \
    color_fps:=6 \
    enable_depth:=false \
    enable_infra1:=false \
    enable_infra2:=false \
    enable_accel:=false \
    enable_gyro:=false \
    enable_pose:=false \
    enable_sync:=false \
    unite_imu_method:="none"

- sudo tegrastats

- colcon build --packages-select f1tenth_stack camera_obj_detection pure_pursuit gap_follower giu_f1t_behavior_tree watchdog trajectory_planning obj_detection opponent_tracker camera_obj_detection safety_node

- colcon build --packages-select f1tenth_stack pure_pursuit gap_follower trajectory_planning