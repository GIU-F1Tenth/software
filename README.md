# software
Use `source ~/ros_humble/install/setup.bash` to source humble on the car.

# SLAM 
- ros2 launch nav2_bringup slam_launch.py params_file:=src/giu_f1t_system/f1tenth_stack/config/f1tenth_online_async_mapping.yaml 
 
- ros2 run nav2_map_server map_saver_cli 

- scp ubuntu@[ip]:/home/ubuntu/giu_f1tenth_ws/software/map_number.pgm /path/to/where/to/save/map.pgm

- convert input.png -colorspace Gray output.png /path/to/map.pgm