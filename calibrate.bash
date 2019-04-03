source ~/.bashrc
rostopic pub /bebop/camera_control geometry_msgs/Twist "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: -90, z: 0}}"