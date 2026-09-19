# furosim_ros_pkgs (ROS 1, Noetic)

ROS wrapper for the FURo-Sim RPC API: vehicle state, IMU, GPS, pressure, DVL, FLS / SSS sonar images and the AUV waypoint services (`move_to_position`, `move_on_path`, `move_to_z`, `hover`).

Build AirLib first (`./setup.sh` and `./build.sh` in the repository root), then

```bash
cd ros
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch furosim_ros_pkgs furosim_node.launch
```

The node connects to the simulator on `localhost:41451`; the sensors it publishes follow the vehicle's `settings.json`.
