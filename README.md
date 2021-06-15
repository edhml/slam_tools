## SLAM Tools

These tools can help you quick start with 3D SLAM. Some of dependent packages need to be downloaded first.

# Introduction



# Dependent Packages

## 2D SLAM

Currently support slam_toolbox.

### slam_toolbox

```
```

## 3D SLAM

Currently support one type.

### lidarslam_ros2

This slam tool is forked from [rsasaki0109](https://github.com/rsasaki0109/lidarslam_ros2)

Insall via git clone

```
git clone https://github.com/edhml/lidarslam_ros2.git
```

## Driver (Options)

### Velodyne Driver

Model: vlp16

```
git clone https://github.com/edhml/velodyne.git
```

### Ouster Driver

Model: os1

```
git clone https://github.com/edhml/ros2_ouster_drivers.git
```

# Build workspace

```
colcon build --symlink-install

source install/local_setup.bash
```

# Start 3D SLAM

Run with velodyne-vlp16 LiDAR

```
ros2 launch slam_tools lidar_slam_3d_vlp16.launch.py
```

or run with ouster-os1 liDAR

```
ros2 launch slam_tools lidar_slam_3d_os1.launch.py
```

Save map with serivce call. The map will be saved in slam_tools where it launch workspace with a default name, which is map.pcd.

```
ros2 service call /map_save std_srvs/Empty
```

![lidar_slam_3d_image](/images/lidar_slam_3d_save_side_view.png?raw=true "Lidar SLAM 3D Image")

Review your 3D map by installing pcl-tools.

```
sudo apt install pcl-tools

pcl_viewer /path/to/your/map.pcd
```

![lidar_slam_3d_image](/images/pose_graph.png?raw=true "Lidar SLAM 3D Image")

# Demo Bags

Provide not yet. (Will be provided soon.)


