# SLAM Tools

## Introduction

This package mainly help you quick start both 2D and 3D SLAM due to lack information or tutorials on ROS2 middleware for newbie. The package will not focus on creating a new SLAM algorithm or constructing SLAM relevant knowledge. But we may add links to help you learn from original authors. 

Some of dependent packages need to be downloaded first. If you would like to simplified obtain all dependencies at once, refer to [quick_start_to_slam](https://github.com/edhml/quick_start_to_slam.git)

Currently verified vendor and LiDAR models: 

Velodyne: vlp16

Ouster: os1

What we have done:
- Help you quick start slam with common state-of-art 3D LiDAR by lidar odomtery configuration
- Fix some of driver, algorithm or configuration are not compatible to slam packages problem
- Unify the configuration of device drivers or ROS2 package, such as topic, frame_id and urdf ... etc
- Provide a ros2 bag meta-data for you to both start and learn 2D/3D slam on ROS2 framework, even you temporarily cannot afford these fancy devices

## 2D SLAM

Currently support slam_toolbox.

### Slam Toolbox

Why slam_toolbox?
- ROS2 middleware officially supported
- Contains the ability to do most everything with SLAM library, also free and paid
- Ordinary point-and-shoot 2D SLAM mobile robotics folks expect (start, map, save pgm file)
- Continuing to refine, remap, or continue mapping a saved (serialized) pose-graph at any time
- An optimization-based localization mode built on the pose-graph
- ... more but those are the highlights

For more slam_toolbox introduction, please refer to [SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox).

## 3D SLAM

Currently support lidarslam_ros2.

### lidarslam_ros2

Why lidarslam?
- Help you quick start LiDAR-based SLAM and get a 3D point cloud map
- Deploy with normal distribution transform algorithm, which is a popular NDT-base graph SLAM
- No need extra sensor, can be performed standalone.

This slam tool is forked from [rsasaki0109](https://github.com/rsasaki0109/lidarslam_ros2).

Insall via git clone

```
git clone https://github.com/edhml/lidarslam_ros2.git
```

## Driver (Optional)

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

## Start 2D SLAM

Run via velodyne-vlp16 LiDAR

```
ros2 launch slam_tools slam_toolbox_2d_vlp16.launch.py
```

or run via ouster-os1 LiDAR

```
ros2 launch slam_tools slam_toolbox_2d_os1.launch.py
```

![slam_toolbox_2d_image](/images/slam_toolbox_side_view.png?raw=true "Lidar SLAM 2D")

Save map with the following service call, then map will be save as a pgm image format.

```
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap
```

![slam_toolbox_2d_image](/images/slam_toolbox_map_2d.png?raw=true "Lidar SLAM 2D Image")

## Start 3D SLAM

Run via velodyne-vlp16 LiDAR

```
ros2 launch slam_tools lidar_slam_3d_vlp16.launch.py
```

or run via ouster-os1 LiDAR

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

## ROS2 Sim Bags

Not yet to provide. (Will be soon.)

## Future Work

- Integrate more fancy components with SLAM package
- Add various SLAM package for running on ROS2 framework
- Provide a much more huge trial metadata bag with different terrains

Let's move onto ROS2 for better development experience.

