# ROS 2 PCD Publisher

This ROS 2 package (`pcd_publisher`) provides a node that reads a PCD (Point Cloud Data) file and publishes its contents as a PointCloud2 message on a ROS 2 topic.

[![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)
[![Static Badge](https://img.shields.io/badge/ROS_2-Jazzy-34aec5)](https://docs.ros.org/en/jazzy/)



## Prerequisites

- ROS 2 (tested on Humble, but should work on other distributions)
- PCL (Point Cloud Library) 1.12 or later
- C++14 or later

## Installation

1. Create a new directory in your ROS 2 workspace's `src` folder:

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/omerssid/pcd_publisher.git
   ```

2. Build the package:

   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install --packages-select pcd_publisher
   ```

3. Source the setup file:

   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### `ROS 2` `PointCloud2` publisher (reads a `PCD` file)

Run the publisher node:

```bash
ros2 run pcd_publisher pcd_publisher --ros-args -p pcd_file_path:=/path/to/pcd
```

```bash
ros2 run pcd_publisher pcd_publisher --ros-args -p pcd_file_path:=/mnt/c/bag/pcd_test/campus1.pcd -p topic_name:=/points_campus
```

```bash
ros2 run pcd_publisher pcd_publisher --ros-args -p pcd_file_path:=/home/dev/autoware_map/gyor_campus/pointcloud_map.pcd -p topic_name:=/points_campus -p frame_id:=map_gyor_0
```

The node will publish the point cloud data on the `/pointcloud` topic.

### `ROS 2` `PointCloud2` subscriber (writes a `PCD` file)

Run the subscriber:

```bash
ros2 run pcd_publisher pcd_subscriber --ros-args -p pcd_file_path:=/path/to/pcd -p topic:=/pointcloud
```

```bash
ros2 run pcd_publisher pcd_subscriber --ros-args --params-file params01.yaml
```

```bash
ros2 run pcd_publisher pcd_subscriber --ros-args -p pcd_file_path:=/mnt/c/bag/pcd_test/ -p topic_name:=/lexus3/os_center/points -p frame_id:=map_gyor_0 -p continuous_saving:=true -p continuous_saving_rate:=0.5
```

The subscriber will listen to the `/pointcloud` topic and save the received point cloud data to a PCD file.


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Remarks

In VS code it is advised to add the following to include path:

``` r
${workspaceFolder}/**
/opt/ros/humble/include/**
/usr/include/pcl-1.12/**
/usr/include/eigen3/**
```

If you are not sure where your header files are use e.g.:
``` r
find /usr/include -name point_cloud.h
find /usr/include -name crop_box.h
```

## Related

- [github.com/jkk-research/lanelet2_rviz2](https://github.com/jkk-research/lanelet2_rviz2): `lanelet2_rviz2` ROS 2 package

You can use `lanelet2_rviz2` and `pcd_publisher` together to create similar view:

![](https://raw.githubusercontent.com/jkk-research/lanelet2_rviz2/refs/heads/lanelet2_lib_based/etc/lanelet_visu01.png)
