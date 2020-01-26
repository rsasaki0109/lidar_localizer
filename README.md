ndt_mapping
====
This is the exraction of the mapping  from the [Autoware](https://github.com/autowarefoundation/autoware)

## Overview
ndt_mapping package

## IO
ndt_mapping 
- input  
/points_raw (sensor_msgs/PointCloud2)  
- output  
/ndt_map (sensor_msgs/PointCloud2)  
/curent_pose (geometry_msgs/PoseStamped) 

ndt_matching  
- input   
/filtered_points (sensor_msgs/PointCloud2)  
/points_map (sensor_msgs/PointCloud2)  
- output  
/curent_pose (geometry_msgs/PoseStamped)  

## Parameter

ndt
|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|max_iter|int|max iteration for alignment |25|
|step_size|double|step_size maximum step length[m]|0.1|
|ndt_res|double|resolution side length of voxels[m]|1.0|
|transform_epsilon|double|transform epsilon to stop iteration|0.1|
|voxel_leaf_size|double|a down sample size of a input cloud[m]|0.2|

ndt_mapping 
|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|min_add_scan_shift|double|a moving distance of a map update[m]|1.5|

ndt_matching 
|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|use_gnss||||
|queue_size||||
|offset||||
|get_height||||
|use_local_transform||||
|use_odom||||
|use_imu||||

## Usage
### Mapping 

```
roslaunch ndt_mapping ndt_mapping.launch
```

to save a map

```
rosrun pcl_ros pointcloud_to_pcd input:=/ndt_map prefix:=map
```

### Matching


```
roslaunch ndt_mapping ndt_matching.launch
```

```
rosrun pcl_ros pcd_to_pointcloud map_0.pcd /cloud_pcd:=/points_map
```


