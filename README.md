ndt_mapping
====
This is the exraction of the mapping  from the [Autoware](https://github.com/autowarefoundation/autoware)

## Overview
ndt_mapping package

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


