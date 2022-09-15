/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 Localization and mapping program using Normal Distributions Transform

 Yuki KITSUKAWA
 */
#define OUTPUT  // If you want to output "position_log.txt", "#define OUTPUT".

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/ndt.h>

#include <time.h>

class ndt_mapping
{
public:
  ndt_mapping();
  ~ndt_mapping();

private:

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber points_sub_;

  struct pose{double x,y,z;double roll,pitch,yaw;};
  struct pose current_pose_,current_pose_imu_;
  struct pose previous_pose_;

  pcl::PointCloud<pcl::PointXYZI> map_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

  // Default values
  int max_iter_ ;        // Maximum iterations
  double ndt_res_ ;      // Resolution
  double step_size_ ;   // Step size
  double trans_eps_ ;  // Transformation epsilon

  double voxel_leaf_size_;// Leaf size of VoxelGrid filter.

  double scan_rate_;
  double min_scan_range_;
  double max_scan_range_;
  bool use_imu_;

  std::string robot_frame_;
  std::string map_frame_;

  ros::Publisher ndt_map_pub_, current_pose_pub_;
  geometry_msgs::PoseStamped current_pose_msg_;

  tf::TransformBroadcaster br_;

  int initial_scan_loaded;
  double min_add_scan_shift_;

  double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
  Eigen::Matrix4f tf_btol_, tf_ltob_;//base_link2localizer等の略?

  bool _incremental_voxel_update;
  
  bool is_first_map_;

  std::ofstream ofs;
  std::string filename;

  void imu_calc(ros::Time current_time);
  void imu_callback(const sensor_msgs::Imu::Ptr& input);
  void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input);
};
