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

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>



struct pose{double x,y,z;double roll,pitch,yaw;};

// global variables
static pose previous_pose_, guess_pose, current_pose_, ndt_pose, added_pose, localizer_pose;

static ros::Time current_scan_time,previous_scan_time;
static ros::Duration scan_duration;

static double diff = 0.0, diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw;  // current_pose_ - previous_pose_

static double current_velocity_x = 0.0, current_velocity_y = 0.0, current_velocity_z = 0.0;


static pcl::PointCloud<pcl::PointXYZI> map, submap;

static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

// Default values
static int max_iter_ = 30;        // Maximum iterations
//static float ndt_res_ = 1.0;      // Resolution
static float ndt_res_ = 5.0;      // Resolution
static double step_size_ = 0.1;   // Step size
static double trans_eps_ = 0.01;  // Transformation epsilon

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size_ = 2.0;

static ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, t4_start, t4_end,
    t5_start, t5_end;
static ros::Duration d_callback, d1, d2, d3, d4, d5;

static ros::Publisher ndt_map_pub_, current_pose__pub_,guess_pose_linaer_pub;
static geometry_msgs::PoseStamped current_pose__msg_, guess_pose_msg;

static ros::Publisher ndt_stat_pub;
static std_msgs::Bool ndt_stat_msg;

static int initial_scan_loaded = 0;

static Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();

static double min_scan_range = 5.0;
//static double min_scan_range = 1.0;
static double max_scan_range = 200.0;
static double min_add_scan_shift_ = 1.0;
//static double min_add_scan_shift_ = 0.5;
static double max_submap_size =100.0;

static double _tf_x=0.0, _tf_y=0.0, _tf_z=0.0, _tf_roll=0.0, _tf_pitch=0.0, _tf_yaw=0.0;
static Eigen::Matrix4f tf_btol_, tf_ltob_;//base_link2localizer等の略?

static bool isMapUpdate = true;

static double fitness_score;

static int submap_num = 0;
static double submap_size = 0.0;

static std::ofstream ofs;
static std::string filename;

static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  double r;
  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI> tmp, scan;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  tf::Quaternion q;

  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
  tf::TransformBroadcaster br;
  tf::Transform transform;

  current_scan_time = input->header.stamp;

  pcl::fromROSMsg(*input, tmp);

  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
  {
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;

    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (min_scan_range < r && r < max_scan_range)
    {
      scan.push_back(p);
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  // Add initial point cloud to velodyne_map
  if (initial_scan_loaded == 0)
  {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol_);
    map += *transformed_scan_ptr;
    initial_scan_loaded = 1;
  }

  // Apply voxelgrid filter
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel_grid_filter_.setInputCloud(scan_ptr);
  voxel_grid_filter_.filter(*filtered_scan_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));

  ndt.setTransformationEpsilon(trans_eps_);
  ndt.setStepSize(step_size_);
  ndt.setResolution(ndt_res_);
  ndt.setMaximumIterations(max_iter_);
  ndt.setInputSource(filtered_scan_ptr);

  if (isMapUpdate == true)
  {
    ndt.setInputTarget(map_ptr);
    isMapUpdate = false;
  }

  guess_pose.x = previous_pose_.x + diff_x;
  guess_pose.y = previous_pose_.y + diff_y;
  guess_pose.z = previous_pose_.z + diff_z;
  guess_pose.roll = previous_pose_.roll;
  guess_pose.pitch = previous_pose_.pitch;
  guess_pose.yaw = previous_pose_.yaw + diff_yaw;

  pose guess_pose_for_ndt;
  guess_pose_for_ndt = guess_pose;

  Eigen::AngleAxisf init_rotation_x(guess_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(guess_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(guess_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());

  Eigen::Translation3f init_translation(guess_pose_for_ndt.x, guess_pose_for_ndt.y, guess_pose_for_ndt.z);

  Eigen::Matrix4f init_guess =
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol_;

  t3_end = ros::Time::now();
  d3 = t3_end - t3_start;

  t4_start = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  ndt.align(*output_cloud, init_guess);
  fitness_score = ndt.getFitnessScore();

  t_localizer = ndt.getFinalTransformation();
  t_base_link = t_localizer * tf_ltob_;

  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

  tf::Matrix3x3 mat_l, mat_b;

  mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
                 static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
                 static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                 static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
                 static_cast<double>(t_localizer(2, 2)));

  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));

  // Update localizer_pose.
  localizer_pose.x = t_localizer(0, 3);
  localizer_pose.y = t_localizer(1, 3);
  localizer_pose.z = t_localizer(2, 3);
  mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

  // Update ndt_pose.
  ndt_pose.x = t_base_link(0, 3);
  ndt_pose.y = t_base_link(1, 3);
  ndt_pose.z = t_base_link(2, 3);
  mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

  current_pose_.x = ndt_pose.x;
  current_pose_.y = ndt_pose.y;
  current_pose_.z = ndt_pose.z;
  current_pose_.roll = ndt_pose.roll;
  current_pose_.pitch = ndt_pose.pitch;
  current_pose_.yaw = ndt_pose.yaw;

  transform.setOrigin(tf::Vector3(current_pose_.x, current_pose_.y, current_pose_.z));
  q.setRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link"));

  scan_duration = current_scan_time - previous_scan_time;
  double secs = scan_duration.toSec();

  // Calculate the offset (curren_pos - previous_pos)
  diff_x = current_pose_.x - previous_pose_.x;
  diff_y = current_pose_.y - previous_pose_.y;
  diff_z = current_pose_.z - previous_pose_.z;
  diff_yaw = current_pose_.yaw - previous_pose_.yaw;
  diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

  current_velocity_x = diff_x / secs;
  current_velocity_y = diff_y / secs;
  current_velocity_z = diff_z / secs;

  // Update position and posture. current_pos -> previous_pos
  previous_pose_.x = current_pose_.x;
  previous_pose_.y = current_pose_.y;
  previous_pose_.z = current_pose_.z;
  previous_pose_.roll = current_pose_.roll;
  previous_pose_.pitch = current_pose_.pitch;
  previous_pose_.yaw = current_pose_.yaw;

  previous_scan_time.sec = current_scan_time.sec;
  previous_scan_time.nsec = current_scan_time.nsec;

  // Calculate the shift between added_pos and current_pos
  double shift = sqrt(pow(current_pose_.x - added_pose.x, 2.0) + pow(current_pose_.y - added_pose.y, 2.0));
  if (shift >= min_add_scan_shift_)
  {
    submap_size += shift;
    map += *transformed_scan_ptr;
    submap += *transformed_scan_ptr;
    added_pose.x = current_pose_.x;
    added_pose.y = current_pose_.y;
    added_pose.z = current_pose_.z;
    added_pose.roll = current_pose_.roll;
    added_pose.pitch = current_pose_.pitch;
    added_pose.yaw = current_pose_.yaw;
    isMapUpdate = true;
  }

  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(submap, *map_msg_ptr);
  map_msg_ptr->header.frame_id = "map";
  ndt_map_pub_.publish(*map_msg_ptr);

  q.setRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
  current_pose__msg_.header.frame_id = "map";
  current_pose__msg_.header.stamp = current_scan_time;
  current_pose__msg_.pose.position.x = current_pose_.x;
  current_pose__msg_.pose.position.y = current_pose_.y;
  current_pose__msg_.pose.position.z = current_pose_.z;
  current_pose__msg_.pose.orientation.x = q.x();
  current_pose__msg_.pose.orientation.y = q.y();
  current_pose__msg_.pose.orientation.z = q.z();
  current_pose__msg_.pose.orientation.w = q.w();

  current_pose__pub_.publish(current_pose__msg_);

  if (submap_size >= max_submap_size)
  {
    std::string s1 = "submap_";
    std::string s2 = std::to_string(submap_num);
    std::string s3 = ".pcd";
    std::string pcd_filename = s1 + s2 + s3;

    if (submap.size() != 0)
    {
      if (pcl::io::savePCDFileBinary(pcd_filename, submap) == -1)
      {
        std::cout << "Failed saving " << pcd_filename << "." << std::endl;
      }
      std::cout << "Saved " << pcd_filename << " (" << submap.size() << " points)" << std::endl;

      map = submap;
      submap.clear();
      submap_size = 0.0;
    }
    submap_num++;
  }

  // Write log
  if (!ofs)
  {
    std::cerr << "Could not open " << filename << "." << std::endl;
    exit(1);
  }

  ofs << input->header.seq << ","
      << input->header.stamp << ","
      << input->header.frame_id << ","
      << scan_ptr->size() << ","
      << filtered_scan_ptr->size() << ","
      << std::fixed << std::setprecision(5) << current_pose_.x << ","
      << std::fixed << std::setprecision(5) << current_pose_.y << ","
      << std::fixed << std::setprecision(5) << current_pose_.z << ","
      << current_pose_.roll << ","
      << current_pose_.pitch << ","
      << current_pose_.yaw << ","
      << ndt_res_ << ","
      << step_size_ << ","
      << trans_eps_ << ","
      << max_iter_ << ","
      << voxel_leaf_size_ << ","
      << min_scan_range << ","
      << max_scan_range << ","
      << min_add_scan_shift_ << ","
      << max_submap_size << std::endl;

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << input->header.seq << std::endl;
  std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
  std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
  std::cout << "map: " << map.points.size() << " points." << std::endl;
  std::cout << "NDT has converged: " << ndt.hasConverged() << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Number of iteration: " << ndt.getFinalNumIteration() << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << current_pose_.x << ", " << current_pose_.y << ", " << current_pose_.z << ", " << current_pose_.roll
            << ", " << current_pose_.pitch << ", " << current_pose_.yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_localizer << std::endl;
  std::cout << "shift: " << shift << std::endl;
  std::cout << "current submap size: " << submap_size << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

int main(int argc, char** argv)
{
  previous_pose_.x = 0.0;
  previous_pose_.y = 0.0;
  previous_pose_.z = 0.0;
  previous_pose_.roll = 0.0;
  previous_pose_.pitch = 0.0;
  previous_pose_.yaw = 0.0;

  ndt_pose.x = 0.0;
  ndt_pose.y = 0.0;
  ndt_pose.z = 0.0;
  ndt_pose.roll = 0.0;
  ndt_pose.pitch = 0.0;
  ndt_pose.yaw = 0.0;

  current_pose_.x = 0.0;
  current_pose_.y = 0.0;
  current_pose_.z = 0.0;
  current_pose_.roll = 0.0;
  current_pose_.pitch = 0.0;
  current_pose_.yaw = 0.0;

  guess_pose.x = 0.0;
  guess_pose.y = 0.0;
  guess_pose.z = 0.0;
  guess_pose.roll = 0.0;
  guess_pose.pitch = 0.0;
  guess_pose.yaw = 0.0;

  added_pose.x = 0.0;
  added_pose.y = 0.0;
  added_pose.z = 0.0;
  added_pose.roll = 0.0;
  added_pose.pitch = 0.0;
  added_pose.yaw = 0.0;

  diff_x = 0.0;
  diff_y = 0.0;
  diff_z = 0.0;
  diff_yaw = 0.0;

  ros::init(argc, argv, "approximate_ndt_mapping");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Set log file name.
  char buffer[80];
  std::time_t now = std::time(NULL);
  std::tm* pnow = std::localtime(&now);
  std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
  filename = "approximate_ndt_mapping_" + std::string(buffer) + ".csv";
  ofs.open(filename.c_str(), std::ios::app);

  // write header for log file
  if (!ofs)
  {
    std::cerr << "Could not open " << filename << "." << std::endl;
    exit(1);
  }

  ofs << "input->header.seq" << ","
      << "input->header.stamp" << ","
      << "input->header.frame_id" << ","
      << "scan_ptr->size()" << ","
      << "filtered_scan_ptr->size()" << ","
      << "current_pose_.x" << ","
      << "current_pose_.y" << ","
      << "current_pose_.z" << ","
      << "current_pose_.roll" << ","
      << "current_pose_.pitch" << ","
      << "current_pose_.yaw" << ","
      << "ndt_res_" << ","
      << "step_size_" << ","
      << "trans_eps_" << ","
      << "max_iter_" << ","
      << "voxel_leaf_size_" << ","
      << "min_scan_range" << ","
      << "max_scan_range" << ","
      << "min_add_scan_shift_" << ","
      << "max_submap_size" << std::endl;

  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")" << std::endl;

  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol_ = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  Eigen::Translation3f tl_ltob((-1.0) * _tf_x, (-1.0) * _tf_y, (-1.0) * _tf_z);  // tl: translation
  Eigen::AngleAxisf rot_x_ltob((-1.0) * _tf_roll, Eigen::Vector3f::UnitX());     // rot: rotation
  Eigen::AngleAxisf rot_y_ltob((-1.0) * _tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_ltob((-1.0) * _tf_yaw, Eigen::Vector3f::UnitZ());
  tf_ltob_ = (tl_ltob * rot_z_ltob * rot_y_ltob * rot_x_ltob).matrix();

  map.header.frame_id = "map";

  ndt_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);
  current_pose__pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose_", 1000);

  ros::Subscriber points_sub_ = nh.subscribe("points_raw", 100000, points_callback);

  ros::spin();

  return 0;
}
