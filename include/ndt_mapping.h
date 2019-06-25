#define OUTPUT  // If you want to output "position_log.txt", "#define OUTPUT".

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

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
  
  ros::NodeHandle nh;
  ros::Subscriber points_sub;

  struct pose{double x,y,z;double roll,pitch,yaw;};
  struct pose previous_pose; 
  struct pose guess_pose;
  struct pose current_pose;
  struct pose ndt_pose;
  struct pose added_pose;
  struct pose localizer_pose;

  ros::Time current_scan_time,previous_scan_time;
  ros::Duration scan_duration;

  double diff , diff_x, diff_y , diff_z , diff_yaw;  // current_pose - previous_pose

  double current_velocity_x , current_velocity_y , current_velocity_z ;
  

  pcl::PointCloud<pcl::PointXYZI> map;

  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

  // Default values
  int max_iter ;        // Maximum iterations
  //static float ndt_res = 1.0;      // Resolution
  float ndt_res ;      // Resolution
  double step_size ;   // Step size
  double trans_eps ;  // Transformation epsilon

  // Leaf size of VoxelGrid filter.
  double voxel_leaf_size;

  ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, t4_start, t4_end,
    t5_start, t5_end;
  ros::Duration d_callback, d1, d2, d3, d4, d5;

  ros::Publisher ndt_map_pub, current_pose_pub,guess_pose_linaer_pub;
  geometry_msgs::PoseStamped current_pose_msg, guess_pose_msg;

  ros::Publisher ndt_stat_pub;
  std_msgs::Bool ndt_stat_msg;

  int initial_scan_loaded;

  Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();

  double min_scan_range;
  //static double min_scan_range = 1.0;
  double max_scan_range;
  double min_add_scan_shift;
  //static double min_add_scan_shift = 0.5;

  double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
  Eigen::Matrix4f tf_btol, tf_ltob;//base_link2localizer等の略?


  bool _incremental_voxel_update;

  double fitness_score;
  bool has_converged;
  int final_num_iteration;
  double transformation_probability;


  std::ofstream ofs;
  std::string filename;

  void timerCallback(const ros::TimerEvent &e);
  
  double calcDiffForRadian(const double lhs_rad, const double rhs_rad);


  void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input);

  void publishMapAndPose();
};
