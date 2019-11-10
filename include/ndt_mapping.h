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
  struct pose current_pose;
  struct pose previous_pose;

  pcl::PointCloud<pcl::PointXYZI> map;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

  // Default values
  int max_iter ;        // Maximum iterations
  float ndt_res ;      // Resolution
  double step_size ;   // Step size
  double trans_eps ;  // Transformation epsilon

  double voxel_leaf_size;// Leaf size of VoxelGrid filter.

  ros::Publisher ndt_map_pub, current_pose_pub;
  geometry_msgs::PoseStamped current_pose_msg;

  int initial_scan_loaded;
  double min_add_scan_shift;

  double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
  Eigen::Matrix4f tf_btol, tf_ltob;//base_link2localizer等の略?

  bool _incremental_voxel_update;

  double fitness_score;
  bool has_converged;
  int final_num_iteration;
  double transformation_probability;
  
  bool is_first_map;

  std::ofstream ofs;
  std::string filename;

  void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input);
};
