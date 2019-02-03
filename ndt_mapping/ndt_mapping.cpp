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

#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/registration/ndt.h>

#include <time.h>

struct pose{double x,y,z;double roll,pitch,yaw;};

// global variables
static pose previous_pose, guess_pose, current_pose, ndt_pose, added_pose, localizer_pose;

static ros::Time current_scan_time,previous_scan_time;
static ros::Duration scan_duration;

static double diff = 0.0, diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw;  // current_pose - previous_pose

static double current_velocity_x = 0.0, current_velocity_y = 0.0, current_velocity_z = 0.0;

static pcl::PointCloud<pcl::PointXYZI> map;

static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
static cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> anh_ndt;

// Default values
static int max_iter = 30;        // Maximum iterations
static float ndt_res = 1.0;      // Resolution
static double step_size = 0.1;   // Step size
static double trans_eps = 0.01;  // Transformation epsilon

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 2.0;

static ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, t4_start, t4_end,
    t5_start, t5_end;
static ros::Duration d_callback, d1, d2, d3, d4, d5;

static ros::Publisher ndt_map_pub, current_pose_pub,guess_pose_linaer_pub;
static geometry_msgs::PoseStamped current_pose_msg, guess_pose_msg;

static ros::Publisher ndt_stat_pub;
static std_msgs::Bool ndt_stat_msg;

static int initial_scan_loaded = 0;

static Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();

static double min_scan_range = 5.0;
static double max_scan_range = 200.0;
static double min_add_scan_shift = 1.0;

static double _tf_x=0.0, _tf_y=0.0, _tf_z=0.0, _tf_roll=0.0, _tf_pitch=0.0, _tf_yaw=0.0;
static Eigen::Matrix4f tf_btol, tf_ltob;//base_link2localizer等の略?


static bool _incremental_voxel_update = false;

static double fitness_score;
static bool has_converged;
static int final_num_iteration;
static double transformation_probability;


static std::ofstream ofs;
static std::string filename;

static double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if      (diff_rad >= M_PI) diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI) diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}


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
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  current_scan_time = input->header.stamp;

  pcl::fromROSMsg(*input, tmp);

  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
  {
    p.x = (double)item->x; p.y = (double)item->y; p.z = (double)item->z;
    p.intensity = (double)item->intensity;

    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (min_scan_range < r && r < max_scan_range) scan.push_back(p);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  // Add initial point cloud to velodyne_map
  if (initial_scan_loaded == 0)
  {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);
    map += *transformed_scan_ptr;
    initial_scan_loaded = 1;
  }

  // Apply voxelgrid filter
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));

  ndt.setTransformationEpsilon(trans_eps);
  ndt.setStepSize(step_size);
  ndt.setResolution(ndt_res);
  ndt.setMaximumIterations(max_iter);
  ndt.setInputSource(filtered_scan_ptr);

  static bool is_first_map = true;
  if (is_first_map == true){
    ndt.setInputTarget(map_ptr);
    is_first_map = false;
  }

  guess_pose.x = previous_pose.x + diff_x; guess_pose.y = previous_pose.y + diff_y; guess_pose.z = previous_pose.z + diff_z;
  guess_pose.roll = previous_pose.roll; guess_pose.pitch = previous_pose.pitch; guess_pose.yaw = previous_pose.yaw + diff_yaw;

  pose guess_pose_for_ndt;
  guess_pose_for_ndt = guess_pose;

  Eigen::AngleAxisf init_rotation_x(guess_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(guess_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(guess_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());

  Eigen::Translation3f init_translation(guess_pose_for_ndt.x, guess_pose_for_ndt.y, guess_pose_for_ndt.z);

  Eigen::Matrix4f init_guess =
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol;

  t3_end = ros::Time::now();
  d3 = t3_end - t3_start;

  t4_start = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

 
  ndt.align(*output_cloud, init_guess);
  fitness_score = ndt.getFitnessScore();
  t_localizer = ndt.getFinalTransformation();
  has_converged = ndt.hasConverged();
  final_num_iteration = ndt.getFinalNumIteration();
  transformation_probability = ndt.getTransformationProbability();

  t_base_link = t_localizer * tf_ltob;

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
  localizer_pose.x = t_localizer(0, 3);localizer_pose.y = t_localizer(1, 3);localizer_pose.z = t_localizer(2, 3);
  mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

  // Update ndt_pose.
  ndt_pose.x = t_base_link(0, 3);ndt_pose.y = t_base_link(1, 3);ndt_pose.z = t_base_link(2, 3);
  mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

  current_pose.x = ndt_pose.x;current_pose.y = ndt_pose.y;current_pose.z = ndt_pose.z;
  current_pose.roll = ndt_pose.roll;current_pose.pitch = ndt_pose.pitch;current_pose.yaw = ndt_pose.yaw;

  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link"));

  scan_duration = current_scan_time - previous_scan_time;
  double secs = scan_duration.toSec();

  // Calculate the offset (curren_pos - previous_pos)
  diff_x = current_pose.x - previous_pose.x;diff_y = current_pose.y - previous_pose.y;diff_z = current_pose.z - previous_pose.z;
  diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
  diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

  current_velocity_x = diff_x / secs; current_velocity_y = diff_y / secs; current_velocity_z = diff_z / secs;

  // Update position and posture. current_pos -> previous_pos
  previous_pose.x = current_pose.x;previous_pose.y = current_pose.y;previous_pose.z = current_pose.z;
  previous_pose.roll = current_pose.roll;previous_pose.pitch = current_pose.pitch;previous_pose.yaw = current_pose.yaw;

  previous_scan_time.sec = current_scan_time.sec; previous_scan_time.nsec = current_scan_time.nsec;

  // Calculate the shift between added_pos and current_pos
  double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
  
  if (shift >= min_add_scan_shift)
  {
    map += *transformed_scan_ptr;
    added_pose.x = current_pose.x;added_pose.y = current_pose.y;added_pose.z = current_pose.z;added_pose.roll = current_pose.roll;added_pose.pitch = current_pose.pitch;added_pose.yaw = current_pose.yaw;

    ndt.setInputTarget(map_ptr);
  }

  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  ndt_map_pub.publish(*map_msg_ptr);

  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  current_pose_msg.header.frame_id = "map";
  current_pose_msg.header.stamp = current_scan_time;
  current_pose_msg.pose.position.x = current_pose.x;current_pose_msg.pose.position.y = current_pose.y;current_pose_msg.pose.position.z = current_pose.z;
  current_pose_msg.pose.orientation.x = q.x();current_pose_msg.pose.orientation.y = q.y();current_pose_msg.pose.orientation.z = q.z();current_pose_msg.pose.orientation.w = q.w();

  current_pose_pub.publish(current_pose_msg);

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
      << std::fixed << std::setprecision(5) << current_pose.x << ","
      << std::fixed << std::setprecision(5) << current_pose.y << ","
      << std::fixed << std::setprecision(5) << current_pose.z << ","
      << current_pose.roll << ","
      << current_pose.pitch << ","
      << current_pose.yaw << ","
      << final_num_iteration << ","
      << fitness_score << ","
      << ndt_res << ","
      << step_size << ","
      << trans_eps << ","
      << max_iter << ","
      << voxel_leaf_size << ","
      << min_scan_range << ","
      << max_scan_range << ","
      << min_add_scan_shift << std::endl;

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << input->header.seq << std::endl;
  std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
  std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
  std::cout << "map: " << map.points.size() << " points." << std::endl;
  std::cout << "NDT has converged: " << has_converged << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Number of iteration: " << final_num_iteration << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
            << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_localizer << std::endl;
  std::cout << "shift: " << shift << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

int main(int argc, char** argv)
{
  previous_pose.x = 0.0;previous_pose.y = 0.0;previous_pose.z = 0.0;previous_pose.roll = 0.0;previous_pose.pitch = 0.0;previous_pose.yaw = 0.0;

  ndt_pose.x = 0.0;ndt_pose.y = 0.0;ndt_pose.z = 0.0;ndt_pose.roll = 0.0;ndt_pose.pitch = 0.0;ndt_pose.yaw = 0.0;

  current_pose.x = 0.0;current_pose.y = 0.0;current_pose.z = 0.0;current_pose.roll = 0.0;current_pose.pitch = 0.0;current_pose.yaw = 0.0;

  guess_pose.x = 0.0;guess_pose.y = 0.0;guess_pose.z = 0.0;guess_pose.roll = 0.0;guess_pose.pitch = 0.0;guess_pose.yaw = 0.0;

  added_pose.x = 0.0;added_pose.y = 0.0;added_pose.z = 0.0;added_pose.roll = 0.0;added_pose.pitch = 0.0;added_pose.yaw = 0.0;

  diff_x = 0.0;diff_y = 0.0;diff_z = 0.0;diff_yaw = 0.0;

  ros::init(argc, argv, "ndt_mapping");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Set log file name.
  char buffer[80];
  std::time_t now = std::time(NULL);
  std::tm* pnow = std::localtime(&now);
  std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
  filename = "ndt_mapping_" + std::string(buffer) + ".csv";
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
      << "current_pose.x" << ","
      << "current_pose.y" << ","
      << "current_pose.z" << ","
      << "current_pose.roll" << ","
      << "current_pose.pitch" << ","
      << "current_pose.yaw" << ","
      << "final_num_iteration" << ","
      << "fitness_score" << ","
      << "ndt_res" << ","
      << "step_size" << ","
      << "trans_eps" << ","
      << "max_iter" << ","
      << "voxel_leaf_size" << ","
      << "min_scan_range" << ","
      << "max_scan_range" << ","
      << "min_add_scan_shift" << std::endl;

  // setting parameters

  std::cout << "incremental_voxel_update: " << _incremental_voxel_update << std::endl;


  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")" << std::endl;


  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
  tf_ltob = tf_btol.inverse();

  map.header.frame_id = "map";

  ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);
  current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);

  ros::Subscriber points_sub = nh.subscribe("points_raw", 100000, points_callback);


  ros::spin();

  return 0;
}
