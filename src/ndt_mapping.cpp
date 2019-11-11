#include "ndt_mapping.h"

ndt_mapping::ndt_mapping() 
{

  points_sub = nh.subscribe("points_raw", 100000, &ndt_mapping::points_callback,this);
  ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);
  current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);

  // Default values
  max_iter = 30;        // Maximum iterations
  ndt_res = 5.0;      // Resolution
  step_size = 0.1;   // Step size
  trans_eps = 0.01;  // Transformation epsilon

  voxel_leaf_size = 2.0;// Leaf size of VoxelGrid filter.

  initial_scan_loaded = 0;
  min_add_scan_shift = 1.0;

  _tf_x=0.0, _tf_y=0.0, _tf_z=0.0, _tf_roll=0.0, _tf_pitch=0.0, _tf_yaw=0.0;

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

  current_pose.x = current_pose.y = current_pose.z = 0.0;current_pose.roll = current_pose.pitch = current_pose.yaw = 0.0;
  previous_pose.x = previous_pose.y = previous_pose.z = 0.0;previous_pose.roll = previous_pose.pitch = previous_pose.yaw = 0.0;

  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);  

  ndt.setTransformationEpsilon(trans_eps);
  ndt.setStepSize(step_size);
  ndt.setResolution(ndt_res);
  ndt.setMaximumIterations(max_iter);

  is_first_map = true;

}; 

ndt_mapping::~ndt_mapping(){}; 

void ndt_mapping::points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  tf::Quaternion q;

  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  pcl::fromROSMsg(*input, scan);
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  // Add initial point cloud to velodyne_map
  if (initial_scan_loaded == 0)
  {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);
    map += *transformed_scan_ptr;
    initial_scan_loaded = 1;
  }

  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);
  ndt.setInputSource(filtered_scan_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
  if (is_first_map == true){
    ndt.setInputTarget(map_ptr);
    is_first_map = false;
  }

  Eigen::Translation3f init_translation(current_pose.x, current_pose.y, current_pose.z);
  Eigen::AngleAxisf init_rotation_x(current_pose.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(current_pose.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(current_pose.yaw, Eigen::Vector3f::UnitZ());

  Eigen::Matrix4f init_guess =
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol;

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
 
  ndt.align(*output_cloud, init_guess);
  fitness_score = ndt.getFitnessScore();
  t_localizer = ndt.getFinalTransformation();
  has_converged = ndt.hasConverged();
  final_num_iteration = ndt.getFinalNumIteration();
  transformation_probability = ndt.getTransformationProbability();

  t_base_link = t_localizer * tf_ltob;

  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

  tf::Matrix3x3 mat_b;
  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));

  // Update current_pose.
  current_pose.x = t_base_link(0, 3);current_pose.y = t_base_link(1, 3);current_pose.z = t_base_link(2, 3);
  mat_b.getRPY(current_pose.roll, current_pose.pitch, current_pose.yaw, 1);//mat2rpy

  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw); //q from rpy
  transform.setRotation(q);//trans from q

  br.sendTransform(tf::StampedTransform(transform, input->header.stamp, "map", "base_link"));

  double shift = sqrt(pow(current_pose.x - previous_pose.x, 2.0) + pow(current_pose.y - previous_pose.y, 2.0));
  if (shift >= min_add_scan_shift)
  {
    map += *transformed_scan_ptr;
    previous_pose.x = current_pose.x;previous_pose.y = current_pose.y;previous_pose.z = current_pose.z;
    previous_pose.roll = current_pose.roll;previous_pose.pitch = current_pose.pitch;previous_pose.yaw = current_pose.yaw;
    ndt.setInputTarget(map_ptr);
  }

  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  ndt_map_pub.publish(*map_msg_ptr);

  current_pose_msg.header.frame_id = "map";
  current_pose_msg.header.stamp = input->header.stamp;
  current_pose_msg.pose.position.x = current_pose.x;current_pose_msg.pose.position.y = current_pose.y;current_pose_msg.pose.position.z = current_pose.z;
  current_pose_msg.pose.orientation.x = q.x();current_pose_msg.pose.orientation.y = q.y();current_pose_msg.pose.orientation.z = q.z();current_pose_msg.pose.orientation.w = q.w();

  current_pose_pub.publish(current_pose_msg);

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
  std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
            << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_localizer << std::endl;
  std::cout << "shift: " << shift << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}


