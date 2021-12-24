#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher points_map_pub;

std::string map_path;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_loader");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("map_path", map_path);

  // Publishers
  points_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_map", 10, true);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (map_path, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  sensor_msgs::PointCloud2 map_out;
  pcl::toROSMsg(*cloud, map_out);
  map_out.header.frame_id = "/map";
  points_map_pub.publish (map_out);
  std::cout << "map loaded!!" << std::endl;

  ros::spin();

  return 0;
}
