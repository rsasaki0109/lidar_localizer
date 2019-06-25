#include "ndt_mapping.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ndt_mapping");
  ndt_mapping ndt;

  ros::spin();

  return 0;
};

