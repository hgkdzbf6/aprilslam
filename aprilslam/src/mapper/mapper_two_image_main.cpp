#include "aprilslam/mapper_two_image_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh;

  try {
    aprilslam::MapperTwoImageNode mapper_node(nh, "world","other_frame_id");
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
