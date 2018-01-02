#include "aprilslam/mapper_two_image_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  std::string new_frame_id;
  std::string parent_frame_id;
  n.getParam("new_frame_id",new_frame_id);
  n.getParam("parent_frame_id",parent_frame_id);
  try {
    aprilslam::MapperTwoImageNode mapper_node(nh, "world",new_frame_id,parent_frame_id);
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
