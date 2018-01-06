#ifndef APRILSLAM_MAPPER_TWO_IMAGE_NODE_H_
#define APRILSLAM_MAPPER_TWO_IMAGE_NODE_H_

#include <ros/ros.h>
#include <aprilslam/Apriltags.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include "aprilslam/visualizer.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include "aprilslam/mapper.h"
#include "aprilslam/tag_map.h"

namespace aprilslam {

class MapperTwoImageNode {
 public:
  MapperTwoImageNode(const ros::NodeHandle& nh, const std::string& frame_id,
		  const std::string& new_frame_id,const std::string& parent_frame_id)
      : nh_(nh),
        sub_tags_(nh_.subscribe("apriltags", 1, &MapperTwoImageNode::TagsCb, this)),
        sub_other_tags_(nh_.subscribe("other_apriltags", 1, &MapperTwoImageNode::TagsCb2, this)),
        sub_cinfo_(nh_.subscribe("camera_info", 1, &MapperTwoImageNode::CinfoCb, this)),
		sub_other_cinfo_(nh_.subscribe("other_camera_info", 1, &MapperTwoImageNode::CinfoCb2, this)),
		pub_dx_dy_(nh_.advertise<geometry_msgs::Vector3>("image_dx_dy", 1)),
		frame_id_(frame_id),
		new_frame_id_(new_frame_id),
		parent_frame_id_(parent_frame_id),
        mapper_(0.04, 1),
        tag_viz_(nh, "apriltags_map") {
    		tag_viz_.SetColor(aprilslam::GREEN);
    		tag_viz_.SetAlpha(0.75);
  	}

  bool GetGoodTags(const std::vector<aprilslam::Apriltag> tags_c,
                   std::vector<aprilslam::Apriltag>* tags_c_good);
  bool GetGoodTags2(const std::vector<aprilslam::Apriltag> tags_c,
                   std::vector<aprilslam::Apriltag>* tags_c_good);

 private:
  //这个函数估计自己的位置
  void TagsCb(const aprilslam::ApriltagsConstPtr& tags_c_msg);
  //这个函数得到两个图像之后回调函数,通过2d-2d来估计位置.
  void TagsCb2(const aprilslam::ApriltagsConstPtr& tags_c_msg);
  void CinfoCb(const sensor_msgs::CameraInfoConstPtr& cinfo_msg);
  void CinfoCb2(const sensor_msgs::CameraInfoConstPtr& cinfo_msg);

  void quaternion2tf(const geometry_msgs::Quaternion,tf::Quaternion&);
  ros::NodeHandle nh_;
  ros::Subscriber sub_tags_;
  ros::Subscriber sub_other_tags_;
  ros::Subscriber sub_cinfo_;
  ros::Subscriber sub_other_cinfo_;

  ros::Publisher pub_dx_dy_;

  std::string frame_id_;
  std::string new_frame_id_;
  std::string parent_frame_id_;

  aprilslam::TagMap map_;
  aprilslam::TagMap map2_;
  aprilslam::Mapper mapper_;

  std_msgs::Header header_;
  std::vector<Apriltag> tags_;
  std::vector<Apriltag> other_tags_;
  aprilslam::ApriltagVisualizer tag_viz_;
  image_geometry::PinholeCameraModel model_;
  image_geometry::PinholeCameraModel model2_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

};

}  // namespace aprilslam

#endif  // APRILSLAM_MAPPER_TWO_IMAGE_NODE_H_
