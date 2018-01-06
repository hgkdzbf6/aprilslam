#include "aprilslam/mapper_two_image_node.h"
#include "aprilslam/utils.h"

namespace aprilslam {

void MapperTwoImageNode::TagsCb(const aprilslam::ApriltagsConstPtr& tags_c_msg) {

  // Do nothing if no detection, this prevents checking in the following steps
  if (tags_c_msg->apriltags.empty()) {
    ROS_WARN_THROTTLE(1, "No tags detected.");
    return;
  }
  // Do nothing if camera info not received
  if (!model_.initialized()) {
    ROS_WARN_THROTTLE(1, "No camera info received");
    return;
  }
  // Do nothing if there are no good tags close to the center of the image
  std::vector<Apriltag> tags_c_good;
  if (!GetGoodTags(tags_c_msg->apriltags, &tags_c_good)) {
    ROS_WARN_THROTTLE(1, "No good tags detected.");
    return;
  }
  // Initialize map by adding the first tag that is not on the edge of the image
  if (!map_.init()) {
    map_.AddFirstTag(tags_c_good.front());
    ROS_INFO("AprilMap initialized.");
  }
  // Do nothing if no pose can be estimated
  geometry_msgs::Pose pose;
  if (!map_.EstimatePose(tags_c_msg->apriltags, model_.fullIntrinsicMatrix(),
                         model_.distortionCoeffs(), &pose)) {
    ROS_WARN_THROTTLE(1, "No 2D-3D correspondence.");
    return;
  }

  // 开始写光流法吧
  //  第一步:拷贝上次图像的结果.默认已经做了
  // 第二步:分析所有图像上的点,得到所有的点对坐标.
  // 首先查看id号
  int i,j,k,count,current_count,this_count,match_count;
  const std::vector<aprilslam::Apriltag>& current_tags=tags_c_msg->apriltags;
  std::vector<geometry_msgs::Vector3> match;
  //两套循环,可能带来效率问题
  count=0;
  current_count=current_tags.size();
  this_count=this->tags_.size();
  for (i=0;i<current_count;i++){
	  for(j=0;j<this_count;j++){
		  //现在要考虑的是匹配点的储存格式问题
		  //定义一个结构体?
		  if(current_tags[i].id==this->tags_[j].id){
			  for(k=0;k<4;k++){
				  geometry_msgs::Vector3 v;
				  v.x=current_tags[i].corners[k].x-tags_[j].corners[k].x;
				  v.y=current_tags[i].corners[k].y-tags_[j].corners[k].y;
				  match.push_back(v);
			  }
			  count++;
			  break;
		  }
	  }
	  //8个点足够了?
	  if(count>2)
		  break;
  }
  match_count=match.size();
  double dx=0,dy=0;
//  第三步,找到了在像素平面上的点的匹配,然后就找像素的偏移.因为就是默认在一个平面上,所以就均值一下就好了?
  for(i=0;i<match_count;i++){
	  dx+=match[i].x;
	  dy+=match[i].y;
  }
  dx=dx/match_count;
  dy=dy/match_count;
  //需要转化成实际坐标吗?还是在控制里面调整?
  //先后者吧
//  第四步:广播
  geometry_msgs::Vector3 vv;
  vv.x=dx;
  vv.y=dy;
  pub_dx_dy_.publish(vv);
  //最后一步:更新上次图像的结果
  this->tags_=std::vector<aprilslam::Apriltag>(tags_c_msg->apriltags);

  // // Now that with the initial pose calculated, we can do some mapping
  // mapper_.AddPose(pose);
  // mapper_.AddFactors(tags_c_good);
  // if (mapper_.init()) {
  //   // This will only add new landmarks
  //   mapper_.AddLandmarks(tags_c_good);
  //   mapper_.Optimize();
  //   // Get latest estimates from mapper and put into map
  //   mapper_.Update(&map_, &pose);
  //   // Prepare for next iteration
  //   mapper_.Clear();
  // } else {
  //   // This will add first landmark at origin and fix scale for first pose and
  //   // first landmark
  //   mapper_.Initialize(map_.first_tag());
  // }

  // Publish camera to world transform
  std_msgs::Header header;
  header.stamp = tags_c_msg->header.stamp;
  header.frame_id = frame_id_;
  this->header_=header;

  geometry_msgs::Vector3 translation;
  translation.x = pose.position.x;
  translation.y = pose.position.y;
  translation.z = pose.position.z;

  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header = header;
  transform_stamped.child_frame_id = tags_c_msg->header.frame_id;
  transform_stamped.transform.translation = translation;
  transform_stamped.transform.rotation = pose.orientation;

  tf_broadcaster_.sendTransform(transform_stamped);

  // Publish visualisation markers
  tag_viz_.PublishApriltagsMarker(map_.tags_w(), frame_id_,
                                  tags_c_msg->header.stamp);
}

void MapperTwoImageNode::TagsCb2(const aprilslam::ApriltagsConstPtr& tags_c_msg) {
	//这一步就是想知道相对位置.
	//方法:先遍历一遍和出现符号相同的符号.
	//先看看有没有好的标签
	static tf::TransformBroadcaster br;

	// Do nothing if no detection, this prevents checking in the following steps
	if (tags_c_msg->apriltags.empty()) {
		ROS_WARN_THROTTLE(1, "No tags detected.");
		return;
	}
	// Do nothing if camera info not received
	if (!model2_.initialized()) {
		ROS_WARN_THROTTLE(1, "No camera info received");
		return;
	}
	// Do nothing if there are no good tags close to the center of the image
	std::vector<Apriltag> tags_c_good;
	if (!GetGoodTags(tags_c_msg->apriltags, &tags_c_good)) {
		ROS_WARN_THROTTLE(1, "other no good tags detected.");
		return;
	}
	//不进行位置的估计了
	//这里广播一个位置,以downward_cam作为参考系
	//先进行apriltag的复制
	this->other_tags_=std::vector<aprilslam::Apriltag>(tags_c_msg->apriltags);
	unsigned int i,j;
	geometry_msgs::Pose pose1,pose2;
	tf::Transform transform;
	for(i=0;i<this->tags_.size();i++){
		for(j=0;j<this->other_tags_.size();j++){
			if(tags_[i].id==other_tags_[j].id){
				pose1=tags_[i].pose;
				pose2=other_tags_[j].pose;
				tf::Quaternion q1,q2;
				quaternion2tf(pose1.orientation,q1);
				quaternion2tf(pose2.orientation,q2);
				tf::Quaternion q=q1.inverse();
				q*=q2;
				transform.setOrigin(tf::Vector3(pose1.position.x-pose2.position.x,
						pose1.position.y-pose2.position.y,
						pose1.position.z-pose2.position.z));
				transform.setRotation(q);
				break;
			}
		}
	}
	br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),parent_frame_id_,new_frame_id_));
}

void MapperTwoImageNode::quaternion2tf(const geometry_msgs::Quaternion q,tf::Quaternion& qq){
	qq.setValue(q.x,q.y,q.z,q.w);
}

void MapperTwoImageNode::CinfoCb(const sensor_msgs::CameraInfoConstPtr& cinfo_msg) {
  if (model_.initialized()) {
    sub_cinfo_.shutdown();
    ROS_INFO("%s: %s", nh_.getNamespace().c_str(), "Camera initialized");
    return;
  }
  model_.fromCameraInfo(cinfo_msg);
}

void MapperTwoImageNode::CinfoCb2(const sensor_msgs::CameraInfoConstPtr& cinfo_msg) {
  if (model2_.initialized()) {
    sub_other_cinfo_.shutdown();
    ROS_INFO("%s: %s", nh_.getNamespace().c_str(), "Camera initialized");
    return;
  }
  model2_.fromCameraInfo(cinfo_msg);
}

bool MapperTwoImageNode::GetGoodTags(const std::vector<Apriltag> tags_c,
                             std::vector<Apriltag>* tags_c_good) {
  for (const Apriltag& tag_c : tags_c) {
    if (IsInsideImageCenter(tag_c.center.x, tag_c.center.y,
                            model_.cameraInfo().width,
                            model_.cameraInfo().height, 5)) {
      tags_c_good->push_back(tag_c);
    }
  }
  return !tags_c_good->empty();
}

bool MapperTwoImageNode::GetGoodTags2(const std::vector<Apriltag> tags_c,
                             std::vector<Apriltag>* tags_c_good) {
  for (const Apriltag& tag_c : tags_c) {
    if (IsInsideImageCenter(tag_c.center.x, tag_c.center.y,
                            model2_.cameraInfo().width,
                            model2_.cameraInfo().height, 5)) {
      tags_c_good->push_back(tag_c);
    }
  }
  return !tags_c_good->empty();
}

}  // namespace aprilslam
