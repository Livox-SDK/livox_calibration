#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver/CustomMsg.h"
#include "loam_horizon/common.h"

ros::Publisher pub_pcl_out1, pub_pcl_out2;
uint64_t TO_MERGE_CNT = 2;
constexpr bool b_dbg_line = false;

std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;

void LivoxMsgCbk1(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {
  livox_data.push_back(livox_msg_in);
  if (livox_data.size() < TO_MERGE_CNT) return;

  pcl::PointCloud<PointType> pcl_in;

  for (size_t j = 0; j < livox_data.size(); j++) {
    double dt = (livox_data[j]->timebase - livox_data[0]->timebase) * 1e-9;

    auto livox_msg = livox_data[j];
    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      PointType pt;
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;
      if (pt.z < -0.3) continue;
      uint32_t offset_time_ns = livox_msg->points[i].offset_time;
      double offset_time_s = dt + offset_time_ns * 1e-9;
      // ROS_INFO("offset_time_s-------- %.3f ",offset_time_s);
      pt.intensity = livox_msg->points[i].line + offset_time_s;
      pt.curvature = livox_msg->points[i].reflectivity * 0.1;
      // ROS_INFO("pt.curvature-------- %.3f ",pt.curvature);
      pcl_in.push_back(pt);
    }
  }

  /// timebase 5ms ~ 50000000, so 10 ~ 1ns

  unsigned long timebase_ns = livox_data[0]->timebase;
  ros::Time timestamp;
  timestamp.fromNSec(timebase_ns);

  //   ROS_INFO("livox1 republish %u points at time %f buf size %ld",
  //   pcl_in.size(),
  //           timestamp.toSec(), livox_data.size());

  sensor_msgs::PointCloud2 pcl_ros_msg;
  pcl::toROSMsg(pcl_in, pcl_ros_msg);
  pcl_ros_msg.header.stamp.fromNSec(timebase_ns);
  pcl_ros_msg.header.frame_id = "/livox";
  pub_pcl_out1.publish(pcl_ros_msg);
  livox_data.clear();
}


std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data2;
void LivoxMsgCbk2(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {
  livox_data2.push_back(livox_msg_in);
  if (livox_data2.size() < TO_MERGE_CNT) return;

  pcl::PointCloud<PointType> pcl_in;

  for (size_t j = 0; j < livox_data2.size(); j++) {
    double dt = (livox_data2[j]->timebase - livox_data2[0]->timebase) * 1e-9;

    auto livox_msg = livox_data2[j];
    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      PointType pt;
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;
      if (pt.z < -0.3) continue;
      uint32_t offset_time_ns = livox_msg->points[i].offset_time;
      double offset_time_s = dt + offset_time_ns * 1e-9;
      // ROS_INFO("offset_time_s-------- %.3f ",offset_time_s);
      pt.intensity = livox_msg->points[i].line + offset_time_s;
      pt.curvature = livox_msg->points[i].reflectivity * 0.1;
      // ROS_INFO("pt.curvature-------- %.3f ",pt.curvature);
      pcl_in.push_back(pt);
    }
  }

  /// timebase 5ms ~ 50000000, so 10 ~ 1ns

  unsigned long timebase_ns = livox_data2[0]->timebase;
  ros::Time timestamp;
  timestamp.fromNSec(timebase_ns);

  //   ROS_INFO("livox1 republish %u points at time %f buf size %ld",
  //   pcl_in.size(),
  //           timestamp.toSec(), livox_data2.size());

  sensor_msgs::PointCloud2 pcl_ros_msg;
  pcl::toROSMsg(pcl_in, pcl_ros_msg);
  pcl_ros_msg.header.stamp.fromNSec(timebase_ns);
  pcl_ros_msg.header.frame_id = "/livox";
  pub_pcl_out2.publish(pcl_ros_msg);
  livox_data2.clear();
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub");
  ros::NodeHandle nh;

  ROS_INFO("start livox_repub");

  ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver::CustomMsg>(
      "/livox/lidar0", 100, LivoxMsgCbk1);
  pub_pcl_out1 = nh.advertise<sensor_msgs::PointCloud2>("/livox_pcl0", 100);

  ros::Subscriber sub_livox_msg2 = nh.subscribe<livox_ros_driver::CustomMsg>(
      "/livox/lidar1", 100, LivoxMsgCbk2);
  pub_pcl_out2 = nh.advertise<sensor_msgs::PointCloud2>("/livox_pcl1", 100);

  ros::spin();
}
