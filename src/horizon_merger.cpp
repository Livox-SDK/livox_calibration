#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <string>
#include <vector>
#include "loam_horizon/common.h"
#include "loam_horizon/tic_toc.h"

int TO_MERGE_CNT = 10;
int merge_cnt = 0;
pcl::PointCloud<pcl::PointXYZI> pcl_out;
ros::Publisher pub_pcl_out;

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
  pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

  size_t sz_pre = pcl_out.size();
  pcl_out += laserCloudIn;
  size_t sz_cur = pcl_out.size();
  /// Add time_offset (intensity = ring + offset_time_s)
  if (TO_MERGE_CNT == 10) {
    for (size_t i = sz_pre; i < sz_cur; ++i) {
      /// 100Hz merged to 10Hz, 1609 data
      /// Intensity = ring + intensity * 0.001
      int ring = (int)pcl_out[i].intensity;
      pcl_out[i].intensity = ring + merge_cnt * 0.1 / TO_MERGE_CNT;
    }
  } else {
    for (size_t i = sz_pre; i < sz_cur; ++i) {
      /// 20Hz merged to 10Hz, 1615 data.
      /// Intensity = ring + offset_s_pkg
      pcl_out[i].intensity += merge_cnt * 0.1 / TO_MERGE_CNT;
    }
  }
  merge_cnt++;

  if (merge_cnt >= TO_MERGE_CNT) {
    /// Publish
    sensor_msgs::PointCloud2 pcl_out_msg;
    pcl::toROSMsg(pcl_out, pcl_out_msg);
    pcl_out_msg.header = laserCloudMsg->header;
    pcl_out_msg.header.frame_id = "/camera_init";
    pub_pcl_out.publish(pcl_out_msg);

    /// clear
    merge_cnt = 0;
    pcl_out.clear();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "horizon_merger");
  ros::NodeHandle nh("~");

  if (!nh.getParam("to_merge_cnt", TO_MERGE_CNT)) {
    ROS_WARN("get to_merge_cnt failed [%d], set to 10", TO_MERGE_CNT);
    TO_MERGE_CNT = 10;
  }
  ROS_INFO("to_merge_cnt: %d", TO_MERGE_CNT);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
      "/horizon_topic", 100, laserCloudHandler);

  pub_pcl_out = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 100);

  ros::spin();
}
