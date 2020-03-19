#ifndef ALOAM_VELODYNE_MULTI_LIDAR_CALIB_H
#define ALOAM_VELODYNE_MULTI_LIDAR_CALIB_H

#include <deque>
#include <thread>
#include <mutex>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <ceres/ceres.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "loam_horizon/common.h"
#include "utils/geoutils.h"
#include "logger/logger.h"
#include "transform.h"

namespace mlc {

inline pcl::PointCloud<PointType>::Ptr CreatePointCloudFromRosMessage(const sensor_msgs::PointCloud2ConstPtr &msg) {
  pcl::PointCloud<PointType>::Ptr c(new pcl::PointCloud<PointType>());

  pcl::fromROSMsg(*msg, *c);

  return c;
}

inline pcl::PointCloud<PointType>::Ptr CreatePointCloudFromPointCloud(const pcl::PointCloud<PointType> &c_) {
  pcl::PointCloud<PointType>::Ptr c(new pcl::PointCloud<PointType>());
  *c = c_;

  return c;
}

inline PointType TransformPclPoint(const Eigen::Quaterniond &q, const Eigen::Vector3d &t, const PointType &pi) {
  PointType po;

  Eigen::Vector3d point_curr(pi.x, pi.y, pi.z);
  Eigen::Vector3d point_w = q * point_curr + t;
  po.x = point_w.x();
  po.y = point_w.y();
  po.z = point_w.z();
  po.intensity = pi.intensity;

  return po;
}

void FilterFarawayPoint(const double max_range, pcl::PointCloud<PointType>::Ptr cloud_in);

struct ScanMessages {
  std::deque<sensor_msgs::PointCloud2ConstPtr> full_res_pcls;
  std::deque<sensor_msgs::PointCloud2ConstPtr> corner_pcls;
  std::deque<sensor_msgs::PointCloud2ConstPtr> surf_pcls;
};

struct ScanData {
  pcl::PointCloud<PointType>::Ptr full_res_pcl;
  pcl::PointCloud<PointType>::Ptr corner_pcl;
  pcl::PointCloud<PointType>::Ptr surf_pcl;
};

struct Frame {
  typedef std::shared_ptr<Frame> Ptr;

  template<typename... Args>
  static Ptr Create(Args &&... args) {
    return Ptr(new Frame(std::forward<Args>(args)...));
  };

  ros::Time stamp;
  std::map<int, ScanData> scan;

  Transform::Ptr tf;
};

struct Map {
  typedef std::shared_ptr<Map> Ptr;

  template<typename... Args>
  static Ptr Create(Args &&... args) {
    return Ptr(new Map(std::forward<Args>(args)...));
  };

  ros::Time stamp;
  ScanData scan;
};

struct LidarRig : std::map<int, Transform::Ptr> {
  LidarRig() {};

  Transform::Ptr Get(int lidar_id);
};

class MultiLidarCalib {
 public:
  struct Option {
    double kTimeAlignDelta = 0.049; // In our dataset, usually scans from two LiDAR is less than 50ms
  };

  MultiLidarCalib();

  ~MultiLidarCalib();

  int Init();

  int Clear();

  int PrepareDataFromMsgBuffer();

  int ProcessCalibration();

  int ProcessFullCalibration();

  int ProcessIncreamentalCalibration();

  int PublishResultVisualization();

  int WriteResultToDisk();

  int SetMap(const ros::Time &stamp,
             const pcl::PointCloud<PointType> &full_res_pcl,
             const pcl::PointCloud<PointType> &corner_pcl,
             const pcl::PointCloud<PointType> &surf_pcl);

  int FilterNoisyPointCloud(pcl::PointCloud<PointType>::Ptr cloud_in);

  int DownsampleByOctree(const double leafsize,
                         const pcl::PointCloud<PointType>::Ptr cloud_in,
                         pcl::PointCloud<PointType>::Ptr& cloud_out);

  std::unique_ptr<ros::NodeHandle> nh_;

  pcl::VoxelGrid<PointType> downSizeFilterCorner_;
  pcl::VoxelGrid<PointType> downSizeFilterSurf_;

  std::vector<int> kLidarIdList;
  std::vector<int> kAuxLidarIdList;
  Option opt_;

  std::deque<nav_msgs::OdometryConstPtr> odoms_;
  std::map<int, ScanMessages> scan_msgs_;
  std::mutex msgs_mutex_;

  std::deque<Frame::Ptr> frames_;
  Map::Ptr map_;
  std::mutex map_mutex_;

  LidarRig lidar_rig_;

  std::map<int, ros::Publisher> registered_pub_;
  ros::Publisher map_pub_;

  bool optimize_translation_;
  float lineRes_;
  float planeRes_;

 private:

};

}

#endif //ALOAM_VELODYNE_MULTI_LIDAR_CALIB_H
