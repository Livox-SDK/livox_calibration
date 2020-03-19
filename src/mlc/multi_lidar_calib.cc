#include "multi_lidar_calib.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "lidarCalibFactor.hpp"

namespace mlc {

void FilterFarawayPoint(const double max_range, pcl::PointCloud<PointType>::Ptr cloud_in) {
  pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>());

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<PointType> extract;

  for (size_t i = 0; i < cloud_in->points.size(); ++i) {
    double dist = Eigen::Vector3d(cloud_in->points.at(i).x, cloud_in->points.at(i).y, cloud_in->points.at(i).z).norm();
    if (dist < max_range) {
      inliers->indices.push_back(i);
    }
  }

  extract.setInputCloud(cloud_in);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_out);

  cloud_in.swap(cloud_out);
}

Transform::Ptr LidarRig::Get(int lidar_id) {
  if (this->find(lidar_id) == this->end()) {
    return Transform::Ptr();
  }

  return this->at(lidar_id);
}

MultiLidarCalib::MultiLidarCalib() {
  kAuxLidarIdList = {1};
//  kAuxLidarIdList = {};
  kLidarIdList = {0};
  kLidarIdList.insert(kLidarIdList.end(), kAuxLidarIdList.begin(), kAuxLidarIdList.end());
}

MultiLidarCalib::~MultiLidarCalib() {

}

int MultiLidarCalib::Init() {

  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());

  for (auto lidar_id : kLidarIdList) {
    scan_msgs_.emplace(lidar_id, ScanMessages());
  }

  for (auto lidar_id : kLidarIdList) {
    lidar_rig_.emplace(lidar_id, Transform::Create(string_format("lidar_%d", 0), string_format("lidar_%d", lidar_id)));
  }

  float lineRes = 0.2;
  float planeRes = 0.2;
//  nh_->param<float>("mapping_line_resolution", lineRes, 0.4);
//  nh_->param<float>("mapping_plane_resolution", planeRes, 0.8);
  lineRes_ = lineRes;
  planeRes_ = planeRes;
  ROS_INFO("[calib] line resolution %.3f plane resolution %.3f", lineRes, planeRes);

  map_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("/mlcalib/map", 100);
  for (auto lidar_id : kLidarIdList) {
    registered_pub_.emplace(lidar_id,
                            nh_->advertise<sensor_msgs::PointCloud2>(string_format("/mlcalib/lidar%d", lidar_id), 100));
  }

  downSizeFilterCorner_.setLeafSize(lineRes, lineRes, lineRes);
  downSizeFilterSurf_.setLeafSize(planeRes, planeRes, planeRes);

  // Set initial parameters
  {
    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();

    std::vector<double> t, ypr;
    bool succ_t = nh_->getParam("/mlcalib/initial_extrinsic_t", t);
    bool succ_ypr = nh_->getParam("/mlcalib/initial_extrinsic_ypr", ypr);
    EXIT_ASSERT(succ_t && succ_ypr);
    EXIT_ASSERT(t.size() == 3);
    EXIT_ASSERT(ypr.size() == 3);

    LOG_INFO("Set initial t as [%.2f, %.2f, %.2f] ypr as [%.2f, %.2f, %.2f]", t[0], t[1], t[2], ypr[0], ypr[1], ypr[2]);

    H.topLeftCorner<3, 3>() = geoutils::ypr_to_R(Eigen::Vector3d(ypr[0], ypr[1], ypr[2]) / 180.0 * M_PI);
    H.topRightCorner<3, 1>() = Eigen::Vector3d(t[0], t[1], t[2]);

    EXIT_ASSERT(lidar_rig_.find(1) != lidar_rig_.end());
    lidar_rig_.at(1)->FromMatrix(H);
  }

  return 0;
}

int MultiLidarCalib::Clear() {
  odoms_.clear();
  for (auto lidar_id : kLidarIdList) {
    scan_msgs_.at(lidar_id).full_res_pcls.clear();
    scan_msgs_.at(lidar_id).corner_pcls.clear();
    scan_msgs_.at(lidar_id).surf_pcls.clear();
  }

  return 0;
}

int MultiLidarCalib::PrepareDataFromMsgBuffer() {
  frames_.clear();

  LOG_INFO("PrepareDataFromMsgBuffer");

  auto odoms = odoms_;
  auto scan_msgs = scan_msgs_;

  int data_count = 0;
  int gather_skip = 10;
  gather_skip = nh_->param("/mlcalib/gather_skip", gather_skip);
  LOG_INFO("gather_skip = %d", gather_skip);

  while (!odoms.empty()) {
    ros::Time stamp = odoms.front()->header.stamp;

    auto RemoveOldMsgs = [this](const ros::Time &t, std::deque<sensor_msgs::PointCloud2ConstPtr> &buf) {
      while (!buf.empty()) {
        if (buf.front()->header.stamp < (t - ros::Duration(opt_.kTimeAlignDelta))) {
          buf.pop_front();
        } else {
          break;
        }
      }
    };

    for (auto lidar_id : kLidarIdList) {
      RemoveOldMsgs(stamp, scan_msgs.at(lidar_id).full_res_pcls);
      RemoveOldMsgs(stamp, scan_msgs.at(lidar_id).surf_pcls);
      RemoveOldMsgs(stamp, scan_msgs.at(lidar_id).corner_pcls);
    }

    auto CheckNear = [this](const ros::Time &t, std::deque<sensor_msgs::PointCloud2ConstPtr> &buf) -> int {
      if (buf.empty()) {
        return -1;
      }

      if (std::abs((t - buf.front()->header.stamp).toSec()) < opt_.kTimeAlignDelta) {
        return 0;
      }

      return 1;
    };

    // Timestamp alignment, since timestamp between LiDAR#1 and LiDAR#0 is not exactly the same
    bool all_success = true;
    for (auto lidar_id : kLidarIdList) {
      int ret;
      ret = CheckNear(stamp, scan_msgs.at(lidar_id).full_res_pcls);
      all_success &= (ret == 0);
      if (ret) {
        LOG_WARN("Cannot match stamp %.3f vs [%d]full_res_pcls front %.3f",
                 stamp.toSec(),
                 lidar_id,
                 ret > 0 ? scan_msgs.at(lidar_id).full_res_pcls.front()->header.stamp.toSec() : -1.0);
      }

      ret = CheckNear(stamp, scan_msgs.at(lidar_id).surf_pcls);
      all_success &= (ret == 0);
      if (ret) {
        LOG_WARN("Cannot match stamp %.3f vs [%d]surf_pcls front %.3f",
                 stamp.toSec(),
                 lidar_id,
                 ret > 0 ? scan_msgs.at(lidar_id).surf_pcls.front()->header.stamp.toSec() : -1.0);
      }

      ret = CheckNear(stamp, scan_msgs.at(lidar_id).corner_pcls);
      all_success &= (ret == 0);
      if (ret) {
        LOG_WARN("Cannot match stamp %.3f vs [%d]corner_pcls front %.3f",
                 stamp.toSec(),
                 lidar_id,
                 ret > 0 ? scan_msgs.at(lidar_id).corner_pcls.front()->header.stamp.toSec() : -1.0);
      }
    }

    if (all_success) {
      data_count++;

      // Construct a frame, that contains frame pose of LiDAR#0, stamp, and scans from all LiDARs
      Frame::Ptr frame = Frame::Create();

      frame->stamp = stamp;
      frame->tf = Transform::Create();
      frame->tf->FromRosMessage(odoms.front());

      for (auto lidar_id : kLidarIdList) {
        frame->scan.emplace(lidar_id, ScanData());
        frame->scan.at(lidar_id).full_res_pcl =
            CreatePointCloudFromRosMessage(scan_msgs.at(lidar_id).full_res_pcls.front());
        frame->scan.at(lidar_id).corner_pcl =
            CreatePointCloudFromRosMessage(scan_msgs.at(lidar_id).corner_pcls.front());
        frame->scan.at(lidar_id).surf_pcl =
            CreatePointCloudFromRosMessage(scan_msgs.at(lidar_id).surf_pcls.front());

        EXIT_ASSERT(std::abs((scan_msgs.at(lidar_id).full_res_pcls.front()->header.stamp - stamp).toSec())
                        <= opt_.kTimeAlignDelta);
        EXIT_ASSERT(std::abs((scan_msgs.at(lidar_id).corner_pcls.front()->header.stamp - stamp).toSec())
                        <= opt_.kTimeAlignDelta);
        EXIT_ASSERT(std::abs((scan_msgs.at(lidar_id).surf_pcls.front()->header.stamp - stamp).toSec())
                        <= opt_.kTimeAlignDelta);
      }

      // Skip frame to speed up calculation
      if (data_count % gather_skip == 0) {
        LOG_DEBUG("+ Add  data %.3f, %s", frame->stamp.toSec(), frame->tf->ToStr().c_str());
        frames_.push_back(frame);
      } else {
        LOG_DEBUG("^ Skip data %.3f", frame->stamp.toSec());
      }

    } else {
      LOG_WARN("Throw odometry @ %.3f", odoms.front()->header.stamp.toSec());
    }

    odoms.pop_front();
  }

  return 0;
}

int MultiLidarCalib::SetMap(const ros::Time &stamp,
                            const pcl::PointCloud<PointType> &full_res_pcl,
                            const pcl::PointCloud<PointType> &corner_pcl,
                            const pcl::PointCloud<PointType> &surf_pcl) {
  std::lock_guard<std::mutex> lk(map_mutex_);

  map_ = Map::Create();
  map_->stamp = stamp;

  map_->scan.surf_pcl = CreatePointCloudFromPointCloud(surf_pcl);
  map_->scan.corner_pcl = CreatePointCloudFromPointCloud(corner_pcl);
  map_->scan.full_res_pcl = CreatePointCloudFromPointCloud(full_res_pcl);

  return 0;
}

int MultiLidarCalib::ProcessCalibration() {
  optimize_translation_ = false;

  // When running this line, the map only contains LiDAR#0 map points.
  // It will find the initial extrinsic by registering all scans of LiDAR#1 with map.
  ProcessFullCalibration();

  ProcessIncreamentalCalibration();

  optimize_translation_ = true;
  ProcessFullCalibration();

  WriteResultToDisk();
  return 0;
}

int MultiLidarCalib::ProcessFullCalibration() {
  if (frames_.empty() || !map_) {
    LOG_WARN("Data not prepared!");
    return -1;
  }

  int map_corner_num = static_cast<int>(map_->scan.corner_pcl->points.size());
  int map_surf_num = static_cast<int>(map_->scan.surf_pcl->points.size());

  LOG_INFO("Processing calibration...");
  LOG_INFO("map corner num %d surf num %d", map_corner_num, map_surf_num);
  if (map_corner_num < 10 || map_surf_num < 50) {
    ROS_WARN("Map corner and surf num are not enough");
  }

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

  FilterNoisyPointCloud(map_->scan.corner_pcl);
  FilterNoisyPointCloud(map_->scan.surf_pcl);

  kdtreeCornerFromMap->setInputCloud(map_->scan.corner_pcl);
  kdtreeSurfFromMap->setInputCloud(map_->scan.surf_pcl);

  for (int iterCount = 0; iterCount < 1; iterCount++) {
    // ceres::LossFunction *loss_function = NULL;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::LocalParameterization *q_parameterization =
        new ceres::EigenQuaternionParameterization();
    ceres::Problem::Options problem_options;

    ceres::Problem problem(problem_options);

    std::vector<ceres::ResidualBlockId> residual_blks;

    // Add frame pose as variables to be optimized
    for (auto &frame : frames_) {
      EXIT_ASSERT(frame);
      EXIT_ASSERT(frame->tf);

      problem.AddParameterBlock(frame->tf->qData(), 4, q_parameterization);
      problem.AddParameterBlock(frame->tf->tData(), 3);
    }

    // Add extrinsic parameters as variables to be optimized
    for (auto pair : lidar_rig_) {
      EXIT_ASSERT(pair.second);

      problem.AddParameterBlock(pair.second->qData(), 4, q_parameterization);
      problem.AddParameterBlock(pair.second->tData(), 3);

      // Since extrinsic for LiDAR#0 should be identity and not optimized, here we make it constant to keep it as identity.
      if (pair.first == 0) {
        problem.SetParameterBlockConstant(pair.second->qData());
        problem.SetParameterBlockConstant(pair.second->tData());
      }

      if (!optimize_translation_) {
        problem.SetParameterBlockConstant(pair.second->tData());
      }
    }

    // Add constraints
    int all_corner_num = 0;
    int all_surf_num = 0;

    std::map<int, int> lidar_feature_cnt;
    for (auto pair : lidar_rig_) {
      lidar_feature_cnt.emplace(pair.first, 0);
    }

    for (auto frame: frames_) {
      int frame_corner_num = 0;
      int frame_surf_num = 0;

      // Add map-scan constraints for both LiDAR#0 and LiDAR#1
      // The logic is same as laserMapping thread
      for (auto pair:frame->scan) {
        const int lidar_id = pair.first;
        const ScanData &scan = pair.second;
        Transform::Ptr lidar_tf = lidar_rig_.Get(lidar_id);

        int scan_corner_num = 0;
        int scan_surf_num = 0;

        EXIT_ASSERT(lidar_tf);

        pcl::PointCloud<PointType>::Ptr pts_corner(new pcl::PointCloud<PointType>());
        FilterFarawayPoint(100.0, pts_corner);
        DownsampleByOctree(lineRes_, scan.corner_pcl, pts_corner);
//        downSizeFilterCorner_.setInputCloud(scan.corner_pcl);
//        downSizeFilterCorner_.filter(*pts_corner);

        pcl::PointCloud<PointType>::Ptr pts_surf(new pcl::PointCloud<PointType>());
        FilterFarawayPoint(100.0, pts_surf);
        DownsampleByOctree(planeRes_, scan.surf_pcl, pts_surf);
//        downSizeFilterSurf_.setInputCloud(scan.surf_pcl);
//        downSizeFilterSurf_.filter(*pts_surf);

        for (size_t i = 0; i < pts_corner->points.size(); i++) {
          std::vector<int> pointSearchInd;
          std::vector<float> pointSearchSqDis;

          PointType p_scan = pts_corner->points[i];
          PointType p_frame = TransformPclPoint(lidar_tf->q(), lidar_tf->t(), p_scan);
          PointType p_map = TransformPclPoint(frame->tf->q(), frame->tf->t(), p_frame);

          kdtreeCornerFromMap->nearestKSearch(p_map, 5, pointSearchInd, pointSearchSqDis);

          if (pointSearchSqDis[4] < -1.0) { // disable corner points
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++) {
              Eigen::Vector3d tmp(
                  map_->scan.corner_pcl->points[pointSearchInd[j]].x,
                  map_->scan.corner_pcl->points[pointSearchInd[j]].y,
                  map_->scan.corner_pcl->points[pointSearchInd[j]].z);
              center = center + tmp;
              nearCorners.push_back(tmp);
            }
            center = center / 5.0;

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++) {
              Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
              covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            // if is indeed line feature
            // note Eigen library sort eigenvalues in increasing order
            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(p_scan.x, p_scan.y, p_scan.z);
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
              Eigen::Vector3d point_on_line = center;
              Eigen::Vector3d point_a, point_b;
              point_a = 0.1 * unit_direction + point_on_line;
              point_b = -0.1 * unit_direction + point_on_line;

              ceres::CostFunction *cost_function = LidarEdgeCalibFactor::Create(curr_point, point_a, point_b, 1.0);
              ceres::ResidualBlockId
                  rid = problem.AddResidualBlock(cost_function, loss_function, frame->tf->qData(), frame->tf->tData(),
                                                 lidar_tf->qData(), lidar_tf->tData());

              residual_blks.push_back(rid);

              scan_corner_num++;
            }
          }
          /*
          else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
          {
                  Eigen::Vector3d center(0, 0, 0);
                  for (int j = 0; j < 5; j++)
                  {
                          Eigen::Vector3d
          tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                                                                  laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                                                                  laserCloudCornerFromMap->points[pointSearchInd[j]].z);
                          center = center + tmp;
                  }
                  center = center / 5.0;
                  Eigen::Vector3d curr_point(pointOri.x, pointOri.y,
          pointOri.z);
                  ceres::CostFunction *cost_function =
          LidarDistanceFactor::Create(curr_point, center);
                  problem.AddResidualBlock(cost_function, loss_function,
          parameters, parameters + 4);
          }
          */
        }

        for (size_t i = 0; i < pts_surf->points.size(); i++) {
          PointType p_scan = pts_surf->points[i];
          // double sqrtDis = pointOri.x * pointOri.x + pointOri.y *
          // pointOri.y + pointOri.z * pointOri.z;
          PointType p_frame = TransformPclPoint(lidar_tf->q(), lidar_tf->t(), p_scan);
          PointType p_map = TransformPclPoint(frame->tf->q(), frame->tf->t(), p_frame);

          std::vector<int> pointSearchInd;
          std::vector<float> pointSearchSqDis;
          kdtreeSurfFromMap->nearestKSearch(p_map, 5, pointSearchInd, pointSearchSqDis);

          Eigen::Matrix<double, 5, 3> matA0;
          Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
          if (pointSearchSqDis[4] < 1.0) {
            for (int j = 0; j < 5; j++) {
              matA0(j, 0) = map_->scan.surf_pcl->points[pointSearchInd[j]].x;
              matA0(j, 1) = map_->scan.surf_pcl->points[pointSearchInd[j]].y;
              matA0(j, 2) = map_->scan.surf_pcl->points[pointSearchInd[j]].z;
              // printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j,
              // 2));
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();

            // Here n(pa, pb, pc) is unit norm of plane
            bool planeValid = true;
            for (int j = 0; j < 5; j++) {
              // if OX * n > 0.2, then plane is not fit well
              if (fabs(norm(0) * map_->scan.surf_pcl->points[pointSearchInd[j]].x +
                  norm(1) * map_->scan.surf_pcl->points[pointSearchInd[j]].y +
                  norm(2) * map_->scan.surf_pcl->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2) {
                planeValid = false;
                break;
              }
            }
            Eigen::Vector3d curr_point(p_scan.x, p_scan.y, p_scan.z);
            if (planeValid) {
              ceres::CostFunction *cost_function =
                  LidarPlaneNormCalibFactor::Create(curr_point, norm, negative_OA_dot_norm);

              ceres::ResidualBlockId rid = problem.AddResidualBlock(cost_function, loss_function,
                                                                    frame->tf->qData(), frame->tf->tData(),
                                                                    lidar_tf->qData(), lidar_tf->tData());

              residual_blks.push_back(rid);

              scan_surf_num++;
            }
          }
          /*
          else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
          {
                  Eigen::Vector3d center(0, 0, 0);
                  for (int j = 0; j < 5; j++)
                  {
                          Eigen::Vector3d
          tmp(laserCloudSurfFromMap->points[pointSearchInd[j]].x,
                                                                  laserCloudSurfFromMap->points[pointSearchInd[j]].y,
                                                                  laserCloudSurfFromMap->points[pointSearchInd[j]].z);
                          center = center + tmp;
                  }
                  center = center / 5.0;
                  Eigen::Vector3d curr_point(pointOri.x, pointOri.y,
          pointOri.z);
                  ceres::CostFunction *cost_function =
          LidarDistanceFactor::Create(curr_point, center);
                  problem.AddResidualBlock(cost_function, loss_function,
          parameters, parameters + 4);
          }
          */
        }

        frame_corner_num += scan_corner_num;
        frame_surf_num += scan_surf_num;

        lidar_feature_cnt[lidar_id] += (scan_corner_num + scan_surf_num);

        LOG_INFO("scan #%2d @ %.3f add %4d corner, %4d surf",
                 lidar_id,
                 frame->stamp.toSec(),
                 scan_corner_num,
                 scan_surf_num);
      }

      all_corner_num += frame_corner_num;
      all_surf_num += frame_surf_num;
    }

    LOG_INFO("ALL add %4d corner, %4d surf", all_corner_num, all_surf_num);
    for (auto &pair: lidar_feature_cnt) {
      LOG_INFO("lidar #%2d add %4d features", pair.first, pair.second);
    }

    // Save initial value for later recovery after optimizing
    std::map<int, Transform::Ptr> not_optimized_tf;

    for (auto &pair: lidar_feature_cnt) {
      if (pair.second < 50) {
        LOG_WARN("#%d lidar have %d feature, < 50! Not optimize", pair.first, pair.second);
        not_optimized_tf.emplace(pair.first, Transform::Create(*(lidar_rig_.at(pair.first))));
      }
    }

    // Tool function for check optimize details
    auto GetResidualInfoStr = [&problem, &residual_blks]() -> std::string {
      std::stringstream ss;

      ceres::Problem::EvaluateOptions options;
      options.residual_blocks = residual_blks;

      ss << " ==== Cost Statistics ====" << std::endl;
      if (residual_blks.empty()) {
        ss << "Empty residual_blks" << std::endl;
        LOG_WARN("Empty residual_blks");
        return ss.str();
      }

      std::vector<double> residuals;
      double total_cost;

      problem.Evaluate(options, &total_cost, &residuals, nullptr, nullptr);
      ss << string_format("total_cost %f residuals.size() %zu", total_cost, residual_blks.size()) << std::endl;

      for (auto &r : residuals) {
        r = std::abs(r);
      }

      std::sort(residuals.begin(), residuals.end());
      double fcnt = static_cast<double>(residuals.size());
      double minval = residuals.front();
      double maxval = residuals.back();
      double midval = residuals.at(residuals.size() / 2);
      double avgval = std::accumulate(residuals.begin(), residuals.end(), 0.0) / fcnt;

      ss << string_format("cnt %.0f min %f max %f mid %f avg %f", fcnt, minval, maxval, midval, avgval);

      return ss.str();
    };

    // Tool function for check optimize parameters
    auto GetParametersStr = [this]() -> std::string {
      std::stringstream ss;
      ss << " ==== Parameters ====" << std::endl;
      for (auto &frame : frames_) {
        Eigen::Vector3d ypr = geoutils::quaternion_to_ypr(frame->tf->q()) * 180.0 / M_PI;

        ss << string_format("frame @ %.3f: t % .3f % .3f % .3f ypr % .2f % .2f % .2f",
                            frame->stamp.toSec(), frame->tf->t().x(), frame->tf->t().y(), frame->tf->t().z(),
                            ypr.x(), ypr.y(), ypr.z()) << std::endl;
      }

      for (auto pair : lidar_rig_) {
        auto &tf = pair.second;

        ss << string_format("rig #%d: %s", pair.first, tf->ToStr().c_str()) << std::endl;
      }

      return ss.str();
    };

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 1000;
    options.minimizer_progress_to_stdout = true;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-4;
    ceres::Solver::Summary summary;

    LOGS_DEBUG("==== Before " << GetResidualInfoStr());
    LOGS_DEBUG("==== Before " << GetParametersStr());

    ceres::Solve(options, &problem, &summary);

    LOGS_DEBUG(summary.FullReport());

    LOGS_DEBUG("==== After " << GetResidualInfoStr());
    LOGS_DEBUG("==== After " << GetParametersStr());

    for (auto &pair: not_optimized_tf) {
      lidar_rig_.at(pair.first).swap(pair.second);
      LOG_WARN("Restoring lidar %d tf", pair.first);
    }

    if (!not_optimized_tf.empty()) {
      LOGS_DEBUG("==== Restore " << GetResidualInfoStr());
      LOGS_DEBUG("==== Restore " << GetParametersStr());
    }
  }

  return 0;
}

int MultiLidarCalib::ProcessIncreamentalCalibration() {
  if (frames_.empty() || !map_) {
    LOG_WARN("Data not prepared!");
    return -1;
  }

  int map_corner_num = static_cast<int>(map_->scan.corner_pcl->points.size());
  int map_surf_num = static_cast<int>(map_->scan.surf_pcl->points.size());

  LOG_INFO("Processing increamental calibration...");
  LOG_INFO("map corner num %d surf num %d", map_corner_num, map_surf_num);
  if (map_corner_num < 10 || map_surf_num < 50) {
    ROS_WARN("Map corner and surf num are not enough");
  }

  // This process is similar as ProcessCalibration, but it will add the Lidar#1 points to map after every optimization
  for (size_t frame_index = 0; frame_index < frames_.size(); ++frame_index) {

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

    kdtreeCornerFromMap->setInputCloud(map_->scan.corner_pcl);
    kdtreeSurfFromMap->setInputCloud(map_->scan.surf_pcl);


    // ceres::LossFunction *loss_function = NULL;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::LocalParameterization *q_parameterization =
        new ceres::EigenQuaternionParameterization();
    ceres::Problem::Options problem_options;

    ceres::Problem problem(problem_options);

    std::vector<ceres::ResidualBlockId> residual_blks;

    for (size_t i = 0; i <= frame_index; ++i) {
      auto &frame = frames_.at(i);

      EXIT_ASSERT(frame);
      EXIT_ASSERT(frame->tf);

      problem.AddParameterBlock(frame->tf->qData(), 4, q_parameterization);
      problem.AddParameterBlock(frame->tf->tData(), 3);
    }

    // Add extrinsic parameters
    for (auto pair : lidar_rig_) {
      EXIT_ASSERT(pair.second);

      problem.AddParameterBlock(pair.second->qData(), 4, q_parameterization);
      problem.AddParameterBlock(pair.second->tData(), 3);

      if (pair.first == 0) {
        problem.SetParameterBlockConstant(pair.second->qData());
        problem.SetParameterBlockConstant(pair.second->tData());
      }

      if (!optimize_translation_) {
        problem.SetParameterBlockConstant(pair.second->tData());
      }
    }

    // Add constraints
    int all_corner_num = 0;
    int all_surf_num = 0;

    std::map<int, int> lidar_feature_cnt;
    for (auto pair : lidar_rig_) {
      lidar_feature_cnt.emplace(pair.first, 0);
    }

    for (size_t i = 0; i <= frame_index; ++i) {
      auto &frame = frames_.at(i);

      int frame_corner_num = 0;
      int frame_surf_num = 0;

      for (auto pair:frame->scan) {
        const int lidar_id = pair.first;
        const ScanData &scan = pair.second;
        Transform::Ptr lidar_tf = lidar_rig_.Get(lidar_id);

        int scan_corner_num = 0;
        int scan_surf_num = 0;

        EXIT_ASSERT(lidar_tf);

        pcl::PointCloud<PointType>::Ptr pts_corner(new pcl::PointCloud<PointType>());
        FilterFarawayPoint(100.0, pts_corner);
        DownsampleByOctree(lineRes_, scan.corner_pcl, pts_corner);
//        downSizeFilterCorner_.setInputCloud(scan.corner_pcl);
//        downSizeFilterCorner_.filter(*pts_corner);

        pcl::PointCloud<PointType>::Ptr pts_surf(new pcl::PointCloud<PointType>());
        FilterFarawayPoint(100.0, pts_surf);
        DownsampleByOctree(planeRes_, scan.surf_pcl, pts_surf);
//        downSizeFilterSurf_.setInputCloud(scan.surf_pcl);
//        downSizeFilterSurf_.filter(*pts_surf);

        for (size_t i = 0; i < pts_corner->points.size(); i++) {
          std::vector<int> pointSearchInd;
          std::vector<float> pointSearchSqDis;

          PointType p_scan = pts_corner->points[i];
          PointType p_frame = TransformPclPoint(lidar_tf->q(), lidar_tf->t(), p_scan);
          PointType p_map = TransformPclPoint(frame->tf->q(), frame->tf->t(), p_frame);

          kdtreeCornerFromMap->nearestKSearch(p_map, 5, pointSearchInd, pointSearchSqDis);

          if (pointSearchSqDis[4] < -1.0) { // disable corner points
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++) {
              Eigen::Vector3d tmp(
                  map_->scan.corner_pcl->points[pointSearchInd[j]].x,
                  map_->scan.corner_pcl->points[pointSearchInd[j]].y,
                  map_->scan.corner_pcl->points[pointSearchInd[j]].z);
              center = center + tmp;
              nearCorners.push_back(tmp);
            }
            center = center / 5.0;

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++) {
              Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
              covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            // if is indeed line feature
            // note Eigen library sort eigenvalues in increasing order
            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(p_scan.x, p_scan.y, p_scan.z);
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
              Eigen::Vector3d point_on_line = center;
              Eigen::Vector3d point_a, point_b;
              point_a = 0.1 * unit_direction + point_on_line;
              point_b = -0.1 * unit_direction + point_on_line;

              ceres::CostFunction *cost_function = LidarEdgeCalibFactor::Create(curr_point, point_a, point_b, 1.0);
              ceres::ResidualBlockId
                  rid = problem.AddResidualBlock(cost_function, loss_function, frame->tf->qData(), frame->tf->tData(),
                                                 lidar_tf->qData(), lidar_tf->tData());

              residual_blks.push_back(rid);

              scan_corner_num++;
            }
          }
          /*
          else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
          {
                  Eigen::Vector3d center(0, 0, 0);
                  for (int j = 0; j < 5; j++)
                  {
                          Eigen::Vector3d
          tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                                                                  laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                                                                  laserCloudCornerFromMap->points[pointSearchInd[j]].z);
                          center = center + tmp;
                  }
                  center = center / 5.0;
                  Eigen::Vector3d curr_point(pointOri.x, pointOri.y,
          pointOri.z);
                  ceres::CostFunction *cost_function =
          LidarDistanceFactor::Create(curr_point, center);
                  problem.AddResidualBlock(cost_function, loss_function,
          parameters, parameters + 4);
          }
          */
        }

        for (size_t i = 0; i < pts_surf->points.size(); i++) {
          PointType p_scan = pts_surf->points[i];
          // double sqrtDis = pointOri.x * pointOri.x + pointOri.y *
          // pointOri.y + pointOri.z * pointOri.z;
          PointType p_frame = TransformPclPoint(lidar_tf->q(), lidar_tf->t(), p_scan);
          PointType p_map = TransformPclPoint(frame->tf->q(), frame->tf->t(), p_frame);

          std::vector<int> pointSearchInd;
          std::vector<float> pointSearchSqDis;
          kdtreeSurfFromMap->nearestKSearch(p_map, 5, pointSearchInd, pointSearchSqDis);

          Eigen::Matrix<double, 5, 3> matA0;
          Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
          if (pointSearchSqDis[4] < 1.0) {
            for (int j = 0; j < 5; j++) {
              matA0(j, 0) = map_->scan.surf_pcl->points[pointSearchInd[j]].x;
              matA0(j, 1) = map_->scan.surf_pcl->points[pointSearchInd[j]].y;
              matA0(j, 2) = map_->scan.surf_pcl->points[pointSearchInd[j]].z;
              // printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j,
              // 2));
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();

            // Here n(pa, pb, pc) is unit norm of plane
            bool planeValid = true;
            for (int j = 0; j < 5; j++) {
              // if OX * n > 0.2, then plane is not fit well
              if (fabs(norm(0) * map_->scan.surf_pcl->points[pointSearchInd[j]].x +
                  norm(1) * map_->scan.surf_pcl->points[pointSearchInd[j]].y +
                  norm(2) * map_->scan.surf_pcl->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2) {
                planeValid = false;
                break;
              }
            }
            Eigen::Vector3d curr_point(p_scan.x, p_scan.y, p_scan.z);
            if (planeValid) {
              ceres::CostFunction *cost_function =
                  LidarPlaneNormCalibFactor::Create(curr_point, norm, negative_OA_dot_norm);

              ceres::ResidualBlockId rid = problem.AddResidualBlock(cost_function, loss_function,
                                                                    frame->tf->qData(), frame->tf->tData(),
                                                                    lidar_tf->qData(), lidar_tf->tData());

              residual_blks.push_back(rid);

              scan_surf_num++;
            }
          }
          /*
          else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
          {
                  Eigen::Vector3d center(0, 0, 0);
                  for (int j = 0; j < 5; j++)
                  {
                          Eigen::Vector3d
          tmp(laserCloudSurfFromMap->points[pointSearchInd[j]].x,
                                                                  laserCloudSurfFromMap->points[pointSearchInd[j]].y,
                                                                  laserCloudSurfFromMap->points[pointSearchInd[j]].z);
                          center = center + tmp;
                  }
                  center = center / 5.0;
                  Eigen::Vector3d curr_point(pointOri.x, pointOri.y,
          pointOri.z);
                  ceres::CostFunction *cost_function =
          LidarDistanceFactor::Create(curr_point, center);
                  problem.AddResidualBlock(cost_function, loss_function,
          parameters, parameters + 4);
          }
          */
        }

        frame_corner_num += scan_corner_num;
        frame_surf_num += scan_surf_num;

        lidar_feature_cnt[lidar_id] += (scan_corner_num + scan_surf_num);

        LOG_INFO("scan #%2d @ %.3f add %4d corner, %4d surf",
                 lidar_id,
                 frame->stamp.toSec(),
                 scan_corner_num,
                 scan_surf_num);
      }

      all_corner_num += frame_corner_num;
      all_surf_num += frame_surf_num;
    }

    LOG_INFO("ALL add %4d corner, %4d surf", all_corner_num, all_surf_num);
    for (auto &pair: lidar_feature_cnt) {
      LOG_INFO("lidar #%2d add %4d features", pair.first, pair.second);
    }

    std::map<int, Transform::Ptr> not_optimized_tf;

    for (auto &pair: lidar_feature_cnt) {
      if (pair.second < 50) {
        LOG_WARN("#%d lidar have %d feature, < 50! Not optimize", pair.first, pair.second);
        not_optimized_tf.emplace(pair.first, Transform::Create(*(lidar_rig_.at(pair.first))));
      }
    }

    auto GetResidualInfoStr = [&problem, &residual_blks]() -> std::string {
      std::stringstream ss;

      ceres::Problem::EvaluateOptions options;
      options.residual_blocks = residual_blks;

      ss << " ==== Cost Statistics ====" << std::endl;
      if (residual_blks.empty()) {
        ss << "Empty residual_blks" << std::endl;
        LOG_WARN("Empty residual_blks");
        return ss.str();
      }

      std::vector<double> residuals;
      double total_cost;

      problem.Evaluate(options, &total_cost, &residuals, nullptr, nullptr);
      ss << string_format("total_cost %f residuals.size() %zu", total_cost, residual_blks.size()) << std::endl;

      for (auto &r : residuals) {
        r = std::abs(r);
      }

      std::sort(residuals.begin(), residuals.end());
      double fcnt = static_cast<double>(residuals.size());
      double minval = residuals.front();
      double maxval = residuals.back();
      double midval = residuals.at(residuals.size() / 2);
      double avgval = std::accumulate(residuals.begin(), residuals.end(), 0.0) / fcnt;

      ss << string_format("cnt %.0f min %f max %f mid %f avg %f", fcnt, minval, maxval, midval, avgval);

      return ss.str();
    };

    auto GetParametersStr = [this, &frame_index]() -> std::string {
      std::stringstream ss;
      ss << " ==== Parameters ====" << std::endl;
      for (size_t k = 0; k < frame_index; ++k) {
        auto &frame = frames_.at(k);
        Eigen::Vector3d ypr = geoutils::quaternion_to_ypr(frame->tf->q()) * 180.0 / M_PI;

        ss << string_format("frame @ %.3f: t % .3f % .3f % .3f ypr % .2f % .2f % .2f",
                            frame->stamp.toSec(), frame->tf->t().x(), frame->tf->t().y(), frame->tf->t().z(),
                            ypr.x(), ypr.y(), ypr.z()) << std::endl;
      }

      for (auto pair : lidar_rig_) {
        auto &tf = pair.second;

        ss << string_format("rig #%d: %s", pair.first, tf->ToStr().c_str()) << std::endl;
      }

      return ss.str();
    };

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 1000;
    options.minimizer_progress_to_stdout = false;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-4;
    ceres::Solver::Summary summary;

    LOGS_DEBUG("==== Before " << GetResidualInfoStr());
    LOGS_DEBUG("==== Before " << GetParametersStr());

    LOG_DEBUG("Optimizing %zu / %zu", frame_index, frames_.size());
    ceres::Solve(options, &problem, &summary);

    LOGS_DEBUG(summary.FullReport());

    LOGS_DEBUG("==== After " << GetResidualInfoStr());
    LOGS_DEBUG("==== After " << GetParametersStr());

    for (auto &pair: not_optimized_tf) {
      lidar_rig_.at(pair.first).swap(pair.second);
      LOG_WARN("Restoring lidar %d tf", pair.first);
    }

    if (!not_optimized_tf.empty()) {
      LOGS_DEBUG("==== Restore " << GetResidualInfoStr());
      LOGS_DEBUG("==== Restore " << GetParametersStr());
    }

    // Add aux LiDARs(#1) scan points to map
    {
      auto &frame = frames_.at(frame_index);
      for (auto pair : frame->scan) {
        if (pair.first == 0) {
          continue;
        }

        auto lidar_id = pair.first;
        auto scan = pair.second;

        pcl::PointCloud<PointType>::Ptr scan_corner_points = scan.corner_pcl;
        pcl::PointCloud<PointType>::Ptr scan_surf_points = scan.surf_pcl;
        pcl::PointCloud<PointType>::Ptr reg_corner_points(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr reg_surf_points(new pcl::PointCloud<PointType>());

        Transform::Ptr map_tf_scan = Transform::Create();
        // map_tf_scan(LiDAR#1) = map_tf_LiDAR#0 * LiDAR#0_tf_LiDAR#1
        map_tf_scan->FromMatrix(frame->tf->Matrix() * lidar_rig_.at(lidar_id)->Matrix());

        pcl::transformPointCloud(*scan_corner_points,
                                 *reg_corner_points,
                                 map_tf_scan->Matrix().cast<float>());

        pcl::transformPointCloud(*scan_surf_points,
                                 *reg_surf_points,
                                 map_tf_scan->Matrix().cast<float>());

//        pcl::PointCloud<PointType>::Ptr reg_corner_points_flt(new pcl::PointCloud<PointType>());
//        pcl::PointCloud<PointType>::Ptr reg_surf_points_flt(new pcl::PointCloud<PointType>());

//        downSizeFilterCorner_.setInputCloud(reg_corner_points);
//        downSizeFilterCorner_.filter(*reg_corner_points_flt);
//
//        downSizeFilterSurf_.setInputCloud(reg_surf_points);
//        downSizeFilterSurf_.filter(*reg_surf_points_flt);

        // Add aux LiDAR poitns to map points
        (*map_->scan.surf_pcl) += *reg_surf_points;
        (*map_->scan.corner_pcl) += *reg_corner_points;
      }

      // After add, downsample the map points
      pcl::PointCloud<PointType>::Ptr pts_corner(new pcl::PointCloud<PointType>());
      DownsampleByOctree(lineRes_, map_->scan.corner_pcl, pts_corner);
//      downSizeFilterCorner_.setInputCloud(map_->scan.corner_pcl);
//      downSizeFilterCorner_.filter(*pts_corner);
      map_->scan.corner_pcl.swap(pts_corner);

      pcl::PointCloud<PointType>::Ptr pts_surf(new pcl::PointCloud<PointType>());
      DownsampleByOctree(planeRes_, map_->scan.surf_pcl, pts_surf);
//      downSizeFilterSurf_.setInputCloud(map_->scan.surf_pcl);
//      downSizeFilterSurf_.filter(*pts_surf);
      map_->scan.surf_pcl.swap(pts_surf);

      FilterNoisyPointCloud(map_->scan.corner_pcl);
      FilterNoisyPointCloud(map_->scan.surf_pcl);
    }

  }

  return 0;
}

int MultiLidarCalib::PublishResultVisualization() {
  ros::Time t0 = ros::Time::now();

  pcl::PointCloud<PointType> map_points = *map_->scan.surf_pcl + *map_->scan.corner_pcl;

  for (size_t k = 0; k < frames_.size(); ++k) {
    Frame::Ptr frame = frames_.at(k);

    ros::Duration dt = frame->stamp - frames_.front()->stamp;
    LOG_INFO("dt = %.3f t0 = %.3f ", dt.toSec(), t0.toSec());
    while (ros::Time::now() < (t0 + dt)) {
      ros::Duration(0.001).sleep();
    }

    // Publish map every 20 frames, or rviz will be very lagging
    if (k % 20 == 0) {
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(map_points, msg);
      msg.header.stamp = frame->stamp;
      msg.header.frame_id = "/camera_init";
      map_pub_.publish(msg);

      LOG_INFO("Publish map result @ %.3f", frame->stamp.toSec());
    }

    // Publish scan points registered with map using optimized pose and extrinsic
    for (auto &pair : frame->scan) {
      const int lidar_id = pair.first;
      const ScanData &scan = pair.second;

      pcl::PointCloud<PointType> scan_points = *scan.surf_pcl + *scan.corner_pcl;
      pcl::PointCloud<PointType> reg_points;

      Transform::Ptr map_tf_scan = Transform::Create();
      map_tf_scan->FromMatrix(frame->tf->Matrix() * lidar_rig_.at(lidar_id)->Matrix());

      pcl::transformPointCloud(scan_points,
                               reg_points,
                               map_tf_scan->Matrix().cast<float>());

      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(reg_points, msg);
      msg.header.stamp = frame->stamp;
      msg.header.frame_id = "/camera_init";
      EXIT_ASSERT(registered_pub_.find(lidar_id) != registered_pub_.end());
      registered_pub_.at(lidar_id).publish(msg);

      LOG_INFO("Publish reg %d result @ %.3f", lidar_id, frame->stamp.toSec());

      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(map_tf_scan->t().x(), map_tf_scan->t().y(), map_tf_scan->t().z()));
      transform.setRotation(tf::Quaternion(map_tf_scan->q().x(),
                                           map_tf_scan->q().y(),
                                           map_tf_scan->q().z(),
                                           map_tf_scan->q().w()));
      br.sendTransform(tf::StampedTransform(transform,
                                            msg.header.stamp,
                                            "/camera_init", string_format("/mlc_l%d", lidar_id)));
    }
  }

  return 0;
}

int MultiLidarCalib::WriteResultToDisk() {

  for (auto &pair : lidar_rig_) {
    if (pair.first == 0) {
      continue;
    }

    std::ofstream ss(string_format("/tmp/mlcalib_%d_%d.txt", pair.first, std::time(nullptr)));

    EXIT_ASSERT(ss.is_open());

    std::time_t tm = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    Eigen::Vector3d t = pair.second->t();
    Eigen::Quaterniond q = pair.second->q();
    Eigen::Vector3d ypr = geoutils::quaternion_to_ypr(q) * 180.0 / M_PI;
    ss << "% Create @ " << std::asctime(std::localtime(&tm));
    ss << string_format("frame_id: \"lidar0\"") << std::endl;
    ss << string_format("child_frame_id: \"lidar%d\"", pair.first) << std::endl;
    ss << string_format("translation: %.12f %.12f %.12f", t(0), t(1), t(2)) << std::endl;
    ss << string_format("quaternion: %.12f %.12f %.12f %.12f", q.w(), q.x(), q.y(), q.z()) << std::endl;
    ss << string_format("euler_deg_ypr: %.2f %.2f %.2f", ypr(0), ypr(1), ypr(2)) << std::endl;
  }

  return 0;
}

int MultiLidarCalib::FilterNoisyPointCloud(pcl::PointCloud<PointType>::Ptr cloud_in) {
  pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>());

  pcl::StatisticalOutlierRemoval<PointType> sor;

  sor.setInputCloud(cloud_in);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_out);

  cloud_in.swap(cloud_out);

  return 0;
}

int MultiLidarCalib::DownsampleByOctree(const double leafsize,
                                        const pcl::PointCloud<PointType>::Ptr cloud_in,
                                        pcl::PointCloud<PointType>::Ptr& cloud_out) {
  if (leafsize <= 0.0) {
    *cloud_out = *cloud_in;
    return 0;
  }

  pcl::octree::OctreePointCloud<PointType> tree(leafsize);
  tree.setInputCloud(cloud_in);
  tree.addPointsFromInputCloud();

  cloud_out = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

  pcl::octree::OctreePointCloud<PointType>::LeafNodeIterator itl(&tree);

  std::vector<int> indexVector;
  while (*++itl) {

    auto idx = itl.getLeafContainer().getPointIndicesVector().at(0);
    cloud_out->points.push_back(cloud_in->points.at(idx));
  }

//  LOG_DEBUG("input sz %zu output sz %zu", cloud_in->points.size(), cloud_out->points.size());


  return 0;
}

} // namespace mlc
