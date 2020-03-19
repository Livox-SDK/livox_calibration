#ifndef ALOAM_MLC_IO_TRANSFORM_H
#define ALOAM_MLC_IO_TRANSFORM_H

#include <memory>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "mlc/logger/logger.h"
#include "mlc/utils/geoutils.h"
#include "mlc/utils/yamlutils.h"

namespace mlc {
class Transform {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<Transform> Ptr;
  typedef std::shared_ptr<const Transform> ConstPtr;

  template<typename... Args>
  static Ptr Create(Args &&... args) {
    return Ptr(new Transform(std::forward<Args>(args)...));
  };

  Transform(const std::string &frame_id,
            const std::string &child_frame_id,
            const Eigen::Vector3d &t,
            const Eigen::Quaterniond &q);

  Transform(const std::string &frame_id,
            const std::string &child_frame_id,
            const Eigen::Vector3d &t,
            const Eigen::Matrix3d &R);

  Transform(const std::string &frame_id,
            const std::string &child_frame_id);

  Transform();

  void SetRotation(const Eigen::Quaterniond &quat);

  void SetTranslation(const Eigen::Vector3d &trans);

  void SetStamp(const uint64_t usec);

  const std::string &frame_id() const { return frame_id_; };
  std::string &frame_id() { return frame_id_; };

  const std::string &child_frame_id() const { return child_frame_id_; };
  std::string &child_frame_id() { return child_frame_id_; };

  const ros::Time &stamp() const { return stamp_; };
  ros::Time &stamp() { return stamp_; };

  const Eigen::Vector3d &t() const { return t_; };

  const Eigen::Quaterniond &q() const { return q_; };

  double *tData() { return t_.data(); };

  double *qData() { return q_.coeffs().data(); };

  void SetRotation(const Eigen::Matrix3d &rot);

  Eigen::Matrix4d Matrix() const;

  void FromMatrix(const Eigen::Matrix4d &H);

  Eigen::Isometry3d IsoMatrix() const;

  Eigen::Matrix3d R() const;

  void InverseInPlace();

  Transform Inverse() const;

  std::string ToStr(const std::string& euler_seq = "") const;

  int FromRosMessage(nav_msgs::OdometryConstPtr& msg);

//  int FromDcosMessage(dcos_tf::TransformStamped *dcos_msg);

//  int FromYamlNode(const YAML::Node &node);

//  int ToYamlNode(YAML::Node *node) const;

  Ptr Clone() const { return Create(*this); };

 private:
  ros::Time stamp_;
  std::string frame_id_;
  std::string child_frame_id_;
  Eigen::Vector3d t_;
  Eigen::Quaterniond q_;
};
} // namespace calinode

#endif //ALOAM_MLC_IO_TRANSFORM_H
