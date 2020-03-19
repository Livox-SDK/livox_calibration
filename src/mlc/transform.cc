#include "transform.h"

namespace mlc {

Transform::Transform(const std::string &frame_id,
                     const std::string &child_frame_id,
                     const Eigen::Vector3d &t,
                     const Eigen::Quaterniond &q)
    : frame_id_(frame_id), child_frame_id_(child_frame_id), t_(t), q_(q.normalized()) {
  // pass
}

Transform::Transform(const std::string &frame_id,
                     const std::string &child_frame_id,
                     const Eigen::Vector3d &t,
                     const Eigen::Matrix3d &R) : Transform(frame_id, child_frame_id, t, Eigen::Quaterniond(R)) {
  // pass
}

Transform::Transform(const std::string &frame_id,
                     const std::string &child_frame_id) : Transform(frame_id,
                                                                    child_frame_id,
                                                                    Eigen::Vector3d::Zero(),
                                                                    Eigen::Quaterniond::Identity()) {
  // pass
}

Transform::Transform() : Transform("", "") {
  // pass
}

void Transform::SetRotation(const Eigen::Quaterniond &quat) {
  q_ = quat.normalized();
}

void Transform::SetTranslation(const Eigen::Vector3d &trans) {
  t_ = trans;
}

void Transform::SetRotation(const Eigen::Matrix3d &rot) {
  SetRotation(Eigen::Quaterniond(rot));
}

Eigen::Matrix4d Transform::Matrix() const {
  Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
  H.topLeftCorner<3, 3>() = q().toRotationMatrix();
  H.topRightCorner<3, 1>() = t();
  return H;
}

void Transform::FromMatrix(const Eigen::Matrix4d &H) {
  SetRotation(Eigen::Quaterniond(H.topLeftCorner<3, 3>()));
  SetTranslation(H.topRightCorner<3, 1>());
}

Eigen::Isometry3d Transform::IsoMatrix() const {
  return Eigen::Isometry3d(Matrix());
}

Eigen::Matrix3d Transform::R() const { return q_.toRotationMatrix(); }

void Transform::InverseInPlace() {
  child_frame_id_.swap(frame_id_);
  FromMatrix(Matrix().inverse());
}

Transform Transform::Inverse() const {
  Transform other = *this;
  other.InverseInPlace();
  return other;
}

std::string Transform::ToStr(const std::string& euler_seq) const {
  if (euler_seq.compare("pry") == 0) {
    Eigen::Vector3d pry = geoutils::quaternion_to_pry(q()) * 180.0 / M_PI;
    return string_format(
        R"(Transform frame "%s"-->"%s" t %.3f %.3f %.3f q %.6f %.6f %.6f %.6f (pry: %.2f %.2f %.2f))",
        frame_id().c_str(),
        child_frame_id().c_str(),
        t().x(),
        t().y(),
        t().z(),
        q().w(),
        q().x(),
        q().y(),
        q().z(),
        pry(0),
        pry(1),
        pry(2));
  } else {
    Eigen::Vector3d ypr = geoutils::quaternion_to_ypr(q()) * 180.0 / M_PI;
    return string_format(
        R"(Transform "%s"-->"%s" t %.3f %.3f %.3f q %.6f %.6f %.6f %.6f (ypr: %.2f %.2f %.2f))",
        frame_id().c_str(),
        child_frame_id().c_str(),
        t().x(),
        t().y(),
        t().z(),
        q().w(),
        q().x(),
        q().y(),
        q().z(),
        ypr(0),
        ypr(1),
        ypr(2));
  }

}

void Transform::SetStamp(const uint64_t usec) {
  stamp().fromSec(static_cast<double>(usec) / 1000.0 / 1000.0);
}

//int Transform::FromDcosMessage(dcos_tf::TransformStamped *dcos_msg) {
//  SetStamp(dcos_msg->timestamp);
//
//  frame_id() = dcos_msg->frame_id;
//
//  child_frame_id() = dcos_msg->child_frame_id;
//
//  SetTranslation(Eigen::Vector3d(dcos_msg->transform.translation.x,
//                                 dcos_msg->transform.translation.y,
//                                 dcos_msg->transform.translation.z));
//  SetRotation(Eigen::Quaterniond(dcos_msg->transform.rotation.w,
//                                 dcos_msg->transform.rotation.x,
//                                 dcos_msg->transform.rotation.y,
//                                 dcos_msg->transform.rotation.z));
//  return 0;
//}

int Transform::FromRosMessage(nav_msgs::OdometryConstPtr& msg) {
  stamp_ = msg->header.stamp;

  frame_id() = msg->header.frame_id;

  child_frame_id() = msg->child_frame_id;

  SetTranslation(Eigen::Vector3d(msg->pose.pose.position.x,
                                 msg->pose.pose.position.y,
                                 msg->pose.pose.position.z));
  SetRotation(Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                 msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y,
                                 msg->pose.pose.orientation.z));

  return 0;
}

//int Transform::FromYamlNode(const YAML::Node &node) {
//  int fail_cnt = 0;
//
//  std::string stamp_string;
//  if (!GetDataFromNode(node, "stamp", &stamp_string)) {
//    LOG_ERROR("Cannot read stamp!");
//    fail_cnt++;
//  } else {
//    stamp().fromNSec(std::stoull(stamp_string));
//  }
//
//  if (!GetDataFromNode(node, "frame_id", &frame_id_)) {
//    LOG_ERROR("Cannot read frame_id!");
//    fail_cnt++;
//  }
//
//  if (!GetDataFromNode(node, "child_frame_id", &child_frame_id_)) {
//    LOG_ERROR("Cannot read child_frame_id!");
//    fail_cnt++;
//  }
//
//  if (!GetDataFromNode(node, "translation", &t_)) {
//    LOG_ERROR("Cannot read translation!");
//    fail_cnt++;
//  }
//
//  if (!GetDataFromNode(node, "rotation", &q_)) {
//    LOG_ERROR("Cannot read rotation!");
//    fail_cnt++;
//  }
//
//  return fail_cnt;
//}
//
//int Transform::ToYamlNode(YAML::Node *node) const {
//  int fail_cnt = 0;
//
//  std::stringstream stamp_ss;
//  stamp_ss << static_cast<uint64_t>(stamp().toNSec());
//
//  if (!SetDataToNode("stamp", stamp_ss.str(), node)) {
//    LOG_ERROR("Cannot write stamp!");
//    fail_cnt++;
//  }
//
//  if (!SetDataToNode("frame_id", frame_id_, node)) {
//    LOG_ERROR("Cannot write frame_id!");
//    fail_cnt++;
//  }
//
//  if (!SetDataToNode("child_frame_id", child_frame_id_, node)) {
//    LOG_ERROR("Cannot write child_frame_id!");
//    fail_cnt++;
//  }
//
//  if (!SetDataToNode("translation", t_, node)) {
//    LOG_ERROR("Cannot write translation!");
//    fail_cnt++;
//  }
//
//  if (!SetDataToNode("rotation", q_, node)) {
//    LOG_ERROR("Cannot write rotation!");
//    fail_cnt++;
//  }
//
//  return fail_cnt;
//}

} // namespace mlc