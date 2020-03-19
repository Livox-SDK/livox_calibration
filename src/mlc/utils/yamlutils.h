#ifndef ALOAM_MLC_UTILS_YAMLUTILS_H
#define ALOAM_MLC_UTILS_YAMLUTILS_H

#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>

namespace YAML {

template<>
struct convert<Eigen::Quaterniond> {
  static Node encode(const Eigen::Quaterniond &rhs) {
    Node node;
    node.push_back(rhs.w());
    node.push_back(rhs.x());
    node.push_back(rhs.y());
    node.push_back(rhs.z());

    return node;
  }

  static bool decode(const Node &node, Eigen::Quaterniond &rhs) {
    if (!node.IsSequence() || node.size() < 4) {
      return false;
    }

    rhs.w() = node[0].as<double>();
    rhs.x() = node[1].as<double>();
    rhs.y() = node[2].as<double>();
    rhs.z() = node[3].as<double>();

    return true;
  }
};


template<typename Matrix_t>
struct convert {
  static Node encode(const Matrix_t &rhs) {
    Node node;
    for (int i = 0; i < rhs.rows(); ++i) {
      for (int j = 0; j < rhs.cols(); ++j) {
        node.push_back(rhs(i, j));
      }
    }

    return node;
  }

  static bool decode(const Node &node, Matrix_t &rhs) {
    if (!node.IsSequence() || node.size() < rhs.cols() * rhs.rows()) {
      return false;
    }

    for (int i = 0; i < rhs.rows(); ++i) {
      for (int j = 0; j < rhs.cols(); ++j) {
        int w = rhs.cols();
        rhs(i, j) = node[i * w + j].as<double>();
      }
    }

    return true;
  }
};

template<>
struct convert<std::unordered_map<int, int>> {
  static Node encode(const std::unordered_map<int, int> &rhs) {
    Node node;
    for (auto it : rhs) {
      node[it.first] = it.second;
    }

    return node;
  };

  static bool decode(const Node &node, std::unordered_map<int, int> &rhs) {
    if (!node.IsMap()) {
      return false;
    }

    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
      rhs[it->first.as<int>()] = it->second.as<int>();
    }

    return true;
  };
};




} // namespace YAML

namespace mlc {
template<typename T, int N>
T GetMatrix(const float buf[N]) {
  T mat;
  for (size_t i = 0; i < mat.rows(); ++i) {
    for (size_t j = 0; j < mat.cols(); ++j) {
      mat(i, j) = buf[i * mat.cols() + j];
    }
  }

  return mat;
};

template<typename T>
bool GetDataFromNode(const YAML::Node &node, const std::string &key, T *data) {
  if (data && node[key]) {
    *data = node[key].as<T>();
    return true;
  } else {
    return false;
  }
}

template<typename T>
bool SetDataToNode(const std::string &key, const T &data, YAML::Node *parent) {
  if (parent && !key.empty()) {
    (*parent)[key] = data;
    return true;
  } else {
    return false;
  }
}

} // namespace mlc

#endif //ALOAM_MLC_UTILS_YAMLUTILS_H
