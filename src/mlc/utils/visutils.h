#ifndef ALOAM_MLC_UTILS_VISUTILS_H
#define ALOAM_MLC_UTILS_VISUTILS_H

namespace mlc {

template<typename Type>
void DepthToColor(const cv::Mat &depth, const int minDisparity, const int numDisparities, cv::Mat * pcolor) {
  if (!pcolor) {
    return;
  }

  cv::Mat & color = (*pcolor);

  if (!color.data) {
    color = cv::Mat(depth.rows, depth.cols, CV_8UC3);
  }

  for (int i = 0; i < depth.rows; i++) {
    for (int j = 0; j < depth.cols; j++) {
      uint32_t index = i * color.cols * 3 + j * 3;
      float value = (float) depth.at<Type>(i, j) / (minDisparity + numDisparities/* - 1.0*/);
      if (value >= 0 && value <= 0.11) {
        color.data[index + 0] = value / 0.11 * 112 + 143;
        color.data[index + 1] = 0;
        color.data[index + 2] = 0;
      } else if (value > 0.11 && value <= 0.125) {
        color.data[index + 0] = 255;
        color.data[index + 1] = 0;
        color.data[index + 2] = 0;
      } else if (value > 0.125 && value <= 0.36) {
        color.data[index + 0] = 255;
        color.data[index + 1] = (value - 0.125) / 0.235 * 255;
        color.data[index + 2] = 0;
      } else if (value > 0.36 && value <= 0.375) {
        color.data[index + 0] = 255;
        color.data[index + 1] = 255;
        color.data[index + 2] = 0;
      } else if (value > 0.375 && value <= 0.61) {
        color.data[index + 0] = 255 - (value - 0.375) / 0.235 * 255;
        color.data[index + 1] = 255;
        color.data[index + 2] = (value - 0.375) / 0.235 * 255;
      } else if (value > 0.61 && value <= 0.625) {
        color.data[index + 0] = 0;
        color.data[index + 1] = 255;
        color.data[index + 2] = 255;
      } else if (value > 0.625 && value <= 0.86) {
        color.data[index + 0] = 0;
        color.data[index + 1] = 255 - (value - 0.625) / 0.235 * 255;
        color.data[index + 2] = 255;
      } else if (value > 0.86 && value <= 0.875) {
        color.data[index + 0] = 0;
        color.data[index + 1] = 0;
        color.data[index + 2] = 255;
      } else if (value > 0.875 && value < 1) {
        color.data[index + 0] = 0;
        color.data[index + 1] = 0;
        color.data[index + 2] = 255 - (value - 0.875) / 0.125 * 127;
      } else {
        color.data[index + 0] = 0;
        color.data[index + 1] = 0;
        color.data[index + 2] = 0;
      }
    }
  }
}

} // namespace mlc

#endif //ALOAM_MLC_UTILS_VISUTILS_H
