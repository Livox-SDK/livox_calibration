#ifndef ALOAM_MLC_LOGGER_H
#define ALOAM_MLC_LOGGER_H

#include <unordered_map>
#include <ros/ros.h>
#include "string_format.h"
//#include "dlog_client.h"

namespace mlc {

#define LOG_TRACE(...) ROS_DEBUG(__VA_ARGS__)
#define LOG_DEBUG(...) ROS_DEBUG(__VA_ARGS__)
#define LOG_INFO(...)  ROS_INFO(__VA_ARGS__)
#define LOG_WARN(...)  ROS_WARN(__VA_ARGS__)
#define LOG_ERROR(...) ROS_ERROR(__VA_ARGS__)
#define LOG_FATAL(...) ROS_FATAL(__VA_ARGS__)

#define LOGS_TRACE(...) ROS_DEBUG_STREAM(__VA_ARGS__)
#define LOGS_DEBUG(...) ROS_DEBUG_STREAM(__VA_ARGS__)
#define LOGS_INFO(...)  ROS_INFO_STREAM(__VA_ARGS__)
#define LOGS_WARN(...)  ROS_WARN_STREAM(__VA_ARGS__)
#define LOGS_ERROR(...) ROS_ERROR_STREAM(__VA_ARGS__)
#define LOGS_FATAL(...) ROS_FATAL_STREAM(__VA_ARGS__)

#define EXIT_ASSERT(...) ROS_ASSERT(__VA_ARGS__)
#define EXIT_ASSERT_MSG(...) ROS_ASSERT_MSG(__VA_ARGS__)

} // end of namespace mlc


#endif //ALOAM_MLC_LOGGER_H
