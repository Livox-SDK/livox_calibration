//
// Created by ltb on 8/20/18.
//

#ifndef ALOAM_MLC_STRING_FORMAT_H
#define ALOAM_MLC_STRING_FORMAT_H

namespace mlc {

inline std::string string_format(const std::string fmt, ...) {
  int size = ((int) fmt.size()) * 2 + 50;  // Use a rubric appropriate for your code
  std::string str;
  va_list ap;
  while (1) {  // Maximum two passes on a POSIX system...
    str.resize(size);
    va_start(ap, fmt);
    int n = vsnprintf((char *) str.data(), size, fmt.c_str(), ap);
    va_end(ap);
    if (n > -1 && n < size) {  // Everything worked
      str.resize(n);
      return str;
    }
    if (n > -1)        // Needed size returned
      size = n + 1;  // For null char
    else
      size *= 2;  // Guess at a larger size (OS specific)
  }
  return str;
};

template<typename T, typename... Args>
inline std::string sfmt(const std::string &fmt, Args... args) {
  return string_format(fmt, args...);
}
} // namespace visfm

#endif //VISFM_STRING_FORMAT_H
