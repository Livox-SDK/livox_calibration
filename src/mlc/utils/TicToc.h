#ifndef ALOAM_MLC_UTILS_TICTOC_H
#define ALOAM_MLC_UTILS_TICTOC_H

#include <chrono>

namespace mlc {

class TicToc {
 public:
  TicToc() {};

  void Tic() {
    t0_ = std::chrono::high_resolution_clock::now();
  };

  void Toc() {
    t1_ = std::chrono::high_resolution_clock::now();
  };

  double Get() const {
    return 1.0e-9 * static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(t1_ - t0_).count());
  };

 private:
  std::chrono::time_point<std::chrono::high_resolution_clock> t0_;
  std::chrono::time_point<std::chrono::high_resolution_clock> t1_;
};

}

#endif //ALOAM_MLC_UTILS_TICTOC_H
