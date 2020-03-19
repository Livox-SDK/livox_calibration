#ifndef ALOAM_MLC_UTILS_STATUTILS_H
#define ALOAM_MLC_UTILS_STATUTILS_H

#include <numeric>
#include <algorithm>

namespace mlc {

namespace statutils {

template <typename T>
void GetMeanAndVariance(const std::vector<T>& buf, T* pmean, T* pvar) {
  T sum = std::accumulate(buf.begin(), buf.end(), 0);
  T mean = sum / static_cast<T>(buf.size());

  T sq_sum = std::inner_product(buf.begin(), buf.end(), buf.begin(), 0);
  T var = std::sqrt(sq_sum / static_cast<T>(buf.size()) - mean * mean);

  *pmean = mean;
  *pvar = var;
}


} // namespace statutils

} // namespace mlc

#endif //ALOAM_MLC_UTILS_STATUTILS_H
