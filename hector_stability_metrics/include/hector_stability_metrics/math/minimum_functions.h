#ifndef HECTOR_STABILITY_METRICS_MINIMUM_FUNCTIONS_H
#define HECTOR_STABILITY_METRICS_MINIMUM_FUNCTIONS_H
#include <vector>
#include <math.h>
#include <functional>

namespace hector_stability_metrics
{
template <typename Scalar>
using MinimumFunction = std::function<Scalar(const std::vector<Scalar>&)>;

template <typename Scalar>
Scalar StandardMinimum(const std::vector<Scalar>& values)
{
  Scalar min_value = values[0];

  for (int i = 1; i < values.size(); i++)
  {
    min_value = std::min(min_value, values[i]);
  }
  return min_value;
}

template <typename Scalar>
Scalar ExponentialWeightig(const std::vector<Scalar>& values, const Scalar& a, const Scalar& b, const Scalar& c)
{
  Scalar value = 0;

  for (int i = 0; i < values.size(); i++)
  {
    value += exp(-(b * values[i] + c));
  }
  return value * a / values.size();
}
}  // namespace hector_stability_metrics
#endif  // HECTOR_STABILITY_METRICS_MINIMUM_FUNCTIONS_H
