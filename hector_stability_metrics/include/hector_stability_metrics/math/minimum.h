// Copyright (c) 2020 Felix Biemüller, Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_STABILITY_METRICS_MINIMUM_FUNCTIONS_H
#define HECTOR_STABILITY_METRICS_MINIMUM_FUNCTIONS_H

#include <vector>
#include <math.h>
#include <functional>

namespace hector_stability_metrics
{
namespace math
{
template <typename Scalar>
using MinimumFunction = Scalar(const std::vector<Scalar>&);

template <typename Scalar>
Scalar standardMinimum(const std::vector<Scalar>& values);

template <typename Scalar>
Scalar exponentialWeighting(const std::vector<Scalar>& values, const Scalar& a, const Scalar& b, const Scalar& c);

// ====================================
// Implementations of Minimum Functions
// ====================================

template <typename Scalar>
Scalar standardMinimum(const std::vector<Scalar>& values)
{
  Scalar min_value = values[0];

  for (int i = 1; i < values.size(); i++)
  {
    min_value = fmin(min_value, values[i]);
  }
  return min_value;
}

template <typename Scalar>
Scalar exponentialWeighting(const std::vector<Scalar>& values, const Scalar& a, const Scalar& b, const Scalar& c)
{
  Scalar value = 0;

  for (int i = 0; i < values.size(); i++)
  {
    value += std::exp(-(b * values[i] + c));
  }
  return value * a / values.size();
}
}  // namespace math
}  // namespace hector_stability_metrics
#endif  // HECTOR_STABILITY_METRICS_MINIMUM_FUNCTIONS_H
