/*
 * Copyright (C) 2020  Felix Biemüller
 *
 * This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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
Scalar standardMinimum(const std::vector<Scalar>& values)
{
  Scalar min_value = values[0];

  for (int i = 1; i < values.size(); i++)
  {
    min_value = std::min(min_value, values[i]);
  }
  return min_value;
}

template <typename Scalar>
Scalar exponentialWeighting(const std::vector<Scalar>& values, const Scalar& a, const Scalar& b, const Scalar& c)
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
