// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_STABILITY_METRICS_METRICS_COMMON_DATA_H
#define HECTOR_STABILITY_METRICS_METRICS_COMMON_DATA_H

namespace hector_stability_metrics
{

namespace math
{
template<typename Scalar, typename ReturnT = Scalar, bool is_integral = std::is_integral<ReturnT>::value>
struct MinimumSelector {
  using ReturnType = ReturnT;
  Scalar minimum = std::numeric_limits<Scalar>::quiet_NaN();

  void updateMinimum( size_t, Scalar value )
  {
    if ( value >= minimum )
      return;
    minimum = value;
  }

  Scalar getMinimum() { return minimum; }
};

template<typename Scalar, typename ReturnT>
struct MinimumSelector<Scalar, ReturnT, true> {
  using ReturnType = ReturnT;
  Scalar minimum = std::numeric_limits<Scalar>::quiet_NaN();
  ReturnT minimum_index = 0;

  void updateMinimum( size_t index, Scalar value )
  {
    if ( value >= minimum )
      return;
    minimum = value;
    minimum_index = index;
  }

  ReturnType getMinimum() { return minimum_index; }
};

template<typename Scalar>
struct MinimumSelector<Scalar, void> {
  using ReturnType = void;

  void updateMinimum( size_t, Scalar ) { }

  void getMinimum() { }
};
} // namespace math

//! @brief Finds and returns the value and index of the edge with the lowest stability.
template<typename Scalar>
Scalar getLeastStableEdgeValue( const std::vector<Scalar> &edge_stabilities,
                                size_t &least_stable_edge )
{
  Scalar min_value = std::numeric_limits<Scalar>::infinity();
  for ( int i = 0; i < edge_stabilities.size(); i++ ) {
    if ( edge_stabilities[i] < min_value ) {
      least_stable_edge = i;
      min_value = edge_stabilities[i];
    }
  }
  return min_value;
}

//! @brief Finds and returns the value of the edge with the lowest stability.
template<typename Scalar>
Scalar getLeastStableEdgeValue( const std::vector<Scalar> &edge_stabilities )
{
  size_t index;
  return getLeastStableEdgeValue( edge_stabilities, index );
}
} // namespace hector_stability_metrics

#endif // HECTOR_STABILITY_METRICS_METRICS_COMMON_DATA_H
