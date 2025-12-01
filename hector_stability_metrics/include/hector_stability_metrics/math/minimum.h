// Copyright (c) 2020 Felix Biemüller, Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_STABILITY_METRICS_MINIMUM_FUNCTIONS_H
#define HECTOR_STABILITY_METRICS_MINIMUM_FUNCTIONS_H

#include <functional>
#include <math.h>
#include <vector>

namespace hector_stability_metrics
{
namespace math
{
template<typename Scalar>
using MinimumFunction = Scalar( const std::vector<Scalar> & );

template<typename Scalar>
Scalar standardMinimum( const std::vector<Scalar> &values );

template<typename Scalar>
Scalar exponentialWeighting( const std::vector<Scalar> &values, const Scalar &a, const Scalar &b,
                             const Scalar &c );

// ====================================
// Implementations of Minimum Functions
// ====================================

template<typename Scalar>
Scalar standardMinimum( const std::vector<Scalar> &values )
{
  Scalar min_value = std::numeric_limits<Scalar>::max();

  for ( int i = 0; i < values.size(); i++ ) { min_value = fmin( min_value, values[i] ); }
  return min_value;
}

template<typename Scalar>
Scalar exponentialWeighting( const std::vector<Scalar> &values, const Scalar &a, const Scalar &b )
{
  Scalar value = 0;

  for ( int i = 0; i < values.size(); i++ ) { value += exp( -( b * values[i] ) ); }
  return -value * a / values.size();
}

template<typename Scalar>
Scalar logarithmicExponentialWeighting( const std::vector<Scalar> &values, const Scalar &a,
                                        const Scalar &b )
{
  if ( values.empty() ) {
    throw std::invalid_argument( "logarithmicExponentialWeighting: values must not be empty" );
  }

  Scalar value = 0;

  for ( std::size_t i = 0; i < values.size(); ++i ) { value += std::exp( -( b * values[i] ) ); }

  const Scalar eps = std::numeric_limits<Scalar>::epsilon();
  value = std::max( value, eps );

  return -std::log( value ) * a / static_cast<Scalar>( values.size() );
}

} // namespace math
} // namespace hector_stability_metrics
#endif // HECTOR_STABILITY_METRICS_MINIMUM_FUNCTIONS_H
