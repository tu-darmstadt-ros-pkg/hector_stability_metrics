//
// Created by undefined on 15.11.2020.
//

#ifndef HECTOR_STABILITY_METRICS_MATH_NORMALIZATION_H
#define HECTOR_STABILITY_METRICS_MATH_NORMALIZATION_H

#include <type_traits>

namespace hector_stability_metrics
{
template<typename T, typename = void>
struct has_normalization_factor : std::false_type
{
};

template<typename T>
struct has_normalization_factor<T, decltype( T::normalizationFactor, 0 )> : std::true_type
{
};

template<typename Scalar, typename DataStruct>
typename std::enable_if<has_normalization_factor<DataStruct>::value, Scalar>::type normalize( Scalar value,
                                                                                              const DataStruct &data )
{
  return value * data.normalizationFactor();
}

template<typename Scalar, typename DataStruct>
typename std::enable_if<!has_normalization_factor<DataStruct>::value, Scalar>::type normalize( Scalar value,
                                                                                               const DataStruct &data )
{
  return value;
}
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_MATH_NORMALIZATION_H