#ifndef HECTOR_STABILITY_METRICS_MINIMUM_FUNCTIONS_H
#define HECTOR_STABILITY_METRICS_MINIMUM_FUNCTIONS_H
#include <vector>
#include <math.h>
template <typename Scalar>

Scalar StandardMinimum(std::vector<Scalar>& values)
{
  Scalar min_value = values[0];

  for (int i = 1; i < values.size(); i++)
  {
    min_value = min(min_value, values[i]);
  }
  return min_value;
}

template <typename Scalar>
Scalar ExponentialWeightig(std::vector<Scalar>& values, Scalar a, Scalar b, Scalar c)
{
  Scalar value = 0;

  for (int i = 0; i < values.size(); i++)
  {
    value += exp(-(b * values[i] + c));
  }
  return value * a / values.size();
}

#endif  // HECTOR_STABILITY_METRICS_MINIMUM_FUNCTIONS_H
