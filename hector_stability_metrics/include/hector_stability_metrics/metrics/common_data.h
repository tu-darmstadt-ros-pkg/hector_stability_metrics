#ifndef HECTOR_STABILITY_METRICS_COMMON_DATA_H
#define HECTOR_STABILITY_METRICS_COMMON_DATA_H

#include "hector_stability_metrics/support_polygon.h"
#include "hector_stability_metrics/types.h"

namespace hector_stability_metrics
{
template <typename Scalar>
struct CommonData

{
  Vector3<Scalar> com;
};
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_COMMON_DATA_H
