#ifndef WALKER_CHAIR_FORCE_ANGLE_STABILITY_MARGIN_H
#define WALKER_CHAIR_FORCE_ANGLE_STABILITY_MARGIN_H

#include <hector_stability_metrics/metrics/force_angle_stability_measure.h>
#include "stability_metric_base.h"

namespace walker_chair_stability_metrics
{

template<typename Scalar, typename DataStruct>
class ForceAngleStabilityMargin : public StabilityMetricBase<ForceAngleStabilityMargin<Scalar, DataStruct>>
{
public:
  ForceAngleStabilityMargin(
    hector_stability_metrics::math::MinimumFunction<Scalar> minimum_function = hector_stability_metrics::math::standardMinimum )
    : StabilityMetricBase<ForceAngleStabilityMargin<Scalar, DataStruct>>( minimum_function ) { }

  /*!
   * Computes the Force Angle Stability Measure (Papadopoulos, Rey, 1996) for the given support polygon and returns the
   * index of the least stable axis.
   * The stabilities for each axis are stored in the stability_vector. Previous values are overwritten.
   * @param support_polygon A support polygon where it is assumed that the points are ordered clockwise when viewed from
   * above.
   * @param data A data struct containing all addition
   * @param stability_vector A vector containting the stability values for all edges of the support polygon
   */
  void computeStabilityValueForAllEdgesImpl( const hector_stability_metrics::SupportPolygon<Scalar> &support_polygon,
                                             const DataStruct &data, std::vector<Scalar> &stability_vector )
  {
    hector_stability_metrics::computeForceAngleStabilityMeasure( support_polygon, data, stability_vector );
  }
};

STABLITY_CREATE_DERIVED_METRIC_TRAITS( ForceAngleStabilityMargin )
}  // namespace hector_stability_metrics

#endif  // WALKER_CHAIR_FORCE_ANGLE_STABILITY_MARGIN_H
