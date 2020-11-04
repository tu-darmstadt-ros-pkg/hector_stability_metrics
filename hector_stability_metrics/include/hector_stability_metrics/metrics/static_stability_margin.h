#ifndef HECTOR_STABILITY_METRICS_STATIC_STABILITY_MARGIN_H
#define HECTOR_STABILITY_METRICS_STATIC_STABILITY_MARGIN_H

#include "hector_stability_metrics/support_polygon.h"
#include "hector_stability_metrics/types.h"
#include "hector_stability_metrics/metrics/stability_metric_base.h"

namespace hector_stability_metrics
{
template <typename Scalar, typename DataStruct = CommonData<Scalar>>
class StaticStabilityMargin : public StabilityMetricBase<StaticStabilityMargin<Scalar, DataStruct>>
{
public:
  StaticStabilityMargin(MinimumFunction<Scalar> minimum_function = StandardMinimum)
    : StabilityMetricBase<StaticStabilityMargin<Scalar, DataStruct>>(minimum_function)
  {}

  void computeStabilityValueForAllEdgesImpl(const SupportPolygon<Scalar>& support_polygon, const DataStruct& data, std::vector<Scalar>& stability_vector)
  {
    size_t nr_of_edges = support_polygon.size();
    stability_vector.resize(nr_of_edges);

    Matrix3<Scalar> project_2d;
    project_2d.setZero();
    project_2d(0, 0) = 1;
    project_2d(1, 1) = 1;
    for (int i = 0; i < nr_of_edges; i++)
    {
      Vector3<Scalar> edge = this->getSupportPolygonEdge(support_polygon, i);
      Vector3<Scalar> projected_edge = project_2d * edge;
      Vector3<Scalar> projected_edge_point = project_2d * support_polygon[i];
      Vector3<Scalar> projected_com = project_2d * data.com;

      Scalar signed_distance_to_edge = (projected_edge.normalized().cross(projected_edge_point - projected_com))(2);
      stability_vector[i] = signed_distance_to_edge;
    }
  }
};

STABLITY_CREATE_DERIVED_METRIC_TRAITS(StaticStabilityMargin)

}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_STATIC_STABILITY_MARGIN_H
