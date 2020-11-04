#ifndef HECTOR_STABILITY_METRICS_ENERGY_STABILITY_MARGIN_H
#define HECTOR_STABILITY_METRICS_ENERGY_STABILITY_MARGIN_H

#include "hector_stability_metrics/support_polygon.h"
#include "hector_stability_metrics/types.h"
#include "hector_stability_metrics/metrics/stability_metric_base.h"

namespace hector_stability_metrics
{
template <typename Scalar, typename DataStruct = CommonData<Scalar>>
class NormalizedEnergyStabilityMargin : public StabilityMetricBase<NormalizedEnergyStabilityMargin<Scalar, DataStruct>>
{
public:
  /**
   * @brief EnergyStabilityMargin This class computes the Normalized Energy Stability Margin as defined in "An improved energy stability margin for walking machines subject to
   * dynamic effects" (Garcia, De Santos, 2005), but despite in the original formulation, the stability value is negative for unstable edges of the support polygon in the default
   * mode. This is helpful if used to analyze or optimize the stability. The original unsigned version can be used by by setting use_signed_version to false.
   * @param minimum_function The minimum function that should be used for determining the over all stability.
   * @param use_signed_version If set to false the original unsigened version as defined by Garcia and De Santos is used.
   */
  NormalizedEnergyStabilityMargin(MinimumFunction<Scalar> minimum_function = StandardMinimum, bool use_signed_version = true)
    : StabilityMetricBase<NormalizedEnergyStabilityMargin<Scalar, DataStruct>>(minimum_function)
  {
    use_signed_version_ = use_signed_version;
  }

  void computeStabilityValueForAllEdgesImpl(const SupportPolygon<Scalar>& support_polygon, const DataStruct& data, std::vector<Scalar>& stability_vector)
  {
    size_t nr_of_edges = support_polygon.size();
    stability_vector.resize(nr_of_edges);

    for (size_t i = 0; i < support_polygon.size(); ++i)
    {
      Vector3<Scalar> A = support_polygon[i];
      Vector3<Scalar> edge = this->getSupportPolygonEdge(support_polygon, i);
      Vector3<Scalar> corner_to_com = data.com - A;

      // linear algebra calculations adapted from Ericson Real Time Collision Detection
      Vector3<Scalar> closest_point_on_edge_to_com = A + corner_to_com.dot(edge) / edge.dot(edge) * edge;

      // compute normal of the vertical plane containing the line
      Vector3<Scalar> plane_normal = edge.cross(Vector3<Scalar>(0, 0, 1));
      Scalar distance_com_to_plane = plane_normal.normalized().dot(corner_to_com);

      // variables R, theta, psi and h based on "An improved energy stability margin for walking machines subject to dynamic effects" (Garcia, De Santos, 2005)
      Vector3<Scalar> R = data.com - closest_point_on_edge_to_com;

      // rotation around the edge neccessary to turn com into the vertical plane of the edge
      Scalar theta = asin(distance_com_to_plane / R.norm());
      // edge inclination relative to horizontal plane
      Scalar psi = asin(edge(2) / edge.norm());

      // mass and acceleration due to gravity are ignored, as they only change the scale
      stability_vector[i] = R.norm() * (1 - cos(theta)) * cos(psi);

      // flip the sign if the vehicle is unstable
      if (use_signed_version_ && theta < 0)
      {
        stability_vector[i] = -stability_vector[i];
      }
    }
  }

private:
  bool use_signed_version_;
};

STABLITY_CREATE_DERIVED_METRIC_TRAITS(NormalizedEnergyStabilityMargin)

}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_ENERGY_STABILITY_MARGIN_H
