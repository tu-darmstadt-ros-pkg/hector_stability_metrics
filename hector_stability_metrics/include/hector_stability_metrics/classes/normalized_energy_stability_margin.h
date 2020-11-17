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

#ifndef WALKER_CHAIR__ENERGY_STABILITY_MARGIN_H
#define WALKER_CHAIR__ENERGY_STABILITY_MARGIN_H

#include <hector_stability_metrics/metrics/normalized_energy_stability_margin.h>
#include "stability_metric_base.h"

namespace walker_chair_stability_metrics
{
template<typename Scalar, typename DataStruct = hector_stability_metrics::CenterOfMassData<Scalar>>
class NormalizedEnergyStabilityMargin : public StabilityMetricBase<NormalizedEnergyStabilityMargin<Scalar, DataStruct>>
{
public:
  /**
   * @brief NormalizedEnergyStabilityMargin This class computes the Normalized Energy Stability Margin by Hirose et al. as defined in "An improved energy stability margin for
   * walking machines subject to dynamic effects" (Garcia, De Santos, 2005). Additionally to the original formulation, a sign function can be given in order for  the stability
   * value to be negative for unstable edges of the support polygon. This is helpful if used to analyze or optimize the stability. The original unsigned version is used in the
   * default case by setting sign_function to a function allways returning 1.
   * @param minimum_function The minimum function that should be used for determining the overall stability.
   * @param sign_function If set to ConstantOne the original unsigened version as defined by Garcia and De Santos is used. If set to the standard sign function or a function
   * approximating it the stability values for unstable edges will be negative. As the sign function is not contiously differentiable it is recommended to use an approximation when
   * this metric is used for gradient based optimization.
   */
  NormalizedEnergyStabilityMargin(
    hector_stability_metrics::math::MinimumFunction<Scalar> minimum_function = hector_stability_metrics::math::standardMinimum )
    : StabilityMetricBase<NormalizedEnergyStabilityMargin<Scalar, DataStruct>>( minimum_function )
  {
  }

  static Scalar ConstantOne( const Scalar &value ) { return Scalar( 1 ); }

  void computeStabilityValueForAllEdgesImpl( const hector_stability_metrics::SupportPolygon<Scalar> &support_polygon,
                                             const DataStruct &data, std::vector<Scalar> &stability_vector )
  {
    hector_stability_metrics::computeNormalizedEnergyStabilityMargin( support_polygon, data, stability_vector );
  }
};

STABLITY_CREATE_DERIVED_METRIC_TRAITS( NormalizedEnergyStabilityMargin )
}  // namespace walker_chair_stability_metrics

#endif  // WALKER_CHAIR__ENERGY_STABILITY_MARGIN_H
