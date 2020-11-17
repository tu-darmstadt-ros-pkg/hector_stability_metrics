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

#ifndef WALKER_CHAIR__STATIC_STABILITY_MARGIN_H
#define WALKER_CHAIR__STATIC_STABILITY_MARGIN_H

#include <hector_stability_metrics/metrics/static_stability_margin.h>
#include "stability_metric_base.h"

namespace walker_chair_stability_metrics
{
template<typename Scalar, typename DataStruct = hector_stability_metrics::CenterOfMassData<Scalar>>
class StaticStabilityMargin : public StabilityMetricBase<StaticStabilityMargin<Scalar, DataStruct>>
{
public:
  /** @brief StaticStabilityMargin This class computes the Static Stability Margin defined in "On the Stability Properties of Quadruped Creeping Gaits" (McGhee, Frank, 1968).
   * @param minimum_function The minimum function that should be used for determining the overall stability.
   * @param use_signed_version If set to false the original unsigened version as defined by Garcia and De Santos is used.
   */
  StaticStabilityMargin(
    hector_stability_metrics::math::MinimumFunction<Scalar> minimum_function = hector_stability_metrics::math::standardMinimum )
    : StabilityMetricBase<StaticStabilityMargin<Scalar, DataStruct>>( minimum_function ) { }

  void computeStabilityValueForAllEdgesImpl( const hector_stability_metrics::SupportPolygon<Scalar> &support_polygon,
                                             const DataStruct &data, std::vector<Scalar> &stability_vector )
  {
    hector_stability_metrics::computeStaticStabilityMargin( support_polygon, data, stability_vector );
  }
};

STABLITY_CREATE_DERIVED_METRIC_TRAITS( StaticStabilityMargin )
}  // namespace walker_chair_stability_metrics

#endif  // WALKER_CHAIR__STATIC_STABILITY_MARGIN_H
