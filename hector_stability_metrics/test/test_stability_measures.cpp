/*
 * Copyright (C) 2020  Stefan Fabian, Martin Oehler
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

#include "eigen_tests.h"
#include <hector_stability_metrics/metrics/force_angle_stability_measure.h>
#include <hector_stability_metrics/metrics/energy_stability_margin.h>
#include <hector_stability_metrics/metrics/stability_metric_base.h>

using namespace hector_stability_metrics;
using namespace Eigen;
TEST(StabilityMeasures, StabilityMetricBase_getLeastStableEdgeValue)
{
  MinimumFunction<double> min_fun = StandardMinimum<double>;
  EnergyStabilityMargin<double> esm(min_fun);
  std::vector<double> vec = { 1, 2, 3, -4 };
  size_t least_stabel_index;
  esm.getLeastStableEdgeValue(vec, least_stabel_index);

  EXPECT_EQ(least_stabel_index, 3);
}

TEST(StabilityMeasures, EnergyStabilityMargin)
{
  MinimumFunction<double> min_fun = StandardMinimum<double>;
  EnergyStabilityMargin<double> esm(min_fun);

  CommonData<double> cd;
  cd.com = Vector3d(0.5, 0.5, 1);
  SupportPolygond sup_pol = { Vector3d(0, 0, 0), Vector3d(0, 1, 0), Vector3d(1, 1, 0), Vector3d(0, 1, 0) };
  std::vector<double> vec;

  esm.getStabilityValue(sup_pol, cd, vec);
}

TEST(StabilityMeasures, forceAngleStabilityMeasure)
{
  // TODO
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
