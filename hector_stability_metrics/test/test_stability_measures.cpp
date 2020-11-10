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
#include <hector_stability_metrics/metrics/normalized_energy_stability_margin.h>
#include <hector_stability_metrics/metrics/stability_metric_base.h>
#include <hector_stability_metrics/metrics/static_stability_margin.h>

using namespace hector_stability_metrics;
using namespace Eigen;

const double FLOATING_POINT_TOLLERANCE = 0.00001;
TEST(StabilityMeasures, StabilityMetricBase_getLeastStableEdgeValue)
{
  MinimumFunction<double> min_fun = StandardMinimum<double>;
  NormalizedEnergyStabilityMargin<double> esm(min_fun);
  std::vector<double> vec = { 1, 2, 3, -4 };
  size_t least_stabel_index;
  esm.getLeastStableEdgeValue(vec, least_stabel_index);

  EXPECT_EQ(least_stabel_index, 3);
}

TEST(StabilityMeasures, StaticStabilityMargin)
{
  MinimumFunction<double> min_fun = StandardMinimum<double>;
  StaticStabilityMargin<double> ssm(min_fun);

  CommonData<double> cd;
  cd.com = Vector3d(0.2, 0.7, 1);

  SupportPolygond sup_pol = { Vector3d(0, 0, 0), Vector3d(0, 2, 0), Vector3d(1, 2, 0), Vector3d(1, 0, 0) };
  std::vector<double> vec;

  double stability_res = ssm.getStabilityValue(sup_pol, cd, vec);

  EXPECT_NEAR(vec[0], 0.2, FLOATING_POINT_TOLLERANCE);
  EXPECT_NEAR(vec[1], 1.3, FLOATING_POINT_TOLLERANCE);
  EXPECT_NEAR(vec[2], 0.8, FLOATING_POINT_TOLLERANCE);
  EXPECT_NEAR(vec[3], 0.7, FLOATING_POINT_TOLLERANCE);

  EXPECT_NEAR(stability_res, 0.2, FLOATING_POINT_TOLLERANCE);

  cd.com = Vector3d(-0.2, 0.7, 1);
  stability_res = ssm.getStabilityValue(sup_pol, cd, vec);

  EXPECT_NEAR(vec[0], -0.2, FLOATING_POINT_TOLLERANCE);
  EXPECT_NEAR(vec[1], 1.3, FLOATING_POINT_TOLLERANCE);
  EXPECT_NEAR(vec[2], 1.2, FLOATING_POINT_TOLLERANCE);
  EXPECT_NEAR(vec[3], 0.7, FLOATING_POINT_TOLLERANCE);

  EXPECT_NEAR(stability_res, -0.2, FLOATING_POINT_TOLLERANCE);
}

TEST(StabilityMeasures, NormalizedEnergyStabilityMargin)
{
  MinimumFunction<double> min_fun = StandardMinimum<double>;
  NormalizedEnergyStabilityMargin<double, CommonData<double>> esm(min_fun);

  CommonData<double> cd;
  cd.com = Vector3d(0.2, 0.7, 1);

  SupportPolygond sup_pol = { Vector3d(0, 0, 0), Vector3d(0, 2, 0), Vector3d(1, 2, 0), Vector3d(1, 0, 0) };
  std::vector<double> vec;

  double stability_res = esm.getStabilityValue(sup_pol, cd, vec);

  EXPECT_NEAR(vec[0], 0, FLOATING_POINT_TOLLERANCE);
  EXPECT_NEAR(vec[1], 0, FLOATING_POINT_TOLLERANCE);
  EXPECT_NEAR(vec[2], 0, FLOATING_POINT_TOLLERANCE);
  EXPECT_NEAR(vec[3], 0, FLOATING_POINT_TOLLERANCE);

  EXPECT_NEAR(stability_res, 0, FLOATING_POINT_TOLLERANCE);

  cd.com = Vector3d(-0.2, 0.7, 1);
  stability_res = esm.getStabilityValue(sup_pol, cd, vec);

  EXPECT_NEAR(vec[0], 0, FLOATING_POINT_TOLLERANCE);
  EXPECT_NEAR(vec[1], 0, FLOATING_POINT_TOLLERANCE);
  EXPECT_NEAR(vec[2], 0, FLOATING_POINT_TOLLERANCE);
  EXPECT_NEAR(vec[3], 0, FLOATING_POINT_TOLLERANCE);

  EXPECT_NEAR(stability_res, 0, FLOATING_POINT_TOLLERANCE);
}

template <typename Scalar>
struct ForceData : CommonData<Scalar>
{
  Vector3<Scalar> external_force;
  Scalar normalization_factor = Scalar(1);
};
TEST(StabilityMeasures, ForceAngleStabilityMeasure)
{
  MinimumFunction<double> min_fun = StandardMinimum<double>;
  ForceAngleStabilityMargin<double, ForceData<double>> fasm(min_fun);

  ForceData<double> fd;
  fd.com = Vector3d(0.5, 0.5, 1);
  fd.external_force = Vector3d(0, 0, -9.81);

  SupportPolygond sup_pol = { Vector3d(0, 0, 0), Vector3d(0, 1, 0), Vector3d(1, 1, 0), Vector3d(1, 0, 0) };
  std::vector<double> vec;

  fasm.getStabilityValue(sup_pol, fd, vec);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
