/*
 * Copyright (C) 2020  Stefan Fabian, Martin Oehler, Felix Biemüller
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
#include <hector_stability_metrics/metrics/static_stability_margin.h>

using namespace hector_stability_metrics;
using namespace Eigen;

const double FLOATING_POINT_TOLLERANCE = 0.00001;

TEST( StabilityMeasures, getLeastStableEdgeValue )
{
  std::vector<double> edge_stabilities = { 1, 2, 3, -4 };
  size_t least_stable_index;
  getLeastStableEdgeValue( edge_stabilities, least_stable_index );

  EXPECT_EQ( least_stable_index, 3 );
}

TEST( StabilityMeasures, StaticStabilityMargin )
{
  Vector3d center_of_mass = Vector3d( 0.2, 0.7, 1 );

  SupportPolygond sup_pol = { Vector3d( 0, 0, 0 ), Vector3d( 0, 2, 0 ), Vector3d( 1, 2, 0 ), Vector3d( 1, 0, 0 ) };
  std::vector<double> edge_stabilities;

  double stability_res = computeStaticStabilityMarginValue( sup_pol, edge_stabilities, center_of_mass );

  EXPECT_NEAR( edge_stabilities[0], 0.2, FLOATING_POINT_TOLLERANCE );
  EXPECT_NEAR( edge_stabilities[1], 1.3, FLOATING_POINT_TOLLERANCE );
  EXPECT_NEAR( edge_stabilities[2], 0.8, FLOATING_POINT_TOLLERANCE );
  EXPECT_NEAR( edge_stabilities[3], 0.7, FLOATING_POINT_TOLLERANCE );

  EXPECT_NEAR( stability_res, 0.2, FLOATING_POINT_TOLLERANCE );

  center_of_mass = Vector3d( -0.2, 0.7, 1 );
  stability_res = computeStaticStabilityMarginValue( sup_pol, edge_stabilities, center_of_mass );

  EXPECT_NEAR( edge_stabilities[0], -0.2, FLOATING_POINT_TOLLERANCE );
  EXPECT_NEAR( edge_stabilities[1], 1.3, FLOATING_POINT_TOLLERANCE );
  EXPECT_NEAR( edge_stabilities[2], 1.2, FLOATING_POINT_TOLLERANCE );
  EXPECT_NEAR( edge_stabilities[3], 0.7, FLOATING_POINT_TOLLERANCE );

  EXPECT_NEAR( stability_res, -0.2, FLOATING_POINT_TOLLERANCE );
}

TEST( StabilityMeasures, NormalizedEnergyStabilityMargin )
{
  Vector3d center_of_mass = Vector3d( 0.2, 0.7, 1 );

  SupportPolygond sup_pol = { Vector3d( 0, 0, 0 ), Vector3d( 0, 2, 0 ), Vector3d( 1, 2, 0 ), Vector3d( 1, 0, 0 ) };
  std::vector<double> edge_stabilities;

  double stability_res = computeNormalizedEnergyStabilityMarginValue( sup_pol, edge_stabilities, center_of_mass );

  EXPECT_NEAR( edge_stabilities[0], 0, FLOATING_POINT_TOLLERANCE );
  EXPECT_NEAR( edge_stabilities[1], 0, FLOATING_POINT_TOLLERANCE );
  EXPECT_NEAR( edge_stabilities[2], 0, FLOATING_POINT_TOLLERANCE );
  EXPECT_NEAR( edge_stabilities[3], 0, FLOATING_POINT_TOLLERANCE );

  EXPECT_NEAR( stability_res, 0, FLOATING_POINT_TOLLERANCE );

  center_of_mass = Vector3d( -0.2, 0.7, 1 );
  stability_res = computeNormalizedEnergyStabilityMarginValue( sup_pol, edge_stabilities, center_of_mass );

  EXPECT_NEAR( edge_stabilities[0], 0, FLOATING_POINT_TOLLERANCE );
  EXPECT_NEAR( edge_stabilities[1], 0, FLOATING_POINT_TOLLERANCE );
  EXPECT_NEAR( edge_stabilities[2], 0, FLOATING_POINT_TOLLERANCE );
  EXPECT_NEAR( edge_stabilities[3], 0, FLOATING_POINT_TOLLERANCE );

  EXPECT_NEAR( stability_res, 0, FLOATING_POINT_TOLLERANCE );
}

TEST( StabilityMeasures, ForceAngleStabilityMeasure )
{
  Vector3d center_of_mass = Vector3d( 0.5, 0.5, 1 );
  Vector3d external_force = Vector3d( 0, 0, -9.81 );

  SupportPolygond sup_pol = { Vector3d( 0, 0, 0 ), Vector3d( 0, 1, 0 ), Vector3d( 1, 1, 0 ), Vector3d( 1, 0, 0 ) };
  std::vector<double> edge_stabilities;

  double stability = computeForceAngleStabilityMeasureValue( sup_pol, edge_stabilities, center_of_mass,
                                                             external_force );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
