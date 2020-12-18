// Copyright (c) 2020 Stefan Fabian, Martin Oehler, Felix Biemüller. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

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

  Vector3dList sup_pol = { Vector3d( 0, 0, 0 ), Vector3d( 0, 2, 0 ), Vector3d( 1, 2, 0 ), Vector3d( 1, 0, 0 ) };
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

  Vector3dList sup_pol = { Vector3d( 0, 0, 0 ), Vector3d( 0, 2, 0 ), Vector3d( 1, 2, 0 ), Vector3d( 1, 0, 0 ) };
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

TEST( StabilityMeasures, ForceAngleStabilityMeasureNonDifferentiable )
{
  Vector3d center_of_mass = Vector3d( 0.5, 0.5, 1 );
  Vector3d external_force = Vector3d( 0, 0, -9.81 );

  Vector3dList sup_pol = { Vector3d( 0, 0, 0 ), Vector3d( 0, 1, 0 ), Vector3d( 1, 1, 0 ), Vector3d( 1, 0, 0 ) };
  std::vector<double> edge_stabilities;

  double stability = non_differentiable::computeForceAngleStabilityMeasureValue( sup_pol, edge_stabilities,
                                                                                 center_of_mass,
                                                                                 external_force );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
