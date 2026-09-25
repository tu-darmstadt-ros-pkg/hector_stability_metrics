// Copyright (c) 2020 Stefan Fabian, Martin Oehler, Felix Biemüller. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "eigen_tests.h"
#include <hector_stability_metrics/metrics/force_angle_stability_measure.h>
#include <hector_stability_metrics/metrics/normalized_dynamic_energy_stability_margin.h>
#include <hector_stability_metrics/metrics/normalized_energy_stability_margin.h>
#include <hector_stability_metrics/metrics/static_stability_margin.h>

using namespace hector_stability_metrics;
using namespace hector_stability_metrics::math;
using namespace Eigen;

const double FLOATING_POINT_TOLERANCE = 0.00001;

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

  Vector3dList sup_pol = { Vector3d( 0, 0, 0 ), Vector3d( 0, 2, 0 ), Vector3d( 1, 2, 0 ),
                           Vector3d( 1, 0, 0 ) };
  std::vector<double> edge_stabilities;

  double stability_res =
      computeStaticStabilityMarginValue( sup_pol, edge_stabilities, center_of_mass );

  EXPECT_NEAR( edge_stabilities[0], 0.2, FLOATING_POINT_TOLERANCE );
  EXPECT_NEAR( edge_stabilities[1], 1.3, FLOATING_POINT_TOLERANCE );
  EXPECT_NEAR( edge_stabilities[2], 0.8, FLOATING_POINT_TOLERANCE );
  EXPECT_NEAR( edge_stabilities[3], 0.7, FLOATING_POINT_TOLERANCE );

  EXPECT_NEAR( stability_res, 0.2, FLOATING_POINT_TOLERANCE );

  center_of_mass = Vector3d( -0.2, 0.7, 1 );
  stability_res = computeStaticStabilityMarginValue( sup_pol, edge_stabilities, center_of_mass );

  EXPECT_NEAR( edge_stabilities[0], -0.2, FLOATING_POINT_TOLERANCE );
  EXPECT_NEAR( edge_stabilities[1], 1.3, FLOATING_POINT_TOLERANCE );
  EXPECT_NEAR( edge_stabilities[2], 1.2, FLOATING_POINT_TOLERANCE );
  EXPECT_NEAR( edge_stabilities[3], 0.7, FLOATING_POINT_TOLERANCE );

  EXPECT_NEAR( stability_res, -0.2, FLOATING_POINT_TOLERANCE );
}

TEST( StabilityMeasures, NormalizedEnergyStabilityMargin )
{
  Vector3d center_of_mass = Vector3d( 0.2, 0.7, 1 );

  Vector3dList sup_pol = { Vector3d( 0, 0, 0 ), Vector3d( 0, 2, 0 ), Vector3d( 1, 2, 0 ),
                           Vector3d( 1, 0, 0 ) };
  std::vector<double> edge_stabilities;

  double stability_res =
      computeNormalizedEnergyStabilityMarginValue( sup_pol, edge_stabilities, center_of_mass );

  // TODO: Martin said he'd write these
  //  EXPECT_NEAR( edge_stabilities[0], 0, FLOATING_POINT_TOLERANCE );
  //  EXPECT_NEAR( edge_stabilities[1], 0, FLOATING_POINT_TOLERANCE );
  //  EXPECT_NEAR( edge_stabilities[2], 0, FLOATING_POINT_TOLERANCE );
  //  EXPECT_NEAR( edge_stabilities[3], 0, FLOATING_POINT_TOLERANCE );
  //
  //  EXPECT_NEAR( stability_res, 0, FLOATING_POINT_TOLERANCE );
  //
  //  center_of_mass = Vector3d( -0.2, 0.7, 1 );
  //  stability_res = computeNormalizedEnergyStabilityMarginValue( sup_pol, edge_stabilities, center_of_mass );
  //
  //  EXPECT_NEAR( edge_stabilities[0], 0, FLOATING_POINT_TOLERANCE );
  //  EXPECT_NEAR( edge_stabilities[1], 0, FLOATING_POINT_TOLERANCE );
  //  EXPECT_NEAR( edge_stabilities[2], 0, FLOATING_POINT_TOLERANCE );
  //  EXPECT_NEAR( edge_stabilities[3], 0, FLOATING_POINT_TOLERANCE );
  //
  //  EXPECT_NEAR( stability_res, 0, FLOATING_POINT_TOLERANCE );
}

// Under gravity alone the NDESM equals the NESM, edge for edge.
TEST( StabilityMeasures, NDESMReducesToNESMUnderGravity )
{
  const Vector3d gravity_only( 0, 0, -1 );
  const Vector3dList sup_pol = { Vector3d( 0, 0, 0 ), Vector3d( 0, 2, 0 ), Vector3d( 1, 2, 0 ),
                                 Vector3d( 1, 0, 0 ) };
  // inclined edges, so that cos(psi) is not 1
  const Vector3dList inclined = { Vector3d( 0, 0, 0 ), Vector3d( 0, 2, 0.3 ),
                                  Vector3d( 1, 2, 0.5 ), Vector3d( 1, 0, 0.2 ) };

  for ( const Vector3dList &polygon : { sup_pol, inclined } ) {
    for ( const Vector3d &com : { Vector3d( 0.2, 0.7, 1 ), Vector3d( 0.5, 1.0, 0.8 ),
                                  Vector3d( 0.9, 1.8, 1.4 ) } ) {
      std::vector<double> nesm_edges;
      std::vector<double> ndesm_edges;
      const double nesm = computeNormalizedEnergyStabilityMarginValue( polygon, nesm_edges, com );
      const double ndesm = computeNormalizedDynamicEnergyStabilityMarginValue(
          polygon, ndesm_edges, com, gravity_only );

      ASSERT_EQ( nesm_edges.size(), ndesm_edges.size() );
      for ( size_t i = 0; i < nesm_edges.size(); ++i ) {
        EXPECT_NEAR( ndesm_edges[i], nesm_edges[i], 1e-12 )
            << "edge " << i << " diverges under gravity alone";
      }
      EXPECT_NEAR( ndesm, nesm, 1e-12 );
    }
  }
}

// Closed-form value of one edge, independent of the NESM implementation.
TEST( StabilityMeasures, NDESMAnalyticValue )
{
  const Vector3dList sup_pol = { Vector3d( 0, 0, 0 ), Vector3d( 0, 2, 0 ), Vector3d( 1, 2, 0 ),
                                 Vector3d( 1, 0, 0 ) };
  const Vector3d com( 0.2, 0.7, 1 );

  // Edge 0 runs from (0,0,0) to (0,2,0), so it is horizontal and psi = 0.
  // The closest point on it to the CoM is (0, 0.7, 0), giving
  //   R     = (0.2, 0, 1),        |R| = sqrt(1.04)
  //   theta = asin(0.2 / |R|)
  //   beta  = |R| (1 - cos theta)
  const double r_norm = std::sqrt( 1.04 );
  const double theta = std::asin( 0.2 / r_norm );
  const double expected = r_norm * ( 1 - std::cos( theta ) );

  std::vector<double> edges;
  computeNormalizedDynamicEnergyStabilityMarginValue( sup_pol, edges, com,
                                                      Vector3d( 0, 0, -1 ) );
  EXPECT_NEAR( edges[0], expected, 1e-12 );
}

// An acceleration towards an edge reduces that edge's margin, one away from it increases it.
TEST( StabilityMeasures, NDESMRespondsToAcceleration )
{
  const Vector3dList sup_pol = { Vector3d( 0, 0, 0 ), Vector3d( 0, 2, 0 ), Vector3d( 1, 2, 0 ),
                                 Vector3d( 1, 0, 0 ) };
  const Vector3d com( 0.5, 1.0, 1.0 );

  std::vector<double> static_edges;
  computeNormalizedDynamicEnergyStabilityMarginValue( sup_pol, static_edges, com,
                                                      Vector3d( 0, 0, -1 ) );

  // Accelerating in +x tilts the resultant towards -x, i.e. towards edge 0
  // (the x = 0 side), so edge 0 becomes less stable and edge 2 (x = 1) more so.
  std::vector<double> accel_edges;
  const Vector3d resultant_accel_x = Vector3d( 0, 0, -1 ) - Vector3d( 0.3, 0, 0 );
  computeNormalizedDynamicEnergyStabilityMarginValue( sup_pol, accel_edges, com,
                                                      resultant_accel_x );

  EXPECT_LT( accel_edges[0], static_edges[0] );
  EXPECT_GT( accel_edges[2], static_edges[2] );
}

// The margin of an edge the CoM has crossed is negative.
TEST( StabilityMeasures, NDESMSignFlipsOutsidePolygon )
{
  const Vector3dList sup_pol = { Vector3d( 0, 0, 0 ), Vector3d( 0, 2, 0 ), Vector3d( 1, 2, 0 ),
                                 Vector3d( 1, 0, 0 ) };
  std::vector<double> inside_edges;
  std::vector<double> outside_edges;

  computeNormalizedDynamicEnergyStabilityMarginValue(
      sup_pol, inside_edges, Vector3d( 0.5, 1.0, 1.0 ), Vector3d( 0, 0, -1 ) );
  computeNormalizedDynamicEnergyStabilityMarginValue(
      sup_pol, outside_edges, Vector3d( -0.4, 1.0, 1.0 ), Vector3d( 0, 0, -1 ) );

  EXPECT_GT( inside_edges[0], 0.0 );
  EXPECT_LT( outside_edges[0], 0.0 );
}

TEST( StabilityMeasures, ForceAngleStabilityMeasureNonDifferentiable )
{
  Vector3d center_of_mass = Vector3d( 0.5, 0.5, 1 );
  Vector3d external_force = Vector3d( 0, 0, -9.81 );

  Vector3dList sup_pol = { Vector3d( 0, 0, 0 ), Vector3d( 0, 1, 0 ), Vector3d( 1, 1, 0 ),
                           Vector3d( 1, 0, 0 ) };
  std::vector<double> edge_stabilities;

  double stability = non_differentiable::computeForceAngleStabilityMeasureValue(
      sup_pol, edge_stabilities, center_of_mass, external_force );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
