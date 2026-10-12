// Copyright (c) 2020 Stefan Fabian, Martin Oehler, Felix Biemüller. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "eigen_tests.h"
#include <hector_stability_metrics/metrics/force_angle_stability_measure.h>
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

TEST( StabilityMeasures, MinimumSelectorIgnoresNaN )
{
  const double nan = std::numeric_limits<double>::quiet_NaN();

  MinimumSelector<double, size_t> index_selector;
  index_selector.updateMinimum( 0, -0.7 );
  index_selector.updateMinimum( 1, nan );
  index_selector.updateMinimum( 2, 1.8 );
  EXPECT_EQ( index_selector.getMinimum(), 0UL );

  MinimumSelector<double> value_selector;
  value_selector.updateMinimum( 0, nan );
  value_selector.updateMinimum( 1, 2.0 );
  value_selector.updateMinimum( 2, nan );
  value_selector.updateMinimum( 3, -1.0 );
  EXPECT_EQ( value_selector.getMinimum(), -1.0 );

  // All NaN values keep the minimum NaN
  MinimumSelector<double> nan_selector;
  nan_selector.updateMinimum( 0, nan );
  EXPECT_TRUE( std::isnan( nan_selector.getMinimum() ) );
}

// Regression test: If the center of mass lies (up to floating point rounding) on the plane spanned
// by a tipover axis and the force, the acos input could end up slightly above 1 resulting in a NaN
// stability for that edge and a wrong least stable edge index.
// Values captured from a pose where this happened with the asterix robot model.
TEST( StabilityMeasures, ForceAngleStabilityMeasureComOnTipoverAxis )
{
  using Vector3f = Eigen::Vector3f;
  // Hexfloat literals to preserve the exact bit patterns that trigger the rounding error
  math::Vector3List<float> sup_pol = { Vector3f( 0x1.bcc08p-3f, -0x1.146364p-2f, -0x1.5d797ap-4f ),
                                       Vector3f( 0x1.be9aacp-3f, -0x1.5bf76ep-3f, -0x1.63fd42p-4f ),
                                       Vector3f( 0x1.35695p-2f, -0x1.143b4ep-2f, -0x1.0a2924p-5f ) };
  const Vector3f center_of_mass( -0x1.0344ccp-5f, -0x1.bbdd24p-7f, 0x1.c78ec6p-5f );
  const Vector3f external_force( 0x1.3619eap+2f, -0x1.885678p-2f, -0x1.10ae42p+3f );

  std::vector<float> edge_stabilities;
  size_t least_stable = non_differentiable::computeForceAngleStabilityMeasureLeastStableEdgeIndex(
      sup_pol, edge_stabilities, center_of_mass, external_force );

  for ( size_t i = 0; i < edge_stabilities.size(); ++i )
    EXPECT_FALSE( std::isnan( edge_stabilities[i] ) ) << "Edge " << i << " stability is NaN";
  // Edge 0 is clearly unstable, edge 1 is marginal (com on axis), edge 2 is stable
  EXPECT_EQ( least_stable, 0UL );
  EXPECT_LT( edge_stabilities[0], 0.f );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
