// Copyright (c) 2026 Aljoscha Schmidt. Licensed under the MIT license. See LICENSE file in the
// project root for full license information.

#include <hector_stability_metrics/metrics/tipping.h>

#include "eigen_tests.h"
#include <hector_stability_metrics/metrics/normalized_energy_stability_margin.h>

#include <Eigen/Dense>

using namespace hector_stability_metrics;
using namespace hector_stability_metrics::math;
using namespace Eigen;

namespace
{
constexpr double kTol = 1e-9;

Vector3dList square()
{
  return { Vector3d( 0, 0, 0 ), Vector3d( 0, 1, 0 ), Vector3d( 1, 1, 0 ), Vector3d( 1, 0, 0 ) };
}

Vector3d rotate( const TippingAxis<double> &axis, double angle, const Vector3d &p )
{
  return axis.point + AngleAxisd( angle, axis.direction ) * ( p - axis.point );
}
} // namespace

TEST( Tipping, TippingAxisTurnsOutward )
{
  const Vector3dList polygon = square();
  const Vector3d com( 0.5, 0.5, 0.5 );
  std::vector<double> nesm;
  computeNormalizedEnergyStabilityMargin( polygon, nesm, com );
  for ( size_t i = 0; i < polygon.size(); ++i ) {
    const TippingAxis<double> axis = tippingAxis( polygon, i );
    EXPECT_NEAR( axis.direction.norm(), 1.0, kTol );
    // Turning toward the edge lifts the com until it stands above the edge, where the
    // height gained is the NESM of that edge.
    const double to_top = std::atan2( 0.5, 0.5 );
    const Vector3d top = rotate( axis, to_top, com );
    EXPECT_NEAR( top.z() - com.z(), nesm[i], 1e-9 ) << "edge " << i;
  }
}

TEST( Tipping, AxisInertiaParallelAxis )
{
  const Matrix3d inertia = Vector3d( 0.1, 0.2, 0.3 ).asDiagonal();
  const TippingAxis<double> axis{ Vector3d( 0, 0, 0 ), Vector3d::UnitX() };
  EXPECT_NEAR( axisInertia( inertia, 2.0, Vector3d( 5, 0, 1 ), axis ), 0.1 + 2.0, kTol );
}

TEST( Tipping, KineticEnergyOfAPointMassPivot )
{
  const double mass = 2, omega = 3, g = 9.81;
  const TippingAxis<double> axis{ Vector3d( 0, 0, 0 ), Vector3d::UnitX() };
  const Vector3d com( 0.7, 0, 1 );
  const Vector3d velocity = omega * axis.direction.cross( com - axis.point );
  const double expected = 0.5 * mass * omega * omega / ( mass * g );
  EXPECT_NEAR( edgeKineticEnergy<double>( Matrix3d::Zero(), mass, com, Vector3d::Zero(), velocity,
                                          axis, g ),
               expected, kTol );
  EXPECT_NEAR( edgeKineticEnergy<double>( Matrix3d::Zero(), mass, com, Vector3d::Zero(),
                                          -velocity, axis, g ),
               -expected, kTol );
}

TEST( Tipping, KineticEnergyCountsSpinAboutTheCom )
{
  const double mass = 1, g = 10;
  const Matrix3d inertia = Matrix3d::Identity() * 0.5;
  const TippingAxis<double> axis{ Vector3d( 0, 0, 0 ), Vector3d::UnitY() };
  const Vector3d com( 0, 0, 1 );
  // Pure spin about the com with the com at rest: L = 0.5, I_axis = 1.5.
  const double energy = edgeKineticEnergy<double>( inertia, mass, com, Vector3d( 0, 0.5, 0 ),
                                                   Vector3d::Zero(), axis, g );
  EXPECT_NEAR( energy, 0.25 / ( 2 * 1.5 ) / ( mass * g ), kTol );
}

TEST( Tipping, TippingAxisOnAnIrregularTiltedPolygon )
{
  const Vector3dList polygon = { Vector3d( 0, 0, 0 ), Vector3d( 0.1, 1.2, 0.2 ),
                                 Vector3d( 1.3, 0.9, 0.1 ), Vector3d( 0.8, -0.2, -0.1 ) };
  for ( const Vector3d &com : { Vector3d( 0.5, 0.4, 0.8 ), Vector3d( -0.3, 0.5, 0.6 ) } ) {
    std::vector<double> nesm;
    computeNormalizedEnergyStabilityMargin( polygon, nesm, com );
    for ( size_t i = 0; i < polygon.size(); ++i ) {
      const TippingAxis<double> axis = tippingAxis( polygon, i );
      // The highest point of the turn is the top of hill one, NESM above the com.
      double best = -1e9, best_angle = 0;
      for ( int k = -3000; k <= 3000; ++k ) {
        const double angle = k * 1e-3;
        const double z = rotate( axis, angle, com ).z();
        if ( z > best ) {
          best = z;
          best_angle = angle;
        }
      }
      EXPECT_NEAR( std::abs( best - com.z() ), std::abs( nesm[i] ), 1e-6 ) << "edge " << i;
      // Inside the polygon the top lies ahead, outside it lies behind.
      EXPECT_EQ( best_angle > 0, nesm[i] > 0 ) << "edge " << i;
    }
  }
}
