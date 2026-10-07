// Copyright (c) 2023 Stefan Fabian, Martin Oehler, Felix Biemüller. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "eigen_tests.h"
#include <hector_stability_metrics/math/support_polygon.h>
#include <hector_stability_metrics/metrics/normalized_energy_stability_margin.h>

using namespace hector_stability_metrics;
using namespace hector_stability_metrics::math;
using namespace Eigen;

TEST( ConvexHull, convexityThreshold )
{
  Vector3dList points = {
      // rectangle polygon
      Vector3d( 0, 0, 0 ),    Vector3d( 2, 0, 0 ),    Vector3d( 2, 3, 0 ),
      Vector3d( 0, 3, 0 ),    Vector3d( 2.01, 2, 0 ), // extra point slightly outside rectangle polygon
      Vector3d( 1.99, 2, 0 ), // extra point slightly inside rectangle polygon
  };

  Vector3dList default_hull = supportPolygonFromUnsortedContactPoints( points, 0.0 );
  Vector3dList default_hull_expected = {
      Vector3d( 0, 0, 0 ),    Vector3d( 0, 3, 0 ), Vector3d( 2, 3, 0 ),
      Vector3d( 2.01, 2, 0 ), Vector3d( 2, 0, 0 ),
  };
  EXPECT_EQ( default_hull, default_hull_expected );

  Vector3dList conservative_hull = supportPolygonFromUnsortedContactPoints( points, 0.1 );
  Vector3dList conservative_hull_expected = {
      Vector3d( 0, 0, 0 ),
      Vector3d( 0, 3, 0 ),
      Vector3d( 2, 3, 0 ),
      Vector3d( 2, 0, 0 ),
  };
  EXPECT_EQ( conservative_hull, conservative_hull_expected );
}

// Support points of one logged step of a tracked robot on a ramp: a track and a flipper touch the
// same edge, two contacts 2 um apart seen from above. Kept apart they made an edge without a
// direction whose NESM, 0.0216, was the polygon's, below the 0.0281 of the real edges.
TEST( ConvexHull, coincidentContactsAreOneVertex )
{
  const Vector3dList points = { Vector3d( 1.46751886288, -0.699852389397, 0.0942029860567 ), Vector3d( 1.46751341837, -0.700014070671, 0.0941782081095 ), Vector3d( 1.51148522039, -0.70672378628, 0.13351247248 ), Vector3d( 1.45912124627, -0.70791804993, 0.147717238069 ), Vector3d( 1.4591191044, -0.70791804993, 0.147709244472 ), Vector3d( 1.51054124221, -0.270907948935, 0.144557496138 ), Vector3d( 1.51101541531, -0.270896946473, 0.14452548936 ), Vector3d( 1.51100471111, -0.271214820759, 0.144476774552 ), Vector3d( 1.52008156828, -0.271195568046, 0.144481933301 ), Vector3d( 2.04957837227, -0.437593437809, 0.123252393237 ), Vector3d( 2.06903629851, -0.49889993307, 0.122432559624 ),  };
  const Vector3d com( 1.7585899942, -0.492148699284, 0.347901918306 );

  const Vector3dList unmerged = supportPolygonFromUnsortedContactPoints( points, 0.0, 0.0 );
  ASSERT_EQ( unmerged.size(), 8U );

  const Vector3dList hull = supportPolygonFromUnsortedContactPoints( points );
  ASSERT_EQ( hull.size(), 7U );
  for ( size_t i = 0; i < hull.size(); ++i )
    EXPECT_GT( ( hull[( i + 1 ) % hull.size()] - hull[i] ).head<2>().norm(), 1e-4 ) << "edge " << i;
  std::vector<double> edges;
  EXPECT_NEAR( computeNormalizedEnergyStabilityMarginValue<double>( hull, edges, com ), 0.0281192, 1e-6 );
}

TEST( ConvexHull, mergeKeepsPolygonsWithoutCoincidentPoints )
{
  const Vector3dList points = { Vector3d( 0, 0, 0 ), Vector3d( 2, 0, 0 ), Vector3d( 2, 3, 0 ),
                                 Vector3d( 0, 3, 0 ), Vector3d( 1, 1, 0 ) };
  EXPECT_EQ( supportPolygonFromUnsortedContactPoints( points ),
             supportPolygonFromUnsortedContactPoints( points, 0.0, 0.0 ) );
}

TEST( ConvexHull, coincidentPointsMergeAtAnyCount )
{
  // Two points at one spot are one vertex, whatever their height.
  EXPECT_EQ( supportPolygonFromUnsortedContactPoints( Vector3dList{ Vector3d( 1, 1, 0 ), Vector3d( 1, 1 + 1e-6, 0.01 ) } ).size(),
             1U );
  // A square whose first corner is there twice.
  const Vector3dList square = { Vector3d( 0, 0, 0 ), Vector3d( 1, 0, 0 ), Vector3d( 1, 1, 0 ),
                                 Vector3d( 0, 1, 0 ), Vector3d( 1e-6, -1e-6, 0.005 ) };
  EXPECT_EQ( supportPolygonFromUnsortedContactPoints( square ).size(), 4U );
  // Zero keeps what the algorithm returns.
  EXPECT_EQ( supportPolygonFromUnsortedContactPoints( Vector3dList{ Vector3d( 1, 1, 0 ), Vector3d( 1, 1, 0 ) }, 0.0, 0.0 ).size(),
             2U );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
