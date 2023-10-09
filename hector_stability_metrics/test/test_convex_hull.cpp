// Copyright (c) 2023 Stefan Fabian, Martin Oehler, Felix Biemüller. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "eigen_tests.h"
#include <hector_stability_metrics/math/support_polygon.h>

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

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
