// Copyright (c) 2026 Aljoscha Schmidt. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// Every header compiles when it is included first.
#include <hector_stability_metrics/math/hull.h>
#include <hector_stability_metrics/math/support_polygon.h>
#include <hector_stability_metrics/metrics/landing_aware_energy_stability_margin.h>
#include <hector_stability_metrics/metrics/normalized_energy_stability_margin.h>

#include <gtest/gtest.h>

TEST( Headers, compileWhenIncludedFirst ) { SUCCEED(); }

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
