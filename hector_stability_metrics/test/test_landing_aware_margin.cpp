// Copyright (c) 2026 Aljoscha Schmidt. Licensed under the MIT license. See LICENSE file in the
// project root for full license information.

#include <hector_stability_metrics/metrics/landing_aware_energy_stability_margin.h>

#include "eigen_tests.h"
#include <hector_stability_metrics/metrics/normalized_energy_stability_margin.h>

#include <Eigen/Dense>
#include <random>

using namespace hector_stability_metrics;
using namespace hector_stability_metrics::math;
using namespace Eigen;

namespace
{
constexpr double kTol = 1e-9;
constexpr double kInf = std::numeric_limits<double>::infinity();

Vector3dList square()
{
  return { Vector3d( 0, 0, 0 ), Vector3d( 0, 1, 0 ), Vector3d( 1, 1, 0 ), Vector3d( 1, 0, 0 ) };
}

Vector3d rotate( const TippingAxis<double> &axis, double angle, const Vector3d &p )
{
  return axis.point + AngleAxisd( angle, axis.direction ) * ( p - axis.point );
}
} // namespace

TEST( LandingAwareMargin, TippingAxisTurnsOutward )
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

TEST( LandingAwareMargin, AxisInertiaParallelAxis )
{
  const Matrix3d inertia = Vector3d( 0.1, 0.2, 0.3 ).asDiagonal();
  const TippingAxis<double> axis{ Vector3d( 0, 0, 0 ), Vector3d::UnitX() };
  EXPECT_NEAR( axisInertia( inertia, 2.0, Vector3d( 5, 0, 1 ), axis ), 0.1 + 2.0, kTol );
}

TEST( LandingAwareMargin, KineticEnergyOfAPointMassPivot )
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

TEST( LandingAwareMargin, KineticEnergyCountsSpinAboutTheCom )
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

TEST( LandingAwareMargin, ImpactTransferSameAxisKeepsEverything )
{
  const Matrix3d inertia = Vector3d( 0.4, 0.7, 0.9 ).asDiagonal();
  const TippingAxis<double> axis{ Vector3d( 0.1, 0.2, 0 ), Vector3d( 1, 1, 0 ).normalized() };
  const ImpactTransfer<double> t = impactTransfer( inertia, 3.0, Vector3d( 0.5, -0.2, 0.4 ), axis, axis );
  EXPECT_NEAR( t.lambda, 1.0, kTol );
  EXPECT_NEAR( t.kappa, 1.0, kTol );
}

TEST( LandingAwareMargin, ImpactTransferMatchesAnImpulseSolve )
{
  // Solve the impact directly: two contact points on the new axis come to rest, the impulses
  // at them change the momentum and the angular momentum about the com.
  std::mt19937 rng( 3 );
  std::uniform_real_distribution<double> u( -1, 1 );
  auto vec = [&] { return Vector3d( u( rng ), u( rng ), u( rng ) ); };
  for ( int trial = 0; trial < 200; ++trial ) {
    const double mass = 1 + 5 * std::abs( u( rng ) );
    const Matrix3d root = Matrix3d::Random();
    const Matrix3d inertia = root * root.transpose() + 0.05 * Matrix3d::Identity();
    const Vector3d com = vec();
    const TippingAxis<double> old_axis{ vec(), vec().normalized() };
    const TippingAxis<double> new_axis{ vec(), vec().normalized() };
    const Vector3d omega = old_axis.direction;
    const Vector3d velocity = omega.cross( com - old_axis.point );
    const Vector3d q1 = new_axis.point, q2 = new_axis.point + 0.5 * new_axis.direction;
    auto skew = []( const Vector3d &v ) {
      Matrix3d m;
      m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
      return m;
    };
    // Unknowns: v', w', P1, P2.
    Matrix<double, 12, 12> a = Matrix<double, 12, 12>::Zero();
    Matrix<double, 12, 1> b = Matrix<double, 12, 1>::Zero();
    a.block<3, 3>( 0, 0 ) = Matrix3d::Identity();
    a.block<3, 3>( 0, 3 ) = -skew( q1 - com );
    a.block<3, 3>( 3, 0 ) = Matrix3d::Identity();
    a.block<3, 3>( 3, 3 ) = -skew( q2 - com );
    a.block<3, 3>( 6, 0 ) = mass * Matrix3d::Identity();
    a.block<3, 3>( 6, 6 ) = -Matrix3d::Identity();
    a.block<3, 3>( 6, 9 ) = -Matrix3d::Identity();
    b.segment<3>( 6 ) = mass * velocity;
    a.block<3, 3>( 9, 3 ) = inertia;
    a.block<3, 3>( 9, 6 ) = -skew( q1 - com );
    a.block<3, 3>( 9, 9 ) = -skew( q2 - com );
    b.segment<3>( 9 ) = inertia * omega;
    const Matrix<double, 12, 1> x = a.completeOrthogonalDecomposition().solve( b );
    const double lambda = new_axis.direction.dot( x.segment<3>( 3 ) );
    const ImpactTransfer<double> t = impactTransfer( inertia, mass, com, old_axis, new_axis );
    ASSERT_NEAR( t.lambda, lambda, 1e-9 ) << "trial " << trial;
    const double energy_before = 0.5 * ( mass * velocity.squaredNorm() + omega.dot( inertia * omega ) );
    const Vector3d v_after = x.head<3>(), w_after = x.segment<3>( 3 );
    const double energy_after =
        0.5 * ( mass * v_after.squaredNorm() + w_after.dot( inertia * w_after ) );
    if ( lambda > 0 )
      ASSERT_NEAR( t.kappa, energy_after / energy_before, 1e-9 ) << "trial " << trial;
    else
      ASSERT_EQ( t.kappa, 0.0 );
    ASSERT_LE( t.kappa, 1.0 + 1e-12 );
  }
}

TEST( LandingAwareMargin, ImpactTransferRebound )
{
  // A low wide body landing flat: the com lies between the axes, the landing reverses it.
  const Matrix3d inertia = Matrix3d::Identity() * 0.01;
  const Vector3d com( 0.3, 0, 0.05 );
  const TippingAxis<double> old_axis{ Vector3d( 0, 0, 0 ), Vector3d::UnitY() };
  const TippingAxis<double> new_axis{ Vector3d( 0.6, 0, 0 ), Vector3d::UnitY() };
  const ImpactTransfer<double> t = impactTransfer( inertia, 1.0, com, old_axis, new_axis );
  EXPECT_LT( t.lambda, 0.0 );
  EXPECT_EQ( t.kappa, 0.0 );
}

TEST( LandingAwareMargin, CandidateEdgesPointLanding )
{
  const Vector3dList polygon = { Vector3d( 0, 0, 0 ), Vector3d( 0, 1, 0 ), Vector3d( 1, 0.5, 0 ) };
  const std::vector<size_t> candidates =
      landingCandidateEdges( polygon, { false, false, true }, 0.05 );
  EXPECT_EQ( candidates, ( std::vector<size_t>{ 1, 2 } ) );
}

TEST( LandingAwareMargin, CandidateEdgesLineLanding )
{
  const Vector3dList polygon = square();
  const std::vector<size_t> candidates =
      landingCandidateEdges( polygon, { false, false, true, true }, 0.05 );
  EXPECT_EQ( candidates, ( std::vector<size_t>{ 2 } ) );
}

TEST( LandingAwareMargin, CandidateEdgesShortLineIsAPoint )
{
  const Vector3dList polygon = { Vector3d( 0, 0, 0 ), Vector3d( 0, 1, 0 ), Vector3d( 1, 0.51, 0 ),
                                 Vector3d( 1, 0.49, 0 ) };
  const std::vector<size_t> candidates =
      landingCandidateEdges( polygon, { false, false, true, true }, 0.05 );
  EXPECT_EQ( candidates, ( std::vector<size_t>{ 1, 2, 3 } ) );
}

TEST( LandingAwareMargin, MarginBranches )
{
  LandingEdge<double> edge;
  edge.hill_one = 0.05;
  edge.kinetic_energy = 0.01;
  edge.drop = 0.03;
  // No landing, failed landing: the ordinary dynamic margin.
  EXPECT_NEAR( landingMargin( edge ).value, 0.04, kTol );
  edge.landed = true;
  edge.failed = true;
  edge.candidates = { { 0.1, 0.5 } };
  EXPECT_NEAR( landingMargin( edge ).value, 0.04, kTol );
  edge.failed = false;
  // Hill two decides: 0.1 / 0.5 - 0.01 - 0.03.
  LandingMargin<double> m = landingMargin( edge );
  EXPECT_NEAR( m.value, 0.16, kTol );
  EXPECT_EQ( m.candidate, 0 );
  // Hill one decides when hill two is cheap.
  edge.candidates = { { 0.01, 0.9 } };
  EXPECT_NEAR( landingMargin( edge ).value, 0.04, kTol );
  // The worse candidate decides.
  edge.candidates = { { 0.1, 0.5 }, { 0.06, 0.5 } };
  m = landingMargin( edge );
  EXPECT_NEAR( m.value, 0.08, kTol );
  EXPECT_EQ( m.candidate, 1 );
  // A landing that stops the robot on a stable edge never fails over it.
  edge.candidates = { { 0.1, 0.0 } };
  EXPECT_EQ( landingMargin( edge ).value, kInf );
  EXPECT_NEAR( landingMargin( edge, 0.25 ).value, 0.25, kTol );
  EXPECT_EQ( landingMargin( edge, 0.25 ).candidate, -1 );
  // A landing on an edge that is unstable by itself fails once hill one is passed.
  edge.candidates = { { -0.01, 0.0 } };
  EXPECT_NEAR( landingMargin( edge ).value, 0.04, kTol );
  // A landing without candidate edges is not trusted to stop the robot.
  edge.candidates.clear();
  EXPECT_NEAR( landingMargin( edge ).value, 0.04, kTol );
}

TEST( LandingAwareMargin, TippingAxisOnAnIrregularTiltedPolygon )
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

TEST( LandingAwareMargin, PastTheTopSwingsBackThrough )
{
  // Past the top and swinging inward too slowly to get back over: it turns and lands with
  // -h_i + D, which is enough to roll over the next edge.
  LandingEdge<double> edge;
  edge.hill_one = -0.05;
  edge.kinetic_energy = -0.02;
  edge.drop = 0.1;
  edge.landed = true;
  edge.candidates = { { 0.05, 0.5 } };
  EXPECT_NEAR( landingMargin( edge ).value, -0.03, kTol );
  // Past the top and falling outward: the same, the smallest failing push is e.
  edge.kinetic_energy = 0.02;
  EXPECT_NEAR( landingMargin( edge ).value, -0.07, kTol );
  // The landing no longer carries it over: the second hill decides.
  edge.candidates = { { 0.2, 0.5 } };
  EXPECT_NEAR( landingMargin( edge ).value, 0.2 / 0.5 - 0.02 - 0.1, kTol );
}

TEST( LandingAwareMargin, PastTheTopAndLandingOnTheTracksIsNotAFailure )
{
  // At rest, the centre of mass 5 cm of energy past edge i. Without a push it lands with
  // D = 0.10, keeps kappa * 0.10 = 0.05 < 0.06 after the impact and stops on its tracks: no
  // failure. Pushed outward by 0.02 it lands with 0.12 and goes over, pushed inward by 0.02 it
  // stops short of the top, turns and comes back with the same 0.12.
  LandingEdge<double> edge;
  edge.hill_one = -0.05;
  edge.kinetic_energy = 0;
  edge.drop = 0.1;
  edge.landed = true;
  edge.candidates = { { 0.06, 0.5 } };
  EXPECT_NEAR( landingMargin( edge ).value, 0.02, kTol );
  // Already moving inward with 0.02 it fails as it is: it needs 0.02 of work to get out, either
  // back over the top (0.03) or to below the failing energy (0.02).
  edge.kinetic_energy = -0.02;
  EXPECT_NEAR( landingMargin( edge ).value, 0.0, kTol );
  edge.kinetic_energy = -0.03;
  EXPECT_NEAR( landingMargin( edge ).value, -0.01, kTol );
  // Inward fast enough to get back over the top: it leaves the edge behind.
  edge.kinetic_energy = -0.06;
  EXPECT_NEAR( landingMargin( edge ).value, 0.01, kTol );
  // Coming back with 0.015 it would leave the pivot (lift off 0.015) before the landing: an inward
  // motion of 0.015 fails as well as an outward one.
  edge.landing_limit = 0.015;
  edge.kinetic_energy = -0.016;
  EXPECT_NEAR( landingMargin( edge ).value, -0.001, kTol );
}

TEST( LandingAwareMargin, TheRequirementOfALandingPointsForward )
{
  // Inside a chain the energy carried into the next turn points forward.
  LandingEdge<double> edge;
  edge.hill_one = 0.03;
  edge.kinetic_energy = 0;
  // Not landed: getting over the top fails.
  EXPECT_NEAR( landingRequirement( edge ), 0.03, kTol );
  edge.hill_one = -0.02;
  EXPECT_NEAR( landingRequirement( edge ), 0.0, kTol );
  // Landed, past the top: forward energy plus the drop has to carry over the next edge.
  edge.landed = true;
  edge.drop = 0.04;
  edge.candidates = { { 0.05, 0.5 } };
  EXPECT_NEAR( landingRequirement( edge ), 0.1 - 0.04, kTol );
  // The margin from rest would also count an inward motion, which a landing cannot bring.
  EXPECT_NEAR( landingMargin( edge ).value, 0.06, kTol );
  // A landing that fails at rest needs nothing more.
  edge.candidates = { { 0.01, 0.5 } };
  EXPECT_NEAR( landingRequirement( edge ), 0.0, kTol );
  EXPECT_LT( landingMargin( edge ).value, 0 );
  // Before the top the barrier counts first.
  edge.hill_one = 0.08;
  EXPECT_NEAR( landingRequirement( edge ), 0.08, kTol );
  edge.landing_limit = 0.05;
  edge.candidates = { { 0.5, 0.5 } };
  EXPECT_NEAR( landingRequirement( edge ), 0.08, kTol );
}

TEST( LandingAwareMargin, AFailurePoseOnTheWayDecides )
{
  // Leaning back steeply, the turn passes the tilt limit after a rise of 0.01, before the top
  // of the edge at 0.05: that rise is what fails, whatever the landing would be.
  LandingEdge<double> edge;
  edge.hill_one = 0.05;
  edge.kinetic_energy = 0;
  edge.drop = 0.02;
  edge.landed = true;
  edge.candidates = { { 0.2, 0.5 } };
  EXPECT_NEAR( landingMargin( edge ).value, 0.38, kTol );
  edge.failure_energy = 0.01;
  EXPECT_NEAR( landingMargin( edge ).value, 0.01, kTol );
  EXPECT_NEAR( landingRequirement( edge ), 0.01, kTol );
  edge.kinetic_energy = 0.03;
  EXPECT_NEAR( landingMargin( edge ).value, -0.02, kTol );
  // Past the top it caps the landing like the lift off energy.
  edge.kinetic_energy = 0;
  edge.failure_energy = 0.2;
  EXPECT_NEAR( landingMargin( edge ).value, 0.2, kTol );
  edge.hill_one = -0.01;
  EXPECT_NEAR( landingMargin( edge ).value, 0.2, kTol );
}

TEST( LandingAwareMargin, NanPropagates )
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  LandingEdge<double> edge;
  edge.hill_one = 0.05;
  edge.kinetic_energy = 0;
  edge.landed = true;
  edge.candidates = { { 0.1, 0.5 } };
  for ( int field = 0; field < 5; ++field ) {
    LandingEdge<double> e = edge;
    switch ( field ) {
    case 0: e.hill_one = nan; break;
    case 1: e.kinetic_energy = nan; break;
    case 2: e.drop = nan; break;
    case 3: e.candidates[0].hill_two = nan; break;
    case 4: e.candidates[0].kappa = nan; break;
    }
    EXPECT_TRUE( std::isnan( landingMargin( e, 0.25 ).value ) ) << "field " << field;
  }
  std::vector<LandingEdge<double>> edges( 3, edge );
  edges[1].hill_one = nan;
  edges[2].hill_one = -1;
  std::vector<double> values;
  EXPECT_TRUE( std::isnan( computeLandingAwareEnergyStabilityMarginValue( edges, values ) ) );
  EXPECT_EQ( computeLandingAwareEnergyStabilityMarginLeastStableEdgeIndex( edges, values ), 1u );
}

TEST( LandingAwareMargin, CandidateEdgesDegenerate )
{
  const Vector3dList duplicated = { Vector3d( 0, 0, 0 ), Vector3d( 0, 1, 0 ), Vector3d( 1, 0.5, 0 ),
                                    Vector3d( 1, 0.5, 0 ) };
  EXPECT_EQ( landingCandidateEdges( duplicated, { false, false, true, true }, 0.05 ),
             ( std::vector<size_t>{ 1, 3 } ) );
  EXPECT_TRUE( landingCandidateEdges( Vector3dList{}, {}, 0.05 ).empty() );
  EXPECT_THROW( landingCandidateEdges( duplicated, { true }, 0.05 ), std::invalid_argument );
  const Vector3dList segment = { Vector3d( 0, 0, 0 ), Vector3d( 0, 1, 0 ) };
  EXPECT_EQ( landingCandidateEdges( segment, { false, true }, 0.05 ),
             ( std::vector<size_t>{ 0, 1 } ) );
}

TEST( LandingAwareMargin, FloatInstantiation )
{
  const Vector3fList polygon = { Vector3f( 0, 0, 0 ), Vector3f( 0, 1, 0 ), Vector3f( 1, 0.5f, 0 ) };
  EXPECT_EQ( landingCandidateEdges( polygon, { false, false, true }, 0.05 ).size(), 2u );
  LandingEdge<float> edge;
  edge.hill_one = 0.1f;
  edge.kinetic_energy = 0;
  std::vector<LandingEdge<float>> edges{ edge };
  std::vector<float> values;
  EXPECT_NEAR( landingMargin( edge, 0.05 ).value, 0.05f, 1e-7 );
  EXPECT_NEAR( computeLandingAwareEnergyStabilityMarginValue( edges, values, 0.25 ), 0.1f, 1e-7 );
}

TEST( LandingAwareMargin, LiftOffOfAPointMass )
{
  // For a point mass I_a = m r², so the pivot unloads at r cos(theta) / 2.
  const TippingAxis<double> axis{ Vector3d( 0, 0, 0 ), Vector3d::UnitY() };
  const double mass = 3;
  const Vector3d above( 0, 0, 0.4 ), aside( 0.3, 0, 0.4 ), level( 0.5, 0, 0 );
  EXPECT_NEAR( pivotLiftOffEnergy<double>( Matrix3d::Zero(), mass, above, axis ), 0.2, kTol );
  EXPECT_NEAR( pivotLiftOffEnergy<double>( Matrix3d::Zero(), mass, aside, axis ), 0.5 * 0.4 / 0.5 * 0.5, kTol );
  EXPECT_EQ( pivotLiftOffEnergy<double>( Matrix3d::Zero(), mass, level, axis ), 0.0 );
  EXPECT_EQ( pivotLiftOffEnergy<double>( Matrix3d::Zero(), mass, Vector3d( 0, 5, 0 ), axis ),
             std::numeric_limits<double>::infinity() );
  // Inertia about the com adds to the energy the pivot holds.
  EXPECT_NEAR( pivotLiftOffEnergy<double>( Matrix3d::Identity() * 0.12, mass, above, axis ),
               ( 0.12 + mass * 0.16 ) / ( 2 * mass * 0.4 ), kTol );
}

TEST( LandingAwareMargin, TheLandingCountsOnlyUpToLiftOff )
{
  LandingEdge<double> edge;
  edge.hill_one = 0.02;
  edge.kinetic_energy = 0.01;
  edge.drop = 0.0;
  edge.landed = true;
  edge.candidates = { { 0.2, 0.5 } };
  EXPECT_NEAR( landingMargin( edge ).value, 0.4 - 0.01, kTol );
  edge.landing_limit = 0.15;
  const LandingMargin<double> limited = landingMargin( edge );
  EXPECT_NEAR( limited.value, 0.15 - 0.01, kTol );
  EXPECT_EQ( limited.candidate, -1 );
  // Never below hill one, since getting over the edge takes that much whatever follows.
  edge.landing_limit = 0.0;
  EXPECT_NEAR( landingMargin( edge ).value, 0.01, kTol );
  edge.landing_limit = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE( std::isnan( landingMargin( edge ).value ) );
}

TEST( LandingAwareMargin, ForwardOffAStep )
{
  // Prototype numbers for a tracked robot's com 3 cm before a 0.2 m step: NESM 0.002, the
  // landing on the lower level drops the com 3 mm and gives h_j 0.170 at kappa 0.340.
  LandingEdge<double> edge;
  edge.hill_one = 0.002;
  edge.kinetic_energy = 0;
  edge.drop = 0.003;
  edge.landed = true;
  edge.candidates = { { 0.170, 0.340 } };
  EXPECT_NEAR( landingMargin( edge ).value, 0.170 / 0.340 - 0.003, kTol );
}

TEST( LandingAwareMargin, MinimumOverEdges )
{
  std::vector<LandingEdge<double>> edges( 3 );
  edges[0].hill_one = 0.2;
  edges[1].hill_one = 0.1;
  edges[2].hill_one = 0.3;
  std::vector<double> values;
  EXPECT_NEAR( computeLandingAwareEnergyStabilityMarginValue( edges, values, 0.25 ), 0.1, kTol );
  EXPECT_EQ( values.size(), 3u );
  EXPECT_NEAR( values[2], 0.25, kTol );
  std::vector<LandingEdge<double>> none;
  EXPECT_TRUE( std::isnan( computeLandingAwareEnergyStabilityMarginValue( none, values ) ) );
}
