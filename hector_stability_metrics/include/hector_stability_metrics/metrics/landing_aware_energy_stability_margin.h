// Copyright (c) 2026 Aljoscha Schmidt. Licensed under the MIT license. See LICENSE file in the
// project root for full license information.

#ifndef HECTOR_STABILITY_METRICS_METRICS_LANDING_AWARE_ENERGY_STABILITY_MARGIN_H
#define HECTOR_STABILITY_METRICS_METRICS_LANDING_AWARE_ENERGY_STABILITY_MARGIN_H

#include "hector_stability_metrics/math/types.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <utility>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace hector_stability_metrics
{
/*!
 * @defgroup LAESM Landing Aware Energy Stability Margin
 *
 * @brief A margin that forgives tipping over an edge when the robot lands on its tracks and
 * stops, and still flags tips that end in a failure or keep rolling.
 *
 * All energies are normalized by the weight, so they are heights in metres like the NESM. For a
 * support edge i let u be the energy toward the edge, the signed rotational kinetic energy about
 * it, and K_i its current value. The margin of the edge is the signed distance of K_i from the
 * set of u that end in a failure over edge i: positive by the work a push in either direction
 * needs to bring the robot into the set, negative by the work it would need to get out of it.
 *
 * - \f$h_i\f$ the energy barrier of edge i, usually its signed NESM,
 * - \f$K_i\f$ the signed rotational kinetic energy toward edge i, see edgeKineticEnergy,
 * - \f$D_i\f$ the drop of the centre of mass from the current pose to the landing pose,
 * - \f$\kappa_{ij}\f$ the fraction of the energy an inelastic landing carries into rotation
 *   about the new edge j, see impactTransfer,
 * - \f$h_j\f$ the energy needed after the impact to fail over edge j: its signed NESM in the
 *   landing pose, or the landingRequirement of a further landing when the caller continues the
 *   chain,
 * - \f$T_i = \min_j h_j / \kappa_{ij}\f$ the cheapest landing energy that fails, and
 *   \f$L_i\f$ the energy at which the robot leaves the pivot, see pivotLiftOffEnergy.
 *
 * A robot that gets over the top of edge i lands with \f$|u| + D_i\f$ measured from the
 * current pose. Before the top (\f$h_i \ge 0\f$) it fails for
 * \f$u \ge \max(h_i, \min(T_i - D_i, L_i))\f$. Past the top (\f$h_i < 0\f$) it fails for
 * \f$u \ge \max(0, \min(T_i - D_i, L_i))\f$, and for \f$h_i < u \le -(T_i - D_i)\f$: an
 * inward motion too weak to get back over the top turns and comes back with the same energy.
 * A landing past the lift off energy is not trusted and counts as a failure.
 * A turn that reaches a failure pose on its way, as a tilt past a limit, fails at the energy
 * that brings the robot there, \f$F_i\f$ (LandingEdge::failure_energy), also before the top:
 * the set then starts at \f$\min(\ldots, F_i)\f$.
 *
 * A landing that is itself a failure, no landing at all, or a landing without candidate edges
 * fails for every u past the top, so the margin is \f$e_i = h_i - K_i\f$. Without motion
 * toward the edge and before its top the margin is never below the NESM. The margin of the state
 * is the minimum over all edges. NaN in any input gives NaN.
 *
 * Geometry is the caller's: this header neither searches for the landing pose nor decides
 * whether a landing is a failure.
 *
 * Tipping axes follow the polygon convention of the other metrics. For a support polygon wound
 * clockwise seen from above, the edge from vertex k to vertex k+1 tips outward under a positive
 * rotation about the unit axis pointing from vertex k+1 to vertex k, see tippingAxis.
 *
 * @{
 */

namespace detail
{
template<typename T>
struct Identity {
  using type = T;
};
/// Keeps an argument out of template deduction, so a float margin takes a double literal.
template<typename Scalar>
using NonDeduced = typename Identity<Scalar>::type;
} // namespace detail

template<typename Scalar>
struct TippingAxis {
  math::Vector3<Scalar> point;     //!< A point on the axis.
  math::Vector3<Scalar> direction; //!< Unit direction, positive rotation tips outward.
};

/*!
 * @return The axis of support polygon edge @p index for a clockwise polygon.
 */
template<typename Scalar>
TippingAxis<Scalar> tippingAxis( const math::Vector3List<Scalar> &support_polygon, size_t index )
{
  const math::Vector3<Scalar> &a = support_polygon[index];
  const math::Vector3<Scalar> &b = support_polygon[( index + 1 ) % support_polygon.size()];
  return { a, ( a - b ).normalized() };
}

/*!
 * @return The moment of inertia about the axis, from the inertia @p inertia_com about the
 * centre of mass (world frame) and the parallel axis theorem. Zero only for a point mass on the
 * axis, for which the energy functions below return NaN.
 */
template<typename Scalar>
Scalar axisInertia( const math::Matrix3<Scalar> &inertia_com, Scalar mass,
                    const math::Vector3<Scalar> &center_of_mass, const TippingAxis<Scalar> &axis )
{
  const math::Vector3<Scalar> r = axis.direction.cross( center_of_mass - axis.point );
  return axis.direction.dot( inertia_com * axis.direction ) + mass * r.squaredNorm();
}

/*!
 * @brief Signed rotational kinetic energy toward the edge, normalized by the weight.
 *
 * The robot is assumed to pivot about the axis. Its angular momentum about the axis
 * \f$L = a^T(L_c + m (x - p) \times v)\f$ is kept by that constraint, so the rotation rate is
 * \f$L / I_a\f$ and the energy is \f$L^2 / (2 I_a)\f$. The sign is the sign of L: rotating away
 * from the edge returns a negative energy, which raises the margin.
 *
 * @param angular_momentum_com Total angular momentum about the centre of mass, world frame.
 * @param com_velocity Velocity of the centre of mass.
 * @param gravity Magnitude of the gravitational acceleration.
 */
template<typename Scalar>
Scalar edgeKineticEnergy( const math::Matrix3<Scalar> &inertia_com, Scalar mass,
                          const math::Vector3<Scalar> &center_of_mass,
                          const math::Vector3<Scalar> &angular_momentum_com,
                          const math::Vector3<Scalar> &com_velocity,
                          const TippingAxis<Scalar> &axis, Scalar gravity )
{
  const Scalar momentum =
      axis.direction.dot( angular_momentum_com +
                          mass * ( center_of_mass - axis.point ).cross( com_velocity ) );
  const Scalar inertia = axisInertia( inertia_com, mass, center_of_mass, axis );
  return std::copysign( momentum * momentum / ( 2 * inertia * mass * gravity ), momentum );
}

template<typename Scalar>
struct ImpactTransfer {
  //! Rotation rate about the new axis right after the impact, per unit rate about the old one.
  //! Positive when the robot keeps tipping over the new edge, negative for a rebound.
  Scalar lambda;
  //! Fraction of the kinetic energy carried into tipping over the new edge, zero for a rebound.
  Scalar kappa;
};

/*!
 * @brief What an inelastic landing does to a rotation about the old axis.
 *
 * At the impact the ground pushes only at points on the new axis, so the angular momentum about
 * it is conserved. For a unit rotation about the old axis the rate about the new one is
 * \f$\lambda = a_j^T [I_c a_i + m (x - p_j) \times (a_i \times (x - p_i))] / I_j\f$ and the
 * energy fraction is \f$\kappa = (I_j / I_i) \max(\lambda, 0)^2\f$. Both axes are oriented to tip
 * outward over their edge, and all quantities are taken in the landing pose.
 */
template<typename Scalar>
ImpactTransfer<Scalar> impactTransfer( const math::Matrix3<Scalar> &inertia_com, Scalar mass,
                                       const math::Vector3<Scalar> &center_of_mass,
                                       const TippingAxis<Scalar> &old_axis,
                                       const TippingAxis<Scalar> &new_axis )
{
  const math::Vector3<Scalar> &a_i = old_axis.direction;
  const math::Vector3<Scalar> &a_j = new_axis.direction;
  const math::Vector3<Scalar> com_velocity = a_i.cross( center_of_mass - old_axis.point );
  const Scalar momentum =
      a_j.dot( inertia_com * a_i + mass * ( center_of_mass - new_axis.point ).cross( com_velocity ) );
  const Scalar inertia_i = axisInertia( inertia_com, mass, center_of_mass, old_axis );
  const Scalar inertia_j = axisInertia( inertia_com, mass, center_of_mass, new_axis );
  const Scalar lambda = momentum / inertia_j;
  const Scalar forward = std::max( lambda, Scalar( 0 ) );
  return { lambda, inertia_j / inertia_i * forward * forward };
}

/*!
 * @brief The edges of the landing polygon the robot can keep tipping over.
 *
 * An inelastic, non slipping landing brings the new contacts to rest, so the robot can only
 * turn about an axis through them. With a point landing that is every edge touching a new
 * vertex. When the new vertices span more than @p line_threshold the landing is on a line, and
 * only edges with both ends new remain. Edges shorter than @p min_edge_length are skipped. A
 * polygon of two vertices has the two edges of the segment, one per tipping direction.
 *
 * @param landing_polygon The support polygon of the landing pose, clockwise seen from above.
 * @param is_new Per vertex, whether it belongs to the new contact rather than the old edge.
 * @return Indices of the candidate edges, edge k running from vertex k to vertex k+1.
 */
template<typename Scalar>
std::vector<size_t> landingCandidateEdges( const math::Vector3List<Scalar> &landing_polygon,
                                           const std::vector<bool> &is_new,
                                           detail::NonDeduced<Scalar> line_threshold,
                                           detail::NonDeduced<Scalar> min_edge_length = Scalar( 1e-6 ) )
{
  const size_t n = landing_polygon.size();
  if ( is_new.size() != n )
    throw std::invalid_argument( "landingCandidateEdges: is_new must have one entry per vertex" );
  Scalar spread = 0;
  for ( size_t k = 0; k < n; ++k ) {
    if ( !is_new[k] )
      continue;
    for ( size_t l = k + 1; l < n; ++l ) {
      if ( is_new[l] )
        spread = std::max( spread, ( landing_polygon[k] - landing_polygon[l] ).norm() );
    }
  }
  const bool line = spread > line_threshold;
  std::vector<size_t> candidates;
  if ( n < 2 )
    return candidates;
  for ( size_t k = 0; k < n; ++k ) {
    const size_t next = ( k + 1 ) % n;
    if ( ( landing_polygon[next] - landing_polygon[k] ).norm() < min_edge_length )
      continue;
    const int touches = int( is_new[k] ) + int( is_new[next] );
    if ( touches == 2 || ( !line && touches == 1 ) )
      candidates.push_back( k );
  }
  return candidates;
}

/*!
 * @brief The energy toward an edge at which its pivot unloads.
 *
 * The edge holds a turn only while the pull of gravity toward the axis covers the centripetal
 * acceleration of the centre of mass, \f$\omega^2 r \le g \cos\theta\f$ with r its distance from
 * the axis and \f$\theta\f$ its angle from straight above it. The kinetic energy at that rate is
 * \f$I_a \cos\theta / (2 m r)\f$ over m g. Beyond it the robot leaves the pivot, and a landing
 * found for a turn about the edge no longer describes what happens. World z is up.
 *
 * @return The energy over m g, zero with the centre of mass level with or below the axis, and
 * infinity with it on the axis.
 */
template<typename Scalar>
Scalar pivotLiftOffEnergy( const math::Matrix3<Scalar> &inertia_com, Scalar mass,
                           const math::Vector3<Scalar> &center_of_mass, const TippingAxis<Scalar> &axis )
{
  math::Vector3<Scalar> radial = center_of_mass - axis.point;
  radial -= radial.dot( axis.direction ) * axis.direction;
  const Scalar radius = radial.norm();
  if ( radius <= std::numeric_limits<Scalar>::epsilon() )
    return std::numeric_limits<Scalar>::infinity();
  const Scalar up = radial.z() / radius;
  if ( up <= 0 )
    return Scalar( 0 );
  return axisInertia( inertia_com, mass, center_of_mass, axis ) * up / ( 2 * mass * radius );
}

template<typename Scalar>
struct CandidateEdge {
  //! Energy needed right after the impact to fail over this edge, see the group description.
  Scalar hill_two;
  //! Energy fraction from impactTransfer. Values at or below zero stop the robot.
  Scalar kappa;
};

template<typename Scalar>
struct LandingEdge {
  Scalar hill_one;       //!< h_i, the barrier of the edge in the current pose
  Scalar kinetic_energy; //!< K_i, see edgeKineticEnergy
  Scalar drop = 0;       //!< D_i, centre of mass height now minus in the landing pose
  bool landed = false;   //!< false if the robot turns half a turn without touching anything
  bool failed = false;   //!< the landing pose is a failure in itself
  //! Energy toward the edge up to which the landing holds, see pivotLiftOffEnergy. What the
  //! landing forgives beyond it is not counted.
  Scalar landing_limit = std::numeric_limits<Scalar>::infinity();
  //! Energy toward the edge at which the turn reaches a failure pose on its way, before any
  //! landing, as a tilt past a limit: the rise of the centre of mass up to there. Infinity when
  //! the turn reaches none.
  Scalar failure_energy = std::numeric_limits<Scalar>::infinity();
  std::vector<CandidateEdge<Scalar>> candidates;
};

template<typename Scalar>
struct LandingMargin {
  Scalar value;
  //! Index into LandingEdge::candidates of the candidate that gave the value, -1 if no candidate
  //! took part or the cap decided.
  int candidate = -1;
};

/*!
 * The cheapest landing energy at which some candidate edge fails, and that candidate: E with
 * kappa_j E >= hill_two_j, minus infinity for a candidate that fails at any energy (kappa_j <= 0
 * and hill_two_j <= 0). Infinity and -1 when none can fail. NaN in a candidate gives NaN.
 */
template<typename Scalar>
std::pair<Scalar, int> landingThreshold( const LandingEdge<Scalar> &edge )
{
  Scalar threshold = std::numeric_limits<Scalar>::infinity();
  int candidate = -1;
  for ( size_t j = 0; j < edge.candidates.size(); ++j ) {
    const CandidateEdge<Scalar> &c = edge.candidates[j];
    if ( std::isnan( c.hill_two ) || std::isnan( c.kappa ) )
      return { std::numeric_limits<Scalar>::quiet_NaN(), static_cast<int>( j ) };
    Scalar t;
    if ( c.kappa > 0 )
      t = c.hill_two / c.kappa;
    else if ( c.hill_two <= 0 )
      t = -std::numeric_limits<Scalar>::infinity();
    else
      continue;
    if ( t < threshold ) {
      threshold = t;
      candidate = static_cast<int>( j );
    }
  }
  return { threshold, candidate };
}

/*!
 * @brief The margin of one edge, at most @p cap.
 *
 * The margin is the signed distance of the energy toward the edge, K, from the set of energies u
 * that end in a failure over this edge: positive by how much work a push in either direction
 * needs to bring the robot into that set, negative by how much it would need to get out of it.
 * With T the cheapest failing landing energy (landingThreshold), D the drop and L the lift off
 * energy, the set is
 * - before the top (h >= 0): u >= max(h, min(T - D, L)),
 * - past the top (h < 0): u >= max(0, min(T - D, L)), and h < u <= -(T - D), a motion inward too
 *   weak to get back over the top, which turns and comes back with the same energy.
 * Without a landing, with a failed one or one without candidates every u past the top fails, and
 * the margin is e = h - K.
 */
template<typename Scalar>
LandingMargin<Scalar> landingMargin( const LandingEdge<Scalar> &edge,
                                     detail::NonDeduced<Scalar> cap = std::numeric_limits<Scalar>::infinity() )
{
  constexpr Scalar nan = std::numeric_limits<Scalar>::quiet_NaN();
  const Scalar h = edge.hill_one, K = edge.kinetic_energy;
  if ( std::isnan( h ) || std::isnan( K ) || std::isnan( edge.failure_energy ) )
    return { nan, -1 };
  // A turn that reaches a failure pose on its way fails at that energy, also before the top.
  if ( edge.failure_energy < std::max( h, Scalar( 0 ) ) )
    return { std::min( edge.failure_energy - K, cap ), -1 };
  if ( !edge.landed || edge.failed || edge.candidates.empty() )
    return { std::min( h - K, cap ), -1 };
  if ( std::isnan( edge.drop ) || std::isnan( edge.landing_limit ) )
    return { nan, -1 };
  const auto [threshold, candidate] = landingThreshold( edge );
  if ( std::isnan( threshold ) )
    return { nan, candidate };
  const Scalar landing = threshold - edge.drop;
  const Scalar limit = std::min( edge.landing_limit, edge.failure_energy );
  const bool lift_off_decides = limit < landing;
  const Scalar forward = std::max( std::max( h, Scalar( 0 ) ), std::min( landing, limit ) );
  // Past the top, inward motion up to this fails too, if it does not reach back over the top.
  const Scalar inward = std::min( Scalar( 0 ), -landing );
  const bool has_inward = h < 0 && inward > h;
  // The two parts of the set touch when the landing fails at zero energy.
  const bool joined = has_inward && inward >= forward;
  Scalar value;
  if ( K >= forward ) {
    value = -( joined ? K - h : K - forward );
  } else if ( has_inward && K > h && K <= inward ) {
    // Out either back over the top or just past the inward part.
    value = -( joined ? K - h : std::min( K - h, inward - K ) );
  } else {
    value = forward - K;
    if ( has_inward )
      value = std::min( value, K > inward ? K - inward : h - K );
  }
  LandingMargin<Scalar> result{ value, lift_off_decides ? -1 : candidate };
  if ( result.value > cap )
    result = { cap, -1 };
  return result;
}

/*!
 * @brief The smallest energy toward the edge, forward only, with which the robot at rest ends in
 * a failure over it: what a landing that turns the robot about this edge has to bring.
 *
 * Inside a chain of landings the energy carried into the next turn points forward, so the inward
 * part of the failure set of landingMargin does not apply. Zero when the robot fails at rest.
 */
template<typename Scalar>
Scalar landingRequirement( const LandingEdge<Scalar> &edge )
{
  constexpr Scalar nan = std::numeric_limits<Scalar>::quiet_NaN();
  const Scalar h = edge.hill_one;
  if ( std::isnan( h ) || std::isnan( edge.failure_energy ) )
    return nan;
  const Scalar over = std::max( h, Scalar( 0 ) );
  if ( edge.failure_energy < over )
    return std::max( edge.failure_energy, Scalar( 0 ) );
  if ( !edge.landed || edge.failed || edge.candidates.empty() )
    return over;
  if ( std::isnan( edge.drop ) || std::isnan( edge.landing_limit ) )
    return nan;
  const Scalar threshold = landingThreshold( edge ).first;
  if ( std::isnan( threshold ) )
    return nan;
  return std::max( over, std::min( threshold - edge.drop, std::min( edge.landing_limit, edge.failure_energy ) ) );
}

/*!
 * @brief The margin of every edge.
 * @param edge_stabilities Output, one value per edge. Existing content is erased.
 */
template<typename Scalar>
void computeLandingAwareEnergyStabilityMargin(
    const std::vector<LandingEdge<Scalar>> &edges, std::vector<Scalar> &edge_stabilities,
    detail::NonDeduced<Scalar> cap = std::numeric_limits<Scalar>::infinity() )
{
  edge_stabilities.resize( edges.size() );
  for ( size_t i = 0; i < edges.size(); ++i ) edge_stabilities[i] = landingMargin( edges[i], cap ).value;
}

/*!
 * @return The index of the least stable edge, the first NaN edge if there is one, and 0 for no
 * edges.
 */
template<typename Scalar>
size_t computeLandingAwareEnergyStabilityMarginLeastStableEdgeIndex(
    const std::vector<LandingEdge<Scalar>> &edges, std::vector<Scalar> &edge_stabilities,
    detail::NonDeduced<Scalar> cap = std::numeric_limits<Scalar>::infinity() )
{
  computeLandingAwareEnergyStabilityMargin( edges, edge_stabilities, cap );
  size_t least = 0;
  for ( size_t i = 0; i < edge_stabilities.size(); ++i ) {
    if ( std::isnan( edge_stabilities[i] ) )
      return i;
    if ( edge_stabilities[i] < edge_stabilities[least] )
      least = i;
  }
  return least;
}

/*!
 * @return The minimum over the edges, NaN if there are none or any edge is NaN.
 */
template<typename Scalar>
Scalar computeLandingAwareEnergyStabilityMarginValue(
    const std::vector<LandingEdge<Scalar>> &edges, std::vector<Scalar> &edge_stabilities,
    detail::NonDeduced<Scalar> cap = std::numeric_limits<Scalar>::infinity() )
{
  if ( edges.empty() ) {
    edge_stabilities.clear();
    return std::numeric_limits<Scalar>::quiet_NaN();
  }
  return edge_stabilities[computeLandingAwareEnergyStabilityMarginLeastStableEdgeIndex(
      edges, edge_stabilities, cap )];
}

/*! @} */
} // namespace hector_stability_metrics

#endif // HECTOR_STABILITY_METRICS_METRICS_LANDING_AWARE_ENERGY_STABILITY_MARGIN_H
