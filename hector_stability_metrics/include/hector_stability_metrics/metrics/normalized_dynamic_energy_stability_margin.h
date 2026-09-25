// Copyright (c) 2026 Aljoscha Schmidt. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_STABILITY_METRICS_METRICS_NORMALIZED_DYNAMIC_ENERGY_STABILITY_MARGIN_H
#define HECTOR_STABILITY_METRICS_METRICS_NORMALIZED_DYNAMIC_ENERGY_STABILITY_MARGIN_H

#include "hector_stability_metrics/math/sign_functions.h"
#include "hector_stability_metrics/math/support_polygon.h"
#include "hector_stability_metrics/metrics/common.h"

namespace hector_stability_metrics
{
/*!
 * @defgroup NDESM Normalized Dynamic Energy Stability Margin
 *
 * @brief Computes the Normalized Dynamic Energy Stability Margin, the dynamic
 * extension of the NESM described in "An improved energy stability margin for
 * walking machines subject to dynamic effects" (Garcia, De Santos, 2005).
 *
 * Where the NESM is the height the centre of mass must be raised to reach the
 * unstable equilibrium above a tip-over edge, the NDESM is the work done
 * against the full resultant force while virtually rotating the system into
 * that configuration, normalised by the weight so that it again has units of
 * length. The resultant is passed in as a normalised acceleration
 *
 *   \f$\hat{f} = (\mathbf{g} - \ddot{x}_{CoM}) / \lVert \mathbf{g} \rVert\f$,
 *
 * which is dimensionless and equals \f$(0,0,-1)\f$ in the quasi-static case.
 *
 * ### Derivation
 *
 * For edge \f$i\f$ let \f$P\f$ be the point on the edge closest to the centre
 * of mass, \f$\mathbf{R} = x_{CoM} - P\f$, and work in the tipping plane normal
 * to the edge direction \f$\hat{e}\f$. That plane is spanned by
 *
 *   - \f$\hat{z}_p\f$, the vertical projected into the plane and normalised,
 *   - \f$\hat{n}\f$, the normal of the vertical plane through the edge,
 *     \f$\hat{e} \times \hat{z}\f$ normalised; \f$\theta\f$ is negative once the
 *     CoM has crossed that plane.
 *
 * With \f$\mathbf{R} = R(\cos\theta\,\hat{z}_p + \sin\theta\,\hat{n})\f$ the CoM
 * follows \f$p(\varphi) = P + R(\cos\varphi\,\hat{z}_p +
 * \sin\varphi\,\hat{n})\f$ from \f$\varphi = \theta\f$ to \f$0\f$, and the work
 * done against a constant \f$\hat{f}\f$ integrates to
 *
 *   \f$\beta_i = R\left[-(\hat{f}\cdot\hat{z}_p)(1-\cos\theta)
 *                       + (\hat{f}\cdot\hat{n})\sin\theta\right].\f$
 *
 * In the quasi-static case \f$\hat{f} = (0,0,-1)\f$: the second term vanishes
 * because \f$\hat{n} \perp \hat{z}\f$ by construction, and
 * \f$\hat{f}\cdot\hat{z}_p = -\cos\psi\f$ with \f$\psi\f$ the edge inclination,
 * so the expression reduces to \f$R(1-\cos\theta)\cos\psi\f$, the NESM as
 * implemented in normalized_energy_stability_margin.h.
 *
 * @tparam Scalar The scalar type used for computations.
 * @tparam signFunction A function returning the sign for the passed value. See
 * math/sign_functions.h. Use hector_stability_metrics::math::constantOneSignum
 * for the original unsigned formulation.
 *
 * @param support_polygon A support polygon where it is assumed that the points
 * are ordered clockwise when viewed from above.
 * @param edge_stabilities Output vector for the stability value of each edge.
 * Existing content is erased.
 * @param center_of_mass A Vector3<Scalar> representing the center of mass.
 * @param normalized_resultant \f$(\mathbf{g} - \ddot{x}_{CoM}) /
 * \lVert\mathbf{g}\rVert\f$. Pass \f$(0,0,-1)\f$ to recover the NESM.
 * @param normalization_factor Optional Scalar factor applied to all edges.
 *
 * @{
 */

template<typename Scalar, math::SignFunction<Scalar> signFunction = math::quickSignum>
void computeNormalizedDynamicEnergyStabilityMargin(
    const math::Vector3List<Scalar> &support_polygon, std::vector<Scalar> &edge_stabilities,
    const math::Vector3<Scalar> &center_of_mass,
    const math::Vector3<Scalar> &normalized_resultant,
    Scalar normalization_factor = Scalar( 1 ) );

/*!
 * @return The minimum stability value of all edges.
 */
template<typename Scalar, math::SignFunction<Scalar> signFunction = math::quickSignum>
Scalar computeNormalizedDynamicEnergyStabilityMarginValue(
    const math::Vector3List<Scalar> &support_polygon, std::vector<Scalar> &edge_stabilities,
    const math::Vector3<Scalar> &center_of_mass,
    const math::Vector3<Scalar> &normalized_resultant,
    Scalar normalization_factor = Scalar( 1 ) );

/*!
 * @return The index of the minimum stability value of all edges.
 */
template<typename Scalar, math::SignFunction<Scalar> signFunction = math::quickSignum>
size_t computeNormalizedDynamicEnergyStabilityMarginLeastStableEdgeIndex(
    const math::Vector3List<Scalar> &support_polygon, std::vector<Scalar> &edge_stabilities,
    const math::Vector3<Scalar> &center_of_mass,
    const math::Vector3<Scalar> &normalized_resultant,
    Scalar normalization_factor = Scalar( 1 ) );

/*! @} */

namespace impl
{
template<typename Scalar, math::SignFunction<Scalar> signFunction = math::quickSignum,
         typename MinimumType = Scalar,
         typename MinimumSelector = math::MinimumSelector<Scalar, MinimumType>>
typename MinimumSelector::ReturnType computeNormalizedDynamicEnergyStabilityMargin(
    const math::Vector3List<Scalar> &support_polygon, std::vector<Scalar> &edge_stabilities,
    const math::Vector3<Scalar> &center_of_mass,
    const math::Vector3<Scalar> &normalized_resultant,
    Scalar normalization_factor = Scalar( 1 ) )
{
  const size_t number_of_edges = support_polygon.size();
  edge_stabilities.resize( number_of_edges );
  MinimumSelector minimum_selector;

  const math::Vector3<Scalar> vertical( 0, 0, 1 );

  for ( size_t i = 0; i < number_of_edges; ++i ) {
    const math::Vector3<Scalar> &A = support_polygon[i];
    const math::Vector3<Scalar> &edge = math::getSupportPolygonEdge( support_polygon, i );
    const math::Vector3<Scalar> &corner_to_com = center_of_mass - A;

    // closest point on the edge line to the com (Ericson, Real Time Collision Detection), so that R
    // is perpendicular to the edge
    const math::Vector3<Scalar> &closest_point_on_edge_to_com =
        A + corner_to_com.dot( edge ) / edge.dot( edge ) * edge;

    // normal of the vertical plane containing the edge, normalized by hand as Eigen's normalization
    // is not usable with automatic differentiation
    const math::Vector3<Scalar> &plane_normal = edge.cross( vertical );
    const math::Vector3<Scalar> &plane_normal_normalized = plane_normal / plane_normal.norm();
    const Scalar distance_com_to_plane = plane_normal_normalized.dot( corner_to_com );

    const math::Vector3<Scalar> &R = center_of_mass - closest_point_on_edge_to_com;
    const Scalar R_norm = R.norm();

    // rotation around the edge necessary to turn the com into the vertical plane of the edge,
    // negative once the com has crossed it
    const Scalar theta = asin( distance_com_to_plane / R_norm );

    // z_p is the vertical projected into the tipping plane (its norm before normalization is
    // cos(psi))
    const math::Vector3<Scalar> &edge_unit = edge / edge.norm();
    const math::Vector3<Scalar> &z_in_plane = vertical - edge_unit.dot( vertical ) * edge_unit;
    const math::Vector3<Scalar> &z_p = z_in_plane / z_in_plane.norm();

    // work against the resultant while rotating the com to the unstable equilibrium, normalized by
    // the weight
    const Scalar lift_term = -normalized_resultant.dot( z_p ) * ( 1 - cos( theta ) );
    const Scalar inertial_term = normalized_resultant.dot( plane_normal_normalized ) * sin( theta );

    const Scalar value = normalization_factor * R_norm * ( lift_term + inertial_term ) *
                         signFunction( theta );
    minimum_selector.updateMinimum( i, value );
    edge_stabilities[i] = value;
  }
  return minimum_selector.getMinimum();
}
} // namespace impl

template<typename Scalar, math::SignFunction<Scalar> signFunction>
void computeNormalizedDynamicEnergyStabilityMargin(
    const math::Vector3List<Scalar> &support_polygon, std::vector<Scalar> &edge_stabilities,
    const math::Vector3<Scalar> &center_of_mass,
    const math::Vector3<Scalar> &normalized_resultant, Scalar normalization_factor )
{
  impl::computeNormalizedDynamicEnergyStabilityMargin<Scalar, signFunction, void>(
      support_polygon, edge_stabilities, center_of_mass, normalized_resultant,
      normalization_factor );
}

template<typename Scalar, math::SignFunction<Scalar> signFunction>
Scalar computeNormalizedDynamicEnergyStabilityMarginValue(
    const math::Vector3List<Scalar> &support_polygon, std::vector<Scalar> &edge_stabilities,
    const math::Vector3<Scalar> &center_of_mass,
    const math::Vector3<Scalar> &normalized_resultant, Scalar normalization_factor )
{
  return impl::computeNormalizedDynamicEnergyStabilityMargin<Scalar, signFunction>(
      support_polygon, edge_stabilities, center_of_mass, normalized_resultant,
      normalization_factor );
}

template<typename Scalar, math::SignFunction<Scalar> signFunction>
size_t computeNormalizedDynamicEnergyStabilityMarginLeastStableEdgeIndex(
    const math::Vector3List<Scalar> &support_polygon, std::vector<Scalar> &edge_stabilities,
    const math::Vector3<Scalar> &center_of_mass,
    const math::Vector3<Scalar> &normalized_resultant, Scalar normalization_factor )
{
  return impl::computeNormalizedDynamicEnergyStabilityMargin<Scalar, signFunction, size_t>(
      support_polygon, edge_stabilities, center_of_mass, normalized_resultant,
      normalization_factor );
}
} // namespace hector_stability_metrics

#endif // HECTOR_STABILITY_METRICS_METRICS_NORMALIZED_DYNAMIC_ENERGY_STABILITY_MARGIN_H
