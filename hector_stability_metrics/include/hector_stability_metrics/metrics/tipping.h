// Copyright (c) 2026 Aljoscha Schmidt. Licensed under the MIT license. See LICENSE file in the
// project root for full license information.

#ifndef HECTOR_STABILITY_METRICS_METRICS_TIPPING_H
#define HECTOR_STABILITY_METRICS_METRICS_TIPPING_H

#include "hector_stability_metrics/math/types.h"

#include <Eigen/Geometry>

#include <cmath>

namespace hector_stability_metrics
{
/*!
 * @defgroup Tipping Tipping over a support edge
 *
 * @brief The axis a support edge tips about, and the energy the robot's motion carries toward it.
 *
 * Tipping axes follow the polygon convention of the other metrics. For a support polygon wound
 * clockwise seen from above, the edge from vertex k to vertex k+1 tips outward under a positive
 * rotation about the unit axis pointing from vertex k+1 to vertex k, see tippingAxis. Energies are
 * normalized by the weight, so they are heights in metres like the NESM.
 *
 * @{
 */

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
 * axis, for which edgeKineticEnergy returns NaN.
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
 * from the edge returns a negative energy.
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

/*! @} */
} // namespace hector_stability_metrics

#endif // HECTOR_STABILITY_METRICS_METRICS_TIPPING_H
