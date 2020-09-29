/*
 * Copyright (C) 2020  Stefan Fabian
 *
 * This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef HECTOR_STABILITY_METRICS_MESSAGE_CONVERSIONS_EIGEN_H
#define HECTOR_STABILITY_METRICS_MESSAGE_CONVERSIONS_EIGEN_H

#include "hector_stability_metrics/types.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <Eigen/Geometry>

namespace hector_stability_metrics
{
/*!
 * IMPORTANT: These conversion methods are provided as headers but the library does not depend on the message types.
 *   Hence, make sure that your project depends on them if you use them.
 */
namespace message_conversions
{
template<typename Scalar>
Vector3<Scalar> msgToVector( const geometry_msgs::Point &msg )
{
  return { static_cast<Scalar>(msg.x), static_cast<Scalar>(msg.y),
           static_cast<Scalar>(msg.z) };
}

template<typename Scalar>
Vector3<Scalar> msgToVector( const geometry_msgs::Vector3 &msg )
{
  return { static_cast<Scalar>(msg.x), static_cast<Scalar>(msg.y),
           static_cast<Scalar>(msg.z) };
}

template<typename Scalar>
Eigen::Quaternion<Scalar> msgToQuaternion( const geometry_msgs::Quaternion &msg )
{
  return { static_cast<Scalar>(msg.w), static_cast<Scalar>(msg.x),
           static_cast<Scalar>(msg.y), static_cast<Scalar>(msg.z) };
}

template<typename Scalar>
Isometry3<Scalar> msgToTransform( const geometry_msgs::Pose &msg )
{
  Isometry3<Scalar> transform = Isometry3<Scalar>::Identity();
  transform.linear() = msgToQuaternion<Scalar>( msg.orientation ).toRotationMatrix();
  transform.translation() = msgToVector<Scalar>( msg.position );
  return transform;
}

template<typename Scalar>
Isometry3<Scalar> msgToTransform( const geometry_msgs::Transform &msg )
{
  Isometry3<Scalar> transform = Isometry3<Scalar>::Identity();
  transform.linear() = msgToQuaternion<Scalar>( msg.rotation ).toRotationMatrix();
  transform.translation() = msgToVector<Scalar>( msg.translation );
  return transform;
}

template<typename Derived>
geometry_msgs::Point vectorToPointMsg( const Eigen::DenseBase <Derived> &vec )
{
  geometry_msgs::Point msg;
  msg.x = vec.x();
  msg.y = vec.y();
  msg.z = vec.z();
  return msg;
}

template<typename Derived>
geometry_msgs::Vector3 vectorToVectorMsg( const Eigen::DenseBase <Derived> &vec )
{
  geometry_msgs::Vector3 msg;
  msg.x = vec.x();
  msg.y = vec.y();
  msg.z = vec.z();
  return msg;
}

template<typename Scalar>
geometry_msgs::Quaternion quaternionToMsg( const Eigen::Quaternion<Scalar> &q )
{
  geometry_msgs::Quaternion msg;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  return msg;
}

template<typename Scalar>
geometry_msgs::Pose transformToPoseMsg( const Isometry3<Scalar> &pose )
{
  geometry_msgs::Pose msg;
  msg.position = vectorToPointMsg( pose.translation());
  msg.orientation = quaternionToMsg( Eigen::Quaternion<Scalar>( pose.linear()));
  return msg;
}

template<typename Scalar>
geometry_msgs::Transform transformToTransformMsg( const Isometry3<Scalar> &pose )
{
  geometry_msgs::Transform msg;
  msg.translation = vectorToVectorMsg( pose.translation());
  msg.rotation = quaternionToMsg( Eigen::Quaternion<Scalar>( pose.linear()));
  return msg;
}
}  // namespace message_conversions
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_MESSAGE_CONVERSIONS_EIGEN_H