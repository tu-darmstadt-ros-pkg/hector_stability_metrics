// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_STABILITY_METRICS_MESSAGE_CONVERSIONS_SUPPORT_POLYGON_H
#define HECTOR_STABILITY_METRICS_MESSAGE_CONVERSIONS_SUPPORT_POLYGON_H

#include "hector_stability_metrics/math/types.h"
#include <visualization_msgs/MarkerArray.h>

namespace hector_stability_metrics
{
/*!
 * IMPORTANT: These conversion methods are provided as headers but the library does not depend on
 * the message types. Hence, make sure that your project depends on them if you use them.
 */
namespace message_conversions
{

template<typename Scalar>
using ColorMapper = std::function<std_msgs::ColorRGBA( Scalar )>;

/*!
 * Converts a value from 0 to 1 to an RGB color from blue to red.
 * Implemented following the colour ramping for data visualisation approach described by Paul
 * Bourke: http://paulbourke.net/miscellaneous/colourspace/
 */
template<typename Scalar>
inline std_msgs::ColorRGBA intensityToColor( Scalar value )
{
  std_msgs::ColorRGBA color;
  color.a = 1.0f;
  if ( value < 0 ) {
    color.g = 1;
  } else if ( value > 1 ) {
    color.r = 1;
  } else if ( value <= 0.25 ) {
    color.g = 4 * value;
    color.b = 1;
  } else if ( value <= 0.5 ) {
    color.g = 1;
    color.b = 2 - 4 * value;
  } else if ( value <= 0.75 ) {
    color.r = 4 * value - 2;
    color.g = 1;
  } else if ( std::isfinite( value ) ) {
    color.r = 1;
    color.g = 4 - 4 * value;
  } else {
    color.r = color.g = color.b = 0.6;
  }
  return color;
}

template<typename Scalar>
inline visualization_msgs::MarkerArray supportPolygonToMarkerArray(
    const math::Vector3List<Scalar> &support_polygon, const std::vector<Scalar> &edge_stabilities,
    const std::string &robot_frame, const std_msgs::ColorRGBA &contact_marker_color,
    const ColorMapper<Scalar> &color_mapper = intensityToColor<Scalar> )
{
  visualization_msgs::MarkerArray support_polygon_msg;
  visualization_msgs::Marker delete_marker;
  delete_marker.header.frame_id = robot_frame;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  support_polygon_msg.markers.push_back( delete_marker );
  for ( size_t i = 0; i < support_polygon.size(); ++i ) {
    const math::Vector3<Scalar> &contact_point = support_polygon[i];
    visualization_msgs::Marker contact_point_msg;
    contact_point_msg.header.frame_id = robot_frame;
    contact_point_msg.ns = "contact_points";
    contact_point_msg.id = i;
    contact_point_msg.type = visualization_msgs::Marker::SPHERE;
    contact_point_msg.pose.orientation.w = 1;
    contact_point_msg.pose.position.x = contact_point.x();
    contact_point_msg.pose.position.y = contact_point.y();
    contact_point_msg.pose.position.z = contact_point.z();
    contact_point_msg.scale.x = contact_point_msg.scale.y = contact_point_msg.scale.z = 0.1;
    contact_point_msg.color = contact_marker_color;
    support_polygon_msg.markers.push_back( contact_point_msg );

    visualization_msgs::Marker axis_msg;
    size_t b = i + 1;
    if ( b == support_polygon.size() )
      b = 0;
    axis_msg.header.frame_id = robot_frame;
    axis_msg.ns = "support_polygon_axis";
    axis_msg.id = i;
    axis_msg.type = visualization_msgs::Marker::ARROW;
    axis_msg.pose.orientation.w = 1;
    geometry_msgs::Point start;
    start.x = contact_point.x();
    start.y = contact_point.y();
    start.z = contact_point.z();
    const math::Vector3<Scalar> &end_point = support_polygon[b];
    geometry_msgs::Point end;
    end.x = end_point.x();
    end.y = end_point.y();
    end.z = end_point.z();
    axis_msg.points.push_back( start );
    axis_msg.points.push_back( end );
    axis_msg.scale.x = axis_msg.scale.y = axis_msg.scale.z = 0.025;
    axis_msg.color = color_mapper( 1 - edge_stabilities[i] );
    axis_msg.color.a = 0.5f;
    support_polygon_msg.markers.push_back( axis_msg );
  }
  return support_polygon_msg;
}

template<typename Scalar>
inline visualization_msgs::MarkerArray
supportPolygonToMarkerArray( const math::Vector3List<Scalar> &support_polygon,
                             const std::vector<Scalar> &edge_stabilities,
                             const std::string &robot_frame,
                             const ColorMapper<Scalar> &color_mapper = intensityToColor<Scalar> )
{
  std_msgs::ColorRGBA color;
  color.r = color.g = 0;
  color.b = 1;
  color.a = 0.5;
  return supportPolygonToMarkerArray<Scalar>( support_polygon, edge_stabilities, robot_frame, color,
                                              color_mapper );
}

template<typename Scalar>
inline visualization_msgs::MarkerArray
supportPolygonToMarkerArray( const math::Vector3List<Scalar> &support_polygon,
                             const std::vector<Scalar> &edge_stabilities,
                             const math::Isometry3<Scalar> &pose, const std::string &world_frame,
                             const std_msgs::ColorRGBA &contact_marker_color,
                             const ColorMapper<Scalar> &color_mapper = intensityToColor<Scalar> )
{
  auto copy = support_polygon;
  for ( auto &cp : copy ) { cp = pose * cp; }
  return supportPolygonToMarkerArray( copy, edge_stabilities, world_frame, contact_marker_color,
                                      color_mapper );
}

template<typename Scalar>
inline visualization_msgs::MarkerArray
supportPolygonToMarkerArray( const math::Vector3List<Scalar> &support_polygon,
                             const std::vector<Scalar> &edge_stabilities,
                             const math::Isometry3<Scalar> &pose, const std::string &world_frame,
                             const ColorMapper<Scalar> &color_mapper = intensityToColor<Scalar> )
{
  std_msgs::ColorRGBA color;
  color.r = color.g = 0;
  color.b = 1;
  color.a = 0.5;
  return supportPolygonToMarkerArray<Scalar>( support_polygon, edge_stabilities, pose, world_frame,
                                              color, color_mapper );
}
} // namespace message_conversions
} // namespace hector_stability_metrics

#endif // HECTOR_STABILITY_METRICS_MESSAGE_CONVERSIONS_SUPPORT_POLYGON_H