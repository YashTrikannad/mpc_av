//
// Created by yash on 11/15/19.
//

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "mpc_av/visualisation.h"

/// Used to visualize 2d waypoint
void add_way_point_visualization(const std::array<double, 2>& way_point,
                                 ros::Publisher* way_point_viz_pub,
                                 const std::string& frame_id,
                                 const std::string& ns,
                                 const size_t& unique_marker_id,
                                 double r,
                                 double g,
                                 double b,
                                 double transparency,
                                 double scale_x,
                                 double scale_y,
                                 double scale_z)
{
    visualization_msgs::Marker way_point_marker;
    way_point_marker.header.frame_id = frame_id;
    way_point_marker.header.stamp = ros::Time();
    way_point_marker.ns = ns;
    way_point_marker.id = unique_marker_id;
    way_point_marker.type = visualization_msgs::Marker::SPHERE;
    way_point_marker.action = visualization_msgs::Marker::ADD;
    way_point_marker.pose.position.x = way_point[0];
    way_point_marker.pose.position.y = way_point[1];
    way_point_marker.pose.position.z = 0;
    way_point_marker.pose.orientation.x = 0.0;
    way_point_marker.pose.orientation.y = 0.0;
    way_point_marker.pose.orientation.z = 0.0;
    way_point_marker.pose.orientation.w = 1.0;
    way_point_marker.scale.x = scale_x;
    way_point_marker.scale.y = scale_y;
    way_point_marker.scale.z = scale_z;
    way_point_marker.color.a = transparency;
    way_point_marker.color.r = r;
    way_point_marker.color.g = g;
    way_point_marker.color.b = b;
    way_point_viz_pub->publish(way_point_marker);
}

/// Used to visualize vector of 2d waypoints
void visualize_waypoint_data(const std::vector<std::array<double, 2>>& waypoints,
                             ros::Publisher* way_point_viz_pub,
                             const std::string& frame_id,
                             double r,
                             double g,
                             double b,
                             double transparency,
                             double scale_x,
                             double scale_y,
                             double scale_z)
{
    const size_t increment = 1;
    for(size_t i=0; i<waypoints.size(); i=i+increment)
    {
        add_way_point_visualization(waypoints[i],  way_point_viz_pub, frame_id, "global_waypoints", i, 0.0, 0.0, 1.0, 0.5);
    }
}