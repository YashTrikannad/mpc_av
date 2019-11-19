//
// Created by yash on 11/15/19.
//

#ifndef SRC_VISUALISATION_H
#define SRC_VISUALISATION_H

#include <ros/ros.h>

/// Used to visualize 2d waypoint
/// @param way_point
/// @param way_point_pub_
/// @param frame_id
/// @param r
/// @param g
/// @param b
/// @param transparency
/// @param scale_x
/// @param scale_y
/// @param scale_z
void add_way_point_visualization(const std::array<double, 2>& way_point,
        ros::Publisher* way_point_viz_pub,
        const std::string& frame_id,
        const std::string& ns,
        const size_t& unique_marker_id,
        double r = 0.5,
        double g = 0,
        double b = 0,
        double transparency = 0.5,
        double scale_x=0.1,
        double scale_y=0.1,
        double scale_z=0.1);

/// Used to visualize vector of 2d waypoints
/// @param waypoints
/// @param way_point_viz_pub
/// @param frame_id
/// @param unique_marker_id
/// @param r
/// @param g
/// @param b
/// @param transparency
/// @param scale_x
/// @param scale_y
/// @param scale_z
void visualize_waypoint_data(const std::vector<std::array<double, 2>>& waypoints,
                             ros::Publisher* way_point_viz_pub,
                             const std::string& frame_id,
                             double r = 0.5,
                             double g = 0,
                             double b = 0,
                             double transparency = 0.5,
                             double scale_x=0.1,
                             double scale_y=0.1,
                             double scale_z=0.1);

#endif //SRC_VISUALISATION_H
