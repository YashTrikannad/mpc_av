//
// Created by yash on 11/15/19.
//

#ifndef SRC_FOLLOW_WAYPOINTS_H
#define SRC_FOLLOW_WAYPOINTS_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include "mpc_av/csv_reader.h"
#include "mpc_av/mpc_solver.h"
#include "mpc_av/visualisation.h"

class FollowWaypoints
{
public:
    FollowWaypoints();

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber pose_sub_;
    ros::Publisher global_way_point_viz_pub_;
    ros::Publisher local_way_point_viz_pub_;
    ros::Publisher drive_pub_;

    double lookahead_distance_;
    double high_speed_;
    double medium_speed_;
    double low_speed_;

    std::vector<std::array<double, 2>> way_point_data_;

    tf2_ros::TransformListener tf_listener_;
    tf2_ros::Buffer tf_buffer_;

    mpc::MPCSolver solver_;

    bool visualization_enabled_;
    bool visualized_;

    /// Subscribes to the current pose, follows the next waypoint and updates the steering angle accordingly
    /// @param pose_msg - Localized Pose of the Robot
    void pose_callback(const nav_msgs::OdometryConstPtr odom_msg);

    /// Publishes the appropriate speed based on the steering angle
    /// @param steering_angle
    void publish_corrected_speed_and_steering(double steering_angle);

    /// Transforms all the waypoints from the map to the car frame
    /// @return waypoints transformed in the car frame
    std::vector<std::array<double, 2>> transform_waypoints_to_car_frame();

    /// Returns the Closest Trackpoint at a lookahead distance from the current pose of the vehicle
    /// @param waypoints
    /// @param lookahead_distance
    /// @return index of the best trackpoint
    size_t get_global_trackpoint(const std::vector<std::array<double, 2>>& waypoints, double lookahead_distance);
};

#endif //SRC_FOLLOW_WAYPOINTS_H
