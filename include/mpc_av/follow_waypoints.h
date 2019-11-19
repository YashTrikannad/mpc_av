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
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

#include "mpc_av/csv_reader.h"
#include "mpc_av/mpc_solver.h"
#include "mpc_av/visualisation.h"

class FollowWaypoints
{
public:
    FollowWaypoints();

private:
    // ROS Publishers and Subscribers
    ros::NodeHandle node_handle_;
    ros::Subscriber pose_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher global_way_point_viz_pub_;
    ros::Publisher local_way_point_viz_pub_;
    ros::Publisher drive_pub_;
    ros::Publisher dynamic_map_pub_;

    // Map
    nav_msgs::OccupancyGrid dynamic_map_;
    int map_cols_;
    std::vector<size_t > new_obstacles_;
    int clear_obstacles_count_;
    int inflation_radius_;
    double local_map_length_;
    double map_origin_x_;
    double map_origin_y_;
    double map_resolution_;

    // obstale avoidance
    bool obstacle_avoidance_;

    // Tracking and Speed
    double lookahead_distance_;
    double high_speed_;
    double medium_speed_;
    double low_speed_;

    // Input Waypoints
    std::vector<std::array<double, 2>> way_point_data_;

    // Transformations
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::Buffer tf_buffer_;
    geometry_msgs::TransformStamped tf_laser_to_map_;

    // MPC
    mpc::MPCSolver solver_;

    // Visualization
    bool visualization_enabled_;
    bool visualized_;

    /// Subscribes to the lidar message and updates the local area in the occupancy grid
    /// @param scan_msg
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

    /// Subscribes to the current pose, follows the next waypoint and updates the steering angle accordingly
    /// @param pose_msg - Localized Pose of the Robot
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);

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

    /// Returns the row major indices for the map of an inflated area around a point based on inflation radius
    /// @param x_map - x coordinates in map frame
    /// @param y_map - y coordinates in map frame
    /// @return row major index of the map
    std::vector<int> get_expanded_row_major_indices(double x_map, double y_map) const;

    /// Solves the MPC problem between the current pose and the required goal point
    /// @param goal_y - y in car frame
    /// @param goal_x - x in car frame
    /// @return steering angle for the car
    double solve_mpc(const double goal_y, const double goal_x);

    /// Checks if the path returned by mpc is in collision
    bool is_collided() const;
};

#endif //SRC_FOLLOW_WAYPOINTS_H
