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
#include <nav_msgs/Odometry.h>
#include <mutex>

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
    ros::Publisher trajectory_viz_pub_;
    ros::Publisher drive_pub_;
    ros::Publisher dynamic_map_pub_;

    ros::ServiceClient global_path_client_;

    std::array<double, 2> current_pose_;
    std::array<double, 2> current_goal_;

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

    // MPC solved trajectories
    std::vector<trajectory> solved_trajectories_;

    // Input Waypoints
    std::vector<std::array<double, 2>> way_point_data_;

    // Goal Sequence Waypoints
    std::vector<std::array<double, 2>> goal_sequence_;

    // Transformations
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::Buffer tf_buffer_;
    geometry_msgs::TransformStamped tf_laser_to_map_;

    // Mutex
    std::mutex way_point_mutex_;

    // Visualization
    bool visualization_enabled_;
    bool visualized_;

    /// This is a separate thread that is responsible for getting plans and doing replanning
    /// whenever the situation is appropriate
    void planning_strategy_thread();

    /// Subscribes to the lidar message and updates the local area in the occupancy grid
    /// @param scan_msg
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

    /// Subscribes to the current pose, follows the next waypoint and updates the steering angle accordingly
    /// @param pose_msg - Localized Pose of the Robot
    void pose_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);

    /// Transforms all the waypoints from the map to the car frame
    /// @return waypoints transformed in the car frame
    std::vector<std::array<double, 2>> transform_waypoints_to_car_frame();

    /// Returns the Closest Trackpoint at a lookahead distance from the current pose of the vehicle
    /// @param waypoints
    /// @param lookahead_distance
    /// @return index of the best trackpoint
    size_t get_global_trackpoint(const std::vector<std::array<double, 2>>& waypoints, double lookahead_distance);

    /// Gets a Required Heading Angle for the goal way point index in map frame
    /// @param waypoints
    /// @param goal_way_point_index
    /// @return heading angle for the waypoing (in radians in map frame)
    double get_waypoint_heading(const std::vector<std::array<double, 2>>& waypoints, int goal_way_point_index);

    /// Returns the row major indices for the map of an inflated area around a point based on inflation radius
    /// @param x_map - x coordinates in map frame
    /// @param y_map - y coordinates in map frame
    /// @return row major index of the map
    std::vector<int> get_expanded_row_major_indices(double x_map, double y_map) const;

    /// Finds the Best Trajectory to follow from the offline trajectories solved and returns the steering
    /// angle at input at time = 0
    /// @param goal_x - car frame x
    /// @param goal_y - car frame y
    /// @param goal_heading - map frame
    /// @return
    double get_best_trajectory_input(const double goal_x, const double goal_y, double goal_heading);

    /// Checks if there is collision between two points (x1, y1) and (x2, y2)
    /// @param x1 - base link frame
    /// @param y1 - base link frame
    /// @param x2 - base link frame
    /// @param y2 - base link frame
    /// @param n - n divisions for checking
    /// @return true if there is collision. false otherwise
    bool is_collided(double x1, double y1, double x2, double y2) const;

    /// Returns the row major index for the map
    /// @param x_map - x coordinates in map frame
    /// @param y_map - y coordinates in map frame
    /// @return row major index of the map
    int get_row_major_index(const double x_map, const double y_map) const;

    /// Get the global plan to the next goal in the sequence
    void get_global_plan_to_next_goal();

    /// Sets the global level goal waypoint sequence to follow
    /// Note: This is the input given by the user manually using interactive markers
    void set_goal_sequence();
};

#endif //SRC_FOLLOW_WAYPOINTS_H
