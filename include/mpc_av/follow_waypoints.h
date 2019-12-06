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
#include <nav_msgs/OccupancyGrid.h>
#include <mutex>

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
    ros::Subscriber scan_sub_;
    ros::Publisher global_way_point_viz_pub_;
    ros::Publisher local_way_point_viz_pub_;
//    ros::Publisher current_pose_viz_pub_;
    ros::Publisher drive_pub_;
    ros::Publisher dynamic_map_pub_;

    double lookahead_distance_;
    double local_map_distance_;
    double high_speed_;
    double medium_speed_;
    double low_speed_;
    double parallel_offset_;
    int raceline_type_;
    int n_collision_checks_;

    /// Map
    nav_msgs::OccupancyGrid input_map_;
    int map_cols_;
    std::vector<size_t > new_obstacles_;
    int clear_obstacles_count_;
    int inflation_radius_;

    /// PID parameters
    double kp_;
    double ki_;
    double kd_;

    double prev_error_;
    double integral_;

    /// Waypoint Data
    /// Index 0 - Inner Line
    /// Index 1 - Center Line
    /// Index 2 - Outer Line
    std::array<std::vector<std::array<double, 2>>, 3> way_point_data_;

    tf2_ros::TransformListener tf_listener_;
    tf2_ros::Buffer tf_buffer_;

    mpc::MPCSolver solver_;
//    geometry_msgs::Pose current_pose_;

    std::mutex pose_mutex;

    int last_raceline_followed_;

    bool visualization_enabled_;
    bool visualized_;

    /// The scan callback updates the occupancy grid
    /// @param scan_msg - pointer to the incoming scan message
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

    /// The pose callback when subscribed to particle filter's inferred pose (RRT* Main Loop)
    /// @param pose_msg - pointer to the incoming pose message
    void pose_callback(const nav_msgs::OdometryConstPtr &pose_msg);

    /// Subscribes to the current pose, follows the next waypoint and updates the steering angle accordingly
    /// @param pose_msg - Localized Pose of the Robot
    void drive_thread();

    /// Publishes the appropriate speed based on the steering angle
    /// @param steering_angle
    void publish_corrected_speed_and_steering(double steering_angle);

    /// Transforms all the waypoints from the map to the car frame (In case of multiple racelines, all racelines)
    /// @return waypoints transformed in the car frame
    std::array<std::vector<std::array<double, 2>>, 3> transform_waypoints_to_car_frame();

    /// Returns the Closest Trackpoint at a lookahead distance from the current pose of the vehicle
    /// @param waypoints
    /// @param lookahead_distance
    /// @return index of the best trackpoint
    std::pair<size_t, double> get_global_trackpoint(const std::vector<std::array<double, 2>>& waypoints, double lookahead_distance, int i);

    /// Returns the row major indeices for the map of an inflated area around a point based on inflation radius
    /// @param x_map - x coordinates in map frame
    /// @param y_map - y coordinates in map frame
    /// @return row major index of the map
    std::vector<int> get_expanded_row_major_indices(const double x_map, const double y_map);

    /// Returns the row major index for the map
    /// @param x_map - x coordinates in map frame
    /// @param y_map - y coordinates in map frame
    /// @return row major index of the map
    int get_row_major_index(const double x_map, const double y_map);

    /// Chooses a the best raceline to follow
    /// @param racelines_map_frame
    /// @return
    int choose_raceline(const std::array<geometry_msgs::Pose, 3>& racelines_map_frame,
                        const std::array<geometry_msgs::Pose, 3>& racelines_car_frame,
                        const geometry_msgs::Pose& current_pose_map_frame);
};

#endif //SRC_FOLLOW_WAYPOINTS_H
