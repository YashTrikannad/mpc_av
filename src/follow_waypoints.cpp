#include "mpc_av/follow_waypoints.h"
#include "mpc_av/mpc_solver.h"

/// Constructs the FollowWaypoints class
FollowWaypoints::FollowWaypoints() : node_handle_(ros::NodeHandle()), tf_listener_(tf_buffer_), visualized_(false)
{
    std::string localized_pose_topic, drive_topic, csv_path;
    node_handle_.getParam("pose_topic", localized_pose_topic);
    node_handle_.getParam("drive_topic", drive_topic);
    node_handle_.getParam("csv_path", csv_path);
    node_handle_.getParam("/lookahead_distance", lookahead_distance_);
    node_handle_.getParam("/high_speed", high_speed_);
    node_handle_.getParam("/medium_speed", medium_speed_);
    node_handle_.getParam("/low_speed", low_speed_);
    node_handle_.getParam("/visualization_enabled", visualization_enabled_);

    pose_sub_ = node_handle_.subscribe(localized_pose_topic, 5, &FollowWaypoints::pose_callback, this);
    drive_pub_ = node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    global_way_point_viz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("global_waypoint_markers", 100);
    local_way_point_viz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("local_waypoint_markers", 1);

    // TODO: CHANGE INTERFACE OF GETTING WAYPOINTS
    CSVReader reader(csv_path);
    way_point_data_ = reader.getData();
    ROS_INFO("WayPoints Recieved");

    ros::Duration(1.0).sleep();
    ROS_INFO("RTDAV_planner node Initialized");
}

/// Subscribes to the current pose, follows the next waypoint and updates the steering angle accordingly
/// @param pose_msg - Localized Pose of the Robot
void FollowWaypoints::pose_callback(const nav_msgs::OdometryConstPtr odom_msg)
{
    if(visualization_enabled_ && !visualized_)
    {
        visualize_waypoint_data(way_point_data_, &global_way_point_viz_pub_, "map");
    }

    // Transform Points
    const auto transformed_way_points = transform_waypoints_to_car_frame();

    // Find the best waypoint to track (at lookahead distance)
    const auto goal_way_point_index = get_global_trackpoint(transformed_way_points, lookahead_distance_);

    geometry_msgs::TransformStamped map_to_base_link;
    map_to_base_link = tf_buffer_.lookupTransform("base_link", "map", ros::Time(0));
    geometry_msgs::Pose goal_way_point;
    goal_way_point.position.x = way_point_data_[goal_way_point_index][0];
    goal_way_point.position.y = way_point_data_[goal_way_point_index][1];
    goal_way_point.position.z = 0;
    goal_way_point.orientation.x = 0;
    goal_way_point.orientation.y = 0;
    goal_way_point.orientation.z = 0;
    goal_way_point.orientation.w = 1;
    tf2::doTransform(goal_way_point, goal_way_point, map_to_base_link);

    add_way_point_visualization({goal_way_point.position.x, goal_way_point.position.y}, &local_way_point_viz_pub_,
            "base_link", "local_waypoint", 1, 0.0, 0.0, 1.0, 0.3, 0.2, 0.2, 0.2);

    const double input = solver_.solve_mpc({goal_way_point.position.y, goal_way_point.position.x});

    publish_corrected_speed_and_steering(input);
}


/// Returns the appropriate speed based on the steering angle
/// @param steering_angle
/// @return
void FollowWaypoints::publish_corrected_speed_and_steering(double steering_angle)
{
    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = ros::Time::now();
    drive_msg.header.frame_id = "base_link";

    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = high_speed_;

    std::cout<< high_speed_ << " " << medium_speed_ << " " << low_speed_ << "\n";

    if(steering_angle > 0.1)
    {
        if (steering_angle > 0.2)
        {
            drive_msg.drive.speed = low_speed_;
        }
        else
        {
            drive_msg.drive.speed = medium_speed_;
        }
    }
    else if(steering_angle < -0.1)
    {
        if (steering_angle < -0.2)
        {
            drive_msg.drive.speed = low_speed_;
        }
        else
        {
            drive_msg.drive.speed = medium_speed_;
        }
    }
    else
    {
        drive_msg.drive.speed = high_speed_;
    }

    std::cout << "speed: " << drive_msg.drive.speed << " ";
    std::cout << "angle: " << drive_msg.drive.steering_angle << "\n";
    drive_pub_.publish(drive_msg);
}

/// Transforms all the waypoints from the map to the car frame
/// @return waypoints transformed in the car frame
std::vector<std::array<double, 2>> FollowWaypoints::transform_waypoints_to_car_frame()
{
    geometry_msgs::TransformStamped map_to_base_link;
    map_to_base_link = tf_buffer_.lookupTransform("base_link", "map", ros::Time(0));

    std::vector<std::array<double, 2>> transformed_way_points;
    for(const auto& reference_way_point: way_point_data_)
    {
        geometry_msgs::Pose map_way_point;
        map_way_point.position.x = reference_way_point[0];
        map_way_point.position.y = reference_way_point[1];
        map_way_point.position.z = 0;
        map_way_point.orientation.x = 0;
        map_way_point.orientation.y = 0;
        map_way_point.orientation.z = 0;
        map_way_point.orientation.w = 1;

        tf2::doTransform(map_way_point, map_way_point, map_to_base_link);

        std::array<double, 2> transformed_waypoint{map_way_point.position.x, map_way_point.position.y};
        transformed_way_points.emplace_back(transformed_waypoint);
    }
    return transformed_way_points;
}

/// Returns the Closest Trackpoint at a lookahead distance from the current pose of the vehicle
/// @param waypoints
/// @param lookahead_distance
/// @return index of the best trackpoint
size_t FollowWaypoints::get_global_trackpoint(const std::vector<std::array<double, 2>>& waypoints, double lookahead_distance)
{
    double closest_distance = std::numeric_limits<double>::max();
    const size_t way_point_size = waypoints.size();
    int best_index = -1;

    for(size_t i=0; i <waypoints.size(); ++i)
    {
        if(waypoints[i][0] < 0) continue;
        double distance = sqrt(waypoints[i][0]*waypoints[i][0] +
                                       waypoints[i][1]*waypoints[i][1]);
        double lookahead_diff = std::abs(distance - lookahead_distance);
        if(lookahead_diff < closest_distance)
        {
            closest_distance = lookahead_diff;
            best_index = i;
        }
    }

    return best_index;
}

