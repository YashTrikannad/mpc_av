//
// Created by yash on 11/15/19.
//

#include "mpc_av/follow_waypoints.h"
#include "mpc_av/mpc_solver.h"

/// Constructs the FollowWaypoints class
FollowWaypoints::FollowWaypoints() : node_handle_(ros::NodeHandle()), tf_listener_(tf_buffer_), visualized_(false)
{
    std::string localized_pose_topic, drive_topic, csv_path, scan_topic;
    node_handle_.getParam("pose_topic", localized_pose_topic);
    node_handle_.getParam("drive_topic", drive_topic);
    node_handle_.getParam("csv_path", csv_path);
    node_handle_.getParam("scan_topic", scan_topic);
    node_handle_.getParam("/lookahead_distance", lookahead_distance_);
    node_handle_.getParam("/high_speed", high_speed_);
    node_handle_.getParam("/medium_speed", medium_speed_);
    node_handle_.getParam("/low_speed", low_speed_);
    node_handle_.getParam("/visualization_enabled", visualization_enabled_);
    node_handle_.getParam("/local_map_length", local_map_length_);
    node_handle_.getParam("/inflation_radius", inflation_radius_);
    node_handle_.getParam("/obstacle_avoidance", obstacle_avoidance_);

    // Load Input Map from map_server
    dynamic_map_  = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map",ros::Duration(2)));
    if(dynamic_map_.data.empty())
    {
        std::__throw_invalid_argument("Input Map Load Unsuccessful\"");
    }
    ROS_INFO("Map Load Successful.");
    map_cols_ = dynamic_map_.info.width;
    new_obstacles_ = {};
    new_obstacles_.reserve(2000);
    clear_obstacles_count_ = 0;
    map_origin_x_ = dynamic_map_.info.origin.position.x;
    map_origin_y_ = dynamic_map_.info.origin.position.y;
    map_resolution_ = dynamic_map_.info.resolution;

    solver_ = mpc::MPCSolver();

    dynamic_map_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("dynamic_map", 1);
    scan_sub_ = node_handle_.subscribe(scan_topic, 1, &FollowWaypoints::scan_callback, this);
    sleep(1);
    pose_sub_ = node_handle_.subscribe(localized_pose_topic, 5, &FollowWaypoints::pose_callback, this);

    drive_pub_ = node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    global_way_point_viz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("global_waypoint_markers", 100);
    local_way_point_viz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("local_waypoint_markers", 1);

    // TODO: CHANGE INTERFACE OF GETTING WAYPOINTS
    CSVReader reader(csv_path);
    way_point_data_ = reader.getData();
    ROS_INFO("WayPoints Recieved");

    ros::Duration(1.0).sleep();
    ROS_INFO("mpc_av_node Initialized");
}

///// The scan callback updates the occupancy grid
///// @param scan_msg - pointer to the incoming scan message
void FollowWaypoints::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    try
    {
        tf_laser_to_map_ = tf_buffer_.lookupTransform("map", "laser", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.01).sleep();
    }
    const auto translation = tf_laser_to_map_.transform.translation;
    const double yaw = tf::getYaw(tf_laser_to_map_.transform.rotation);

    const auto start = static_cast<int>(scan_msg->ranges.size()/4);
    const auto end = static_cast<int>(3*scan_msg->ranges.size()/4);
    const double angle_increment = scan_msg->angle_increment;
    double theta = scan_msg->angle_min + angle_increment*(start-1);

    for(int i=start; i<end; ++i)
    {
        theta+=angle_increment;

        const double hit_range = scan_msg->ranges[i];
        if(std::isinf(hit_range) || std::isnan(hit_range)) continue;

        // laser hit x, y in base_link frame
        const double x_base_link = hit_range*cos(theta);
        const double y_base_link = hit_range*sin(theta);

        if(x_base_link > local_map_length_ || y_base_link > local_map_length_) continue;

        // laser hit x, y in base_link frame
        const double x_map = x_base_link*cos(yaw) - y_base_link*sin(yaw) + translation.x;
        const double y_map = x_base_link*sin(yaw) + y_base_link*cos(yaw) + translation.y;

        std::vector<int> index_of_expanded_obstacles = get_expanded_row_major_indices(x_map, y_map);

        for(const auto& index: index_of_expanded_obstacles)
        {
            if(dynamic_map_.data[index] != 100)
            {
                dynamic_map_.data[index] = 100;
                new_obstacles_.emplace_back(index);
            }
        }
    }

    clear_obstacles_count_++;
    if(clear_obstacles_count_ > 10)
    {
        for(const auto index: new_obstacles_)
        {
            dynamic_map_.data[index] = 0;
        }
        new_obstacles_.clear();
        clear_obstacles_count_ = 0;
        ROS_INFO("Obstacles Cleared");
    }

    dynamic_map_pub_.publish(dynamic_map_);
    ROS_INFO("Map Updated");
}

/// Subscribes to the current pose, follows the next waypoint and updates the steering angle accordingly
/// @param pose_msg - Localized Pose of the Robot
void FollowWaypoints::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
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

    const auto input = solve_mpc(goal_way_point.position.y, goal_way_point.position.x);
    settings.verbose = 0;
    settings.eps = 1e-2;

    // Calculate curvature/steering angle
    const double steering_angle = 2*(goal_way_point.position.y)/(lookahead_distance_*lookahead_distance_);

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

/// Returns the row major indeices for the map of an inflated area around a point based on inflation radius
/// @param x_map - x coordinates in map frame
/// @param y_map - y coordinates in map frame
/// @return row major index of the map
std::vector<int> FollowWaypoints::get_expanded_row_major_indices(const double x_map, const double y_map) const
{
    std::vector<int> expanded_row_major_indices;
    const auto x_index = static_cast<int>((x_map - map_origin_x_)/map_resolution_);
    const auto y_index = static_cast<int>((y_map - map_origin_y_)/map_resolution_);
    for(int i=-inflation_radius_+x_index; i<inflation_radius_+1+x_index; i++)
    {
        for(int j=-inflation_radius_+y_index; j<inflation_radius_+1+y_index; j++)
        {
            expanded_row_major_indices.emplace_back(j*map_cols_ + i);
        }
    }
    return expanded_row_major_indices;
}

/// Solves the MPC problem between the current pose and the required goal point
/// @param required_goalpoint - (y, x) of car in local frame
/// @return steering angle for the car
double FollowWaypoints::solve_mpc(const double goal_y, const double goal_x)
{
    double input_original = solver_.solve_mpc({goal_y, goal_x});
    if(!obstacle_avoidance_ || !is_collided()) return input_original;

    double increment = 0.1;
    double positive_goal_y = goal_y;
    double negative_goal_y = goal_y;
    double input = 0;
    for(int i=0; i< 5; i++)
    {
        positive_goal_y += increment;
        input = solver_.solve_mpc({positive_goal_y, goal_x});
        if(!is_collided()) return input;
        negative_goal_y -= increment;
        input = solver_.solve_mpc({negative_goal_y, goal_x});
        if(!is_collided()) return input;
    }
    ROS_ERROR("MPC was not able to find a Path");
    return input_original;
}

/// Checks if the path returned by mpc is in collision
bool FollowWaypoints::is_collided() const
{
    for(int i=1; i<3; i++)
    {
        geometry_msgs::Pose traj_point;
        traj_point.position.x = vars.x[i][1];
        traj_point.position.y = vars.x[i][0];
        traj_point.position.z = 0;
        traj_point.orientation.x = 0;
        traj_point.orientation.y = 0;
        traj_point.orientation.z = 0;
        traj_point.orientation.w = 1;
        tf2::doTransform(traj_point, traj_point, tf_laser_to_map_);
        std::vector<int> map_indices = get_expanded_row_major_indices(traj_point.position.x, traj_point.position.y);
        for(const auto& index: map_indices)
        {
            if(dynamic_map_.data[index]==100)
                return true;
        }
        std::cout << "Didnt Collide for " << i << std::endl;
    }
    return false;
}