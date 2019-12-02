//
// Created by yash on 11/15/19.
//

#include <thread>

#include <fmt_star/plan_srv.h>

#include "mpc_av/follow_waypoints.h"
#include "mpc_av/mpc_solver.h"
#include "mpc_av/car_util.h"

/// Constructs the FollowWaypoints class
FollowWaypoints::FollowWaypoints() : node_handle_(ros::NodeHandle()), tf_listener_(tf_buffer_), visualized_(false)
{
    std::string pose_topic, drive_topic, csv_path, scan_topic, waypoint_input, csv_trajectories_path;
    node_handle_.getParam("pose_topic", pose_topic);
    node_handle_.getParam("drive_topic", drive_topic);
    node_handle_.getParam("csv_path", csv_path);
    node_handle_.getParam("csv_trajectories_path", csv_trajectories_path);
    node_handle_.getParam("scan_topic", scan_topic);
    node_handle_.getParam("/lookahead_distance", lookahead_distance_);
    node_handle_.getParam("/visualization_enabled", visualization_enabled_);
    node_handle_.getParam("/local_map_length", local_map_length_);
    node_handle_.getParam("/inflation_radius", inflation_radius_);
    node_handle_.getParam("/obstacle_avoidance", obstacle_avoidance_);
    node_handle_.getParam("/waypoint_input", waypoint_input);

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

    CSVReader trajectory_reader(csv_trajectories_path);
    solved_trajectories_ = trajectory_reader.getTrajectoriesData();
    ROS_INFO("Solved Trajectories Recieved");

    dynamic_map_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("dynamic_map", 1);
    scan_sub_ = node_handle_.subscribe(scan_topic, 1, &FollowWaypoints::scan_callback, this);
    sleep(1);
    pose_sub_ = node_handle_.subscribe(pose_topic, 5, &FollowWaypoints::pose_callback, this);

    drive_pub_ = node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    global_way_point_viz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("global_waypoint_markers", 100);
    local_way_point_viz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("local_waypoint_markers", 1);
    trajectory_viz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("trajectory_marker", 10);

    if(waypoint_input == "user")
    {
        global_path_client_ = node_handle_.serviceClient<fmt_star::plan_srv>("FMTstar_search");
        set_goal_sequence();
        get_global_plan_to_next_goal();
        std::thread strategy_thread(&FollowWaypoints::planning_strategy_thread, this);
        strategy_thread.detach();
        ROS_INFO("Online WayPoints Recieved");
    }
    else if(waypoint_input == "raceline")
    {
        CSVReader reader(csv_path);
        way_point_data_ = reader.getData();
        ROS_INFO("Offline WayPoints Recieved");
    }
    else
    {
        ROS_WARN("Invalid waypoint_input configuration. Using Default Waypoints");
    }

    ros::Duration(1.0).sleep();
    ROS_INFO("mpc_av_node Initialized");
}

/// This is a separate thread that is responsible for getting plans and doing replanning
/// whenever the situation is appropriate
void FollowWaypoints::planning_strategy_thread()
{
    while (ros::ok())
    {
        if (sqrt(pow(current_pose_[0] - current_goal_[0], 2) + pow(current_pose_[1] - current_goal_[1], 2)) < 2.5)
        {
            ROS_INFO("Updating Goal!");
            get_global_plan_to_next_goal();
        }
    }
}


/// The scan callback updates the occupancy grid
/// @param scan_msg - pointer to the incoming scan message
void FollowWaypoints::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    try
    {
        tf_laser_to_map_ = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
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
    if(clear_obstacles_count_ > 50)
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
void FollowWaypoints::pose_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    current_pose_ = {odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y};
    if(visualization_enabled_ && !visualized_)
    {
        visualize_waypoint_data(way_point_data_, &global_way_point_viz_pub_, "map");
    }

    // Transform Points
    const auto transformed_way_points = transform_waypoints_to_car_frame();

    // Find the best waypoint to track (at lookahead distance)
    const auto goal_way_point_index = get_global_trackpoint(transformed_way_points, lookahead_distance_);

    const auto goal_way_point_heading = get_waypoint_heading(transformed_way_points, goal_way_point_index);

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

    if(visualization_enabled_)
    {
        add_way_point_visualization({goal_way_point.position.x, goal_way_point.position.y}, &local_way_point_viz_pub_,
                                    "base_link", "local_waypoint", 1, 0.0, 0.0, 1.0, 0.3, 0.2, 0.2, 0.2);
    }

    const double input = get_best_trajectory_input(goal_way_point.position.x,
            goal_way_point.position.y,
            goal_way_point_heading);

    const double steering_speed = get_steering_speed(input);

    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = ros::Time::now();
    drive_msg.header.frame_id = "base_link";
    drive_msg.drive.steering_angle = input;
    drive_msg.drive.speed = steering_speed;
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

/// Gets a Required Heading Angle for the goal way point index in map frame
/// @param goal_way_point_index
/// @return heading angle for the waypoing (in radians in map frame)
double FollowWaypoints::get_waypoint_heading(const std::vector<std::array<double, 2>>& waypoints, int goal_way_point_index)
{
    const int start = [&](){
        if(goal_way_point_index == 0) return 0;
        if(goal_way_point_index == waypoints.size()) return static_cast<int>(waypoints.size()-3);
        else return goal_way_point_index-1;
    }();

    auto angle = [&](int endpoint_index_1, int endpoint_index_2){
        return atan2(waypoints[endpoint_index_2][1] - waypoints[endpoint_index_1][1],
                     waypoints[endpoint_index_2][0] - waypoints[endpoint_index_1][1]);
    };

    double average_angle = 0;
    for(int index=start; index<start+2; index++)
    {
        average_angle += angle(index, index+1);
    }
    return average_angle;
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

/// Checks if there is collision between two points (x1, y1) and (x2, y2)
/// @param x1 - base link frame
/// @param y1 - base link frame
/// @param x2 - base link frame
/// @param y2 - base link frame
/// @param n - n divisions for checking
/// @return true if there is collision. false otherwise
bool FollowWaypoints::is_collided(double x1, double y1, double x2, double y2) const
{
    for(int i=1; i<4; i++)
    {
        geometry_msgs::Pose traj_point;
        traj_point.position.x = (1-(0.3*i))*x1 + 0.3*i*x2;
        traj_point.position.y = (1-(0.3*i))*y1 + 0.3*i*y2;
        traj_point.position.z = 0;
        traj_point.orientation.x = 0;
        traj_point.orientation.y = 0;
        traj_point.orientation.z = 0;
        traj_point.orientation.w = 1;
        tf2::doTransform(traj_point, traj_point, tf_laser_to_map_);
        const auto index = get_row_major_index(traj_point.position.x, traj_point.position.y);
        if (dynamic_map_.data[index] == 100)
            return true;
    }
    return false;
}

double FollowWaypoints::get_best_trajectory_input(double goal_x, double goal_y, double goal_heading)
{
    ROS_DEBUG("Getting Best Trajectory ");
    int unique_id = 1;
    auto visualize_traj = [&](const std::vector<geometry_msgs::Pose>& states) {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "/base_link";
        line_strip.ns = "trajectories";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = unique_id++;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = 0.1;
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;
        line_strip.id = 1;
        line_strip.lifetime = ros::Duration(0.1);
        line_strip.header.stamp = ros::Time::now();
        geometry_msgs::Point start;
        start.x = 0;
        start.y = 0;
        start.z = 0;
        line_strip.points.emplace_back(start);
        for (const auto &state : states)
        {
            geometry_msgs::Point p;
            p.x = state.position.x;
            p.y = state.position.y;
            p.z = 0;
            line_strip.points.emplace_back(p);
        }
        trajectory_viz_pub_.publish(line_strip);
    };

    double best_cost = std::numeric_limits<double >::max();
    double best_input = 0;
    int best_index = -1;

    const int trajectory_state_size = solved_trajectories_[0].states.size();

    ROS_DEBUG("Starting Iteration");
    for(int i=0; i<solved_trajectories_.size(); i=i+5)
    {
        // Narrow down the best final heading angle based on the current required heading angle
        int best_heading_index = -1;
        double best_heading_cost = std::numeric_limits<double >::max();
        for(int k=i; k<i+5; k++)
        {
            double heading_cost = pow(solved_trajectories_[k].goal_heading - goal_heading, 2);
            if(heading_cost < best_heading_cost)
            {
                best_heading_cost = heading_cost;
                best_heading_index = k;
            }
        }

        // Collision Checking
        bool collision = false;
        for(int j=0; j<solved_trajectories_[best_heading_index].states.size(); j++)
        {
            geometry_msgs::Pose map_frame_state;
            tf2::doTransform(solved_trajectories_[best_heading_index].states[j], map_frame_state, tf_laser_to_map_);
            if (dynamic_map_.data[get_row_major_index(map_frame_state.position.x, map_frame_state.position.y)] == 100)
            {
                collision = true;
                break;
            }
            if(j==0)
            {
               if(is_collided(0, 0,
                            solved_trajectories_[best_heading_index].states[1].position.x, solved_trajectories_[best_heading_index].states[1].position.y))
               {
                   collision = true;
                   break;
               }
            }
            else if(j < 5 && is_collided(solved_trajectories_[best_heading_index].states[j-1].position.x, solved_trajectories_[best_heading_index].states[j-1].position.y,
                                solved_trajectories_[best_heading_index].states[j].position.x, solved_trajectories_[best_heading_index].states[j].position.y))
            {
                collision = true;
                break;
            }
        }
        if(collision)
        {
            continue;
        }

        // If no collisions, cost the trajectories
        double current_cost = pow(solved_trajectories_[best_heading_index].states[trajectory_state_size-1].position.x - goal_x,2) +
                pow(solved_trajectories_[best_heading_index].states[trajectory_state_size-1].position.y - goal_y,2);

        if(current_cost < best_cost)
        {
            best_cost = current_cost;
            best_input = solved_trajectories_[best_heading_index].steering_input;
            best_index = best_heading_index;
        }
    }
    if(best_index == -1)
    {
        ROS_ERROR("Failed to Find a Feasible Trajectory");
        return 0;
    }
    if(visualization_enabled_)
    {
        visualize_traj(solved_trajectories_[best_index].states);
    }
    return best_input;
}

/// Returns the row major index for the map
/// @param x_map - x coordinates in map frame
/// @param y_map - y coordinates in map frame
/// @return row major index of the map
int FollowWaypoints::get_row_major_index(const double x_map, const double y_map) const
{
    const auto x_index = static_cast<int>((x_map - map_origin_x_)/map_resolution_);
    const auto y_index = static_cast<int>((y_map - map_origin_y_)/map_resolution_);
    return y_index*map_cols_ + x_index;
}

void FollowWaypoints::get_global_plan_to_next_goal()
{
    fmt_star::plan_srv srv_message;

    const auto start = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("gt_pose",ros::Duration(1));
    const auto goal_point = goal_sequence_.front();

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "/map";
    goal.pose.position.x = goal_point[0];
    goal.pose.position.y = goal_point[1];
    goal.pose.position.z = 0;
    goal.pose.orientation.w = 1;

    if(!start)
    {
        ROS_ERROR("Unable to Plan. Localized location not available");
    }
    else
    {
        srv_message.request.start_position = *start;
        srv_message.request.end_position = goal;
        srv_message.request.update_map = true;
    }

    if (global_path_client_.call(srv_message))
    {
        ROS_INFO("Plan Recieved");
        way_point_mutex_.lock();
        way_point_data_.clear();
        for(const auto& path_node : srv_message.response.path.poses)
        {
            way_point_data_.emplace_back(std::array<double ,2>{path_node.pose.position.x,
                                                               path_node.pose.position.y});
        }
        current_goal_ = {goal_point[0], goal_point[1]};
        std::rotate(goal_sequence_.begin(), goal_sequence_.begin() + 1, goal_sequence_.end());
        way_point_mutex_.unlock();
    }
    else
    {
        ROS_ERROR("No Plan Recieved");
    }
}

/// Sets the global level goal waypoint sequence to follow
/// Note: This is the input given by the user manually using interactive markers
void FollowWaypoints::set_goal_sequence()
{
    char quit = 'x';
    ROS_INFO("Add Global Goal Sequence to Follow");
    while(quit != 'q')
    {
        ROS_INFO("Send a 2d Nav Goal using interactive 2d nav goal marker in rviz.");
        const auto goal = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("move_base_simple/goal",ros::Duration(20));
        if(goal)
        {
            goal_sequence_.emplace_back(std::array<double ,2>{goal->pose.position.x, goal->pose.position.y});
        }
        ROS_INFO("Press 'q' to stop adding more points. Press any other key to continue adding to the sequence.");
        std::cin >> quit;
    }
    ROS_INFO("Goal Sequence of size %i ready", goal_sequence_.size());
}