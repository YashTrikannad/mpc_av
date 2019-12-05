#include <tf/exceptions.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <thread>

#include "mpc_av/follow_waypoints.h"
#include "mpc_av/mpc_solver.h"

/// Constructs the FollowWaypoints class
FollowWaypoints::FollowWaypoints() : node_handle_(ros::NodeHandle()), tf_listener_(tf_buffer_), visualized_(false)
{
    std::string localized_pose_topic, drive_topic, csv_path_inner, csv_path_outer, csv_path_center;
    node_handle_.getParam("pose_topic", localized_pose_topic);
    node_handle_.getParam("drive_topic", drive_topic);
    node_handle_.getParam("csv_path_inner", csv_path_inner);
    node_handle_.getParam("csv_path_center", csv_path_center);
    node_handle_.getParam("csv_path_outer", csv_path_outer);
    node_handle_.getParam("/lookahead_distance", lookahead_distance_);
    node_handle_.getParam("/high_speed", high_speed_);
    node_handle_.getParam("/medium_speed", medium_speed_);
    node_handle_.getParam("/low_speed", low_speed_);
    node_handle_.getParam("/visualization_enabled", visualization_enabled_);
    node_handle_.getParam("inflation_radius", inflation_radius_);
    node_handle_.getParam("local_map_distance", local_map_distance_);

    // Load Input Map from map_server
    input_map_  = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map",ros::Duration(2)));
    if(input_map_.data.empty())
    {
        std::__throw_invalid_argument("Input Map Load Unsuccessful\"");
    }
    ROS_INFO("Map Load Successful.");

    last_raceline_followed_ = 0;

    map_cols_ = input_map_.info.width;
    new_obstacles_ = {};
    new_obstacles_.reserve(2000);
    clear_obstacles_count_ = 0;

    bool transform_recieved = false;
    while(!transform_recieved)
    {
        try
        {
            const auto map_to_base_link = tf_buffer_.lookupTransform("base_link", "map", ros::Time(0));
            ROS_INFO("Localized");
            transform_recieved = true;
            ros::Duration(5.0).sleep();
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    CSVReader inner_reader(csv_path_inner);
    way_point_data_[0] = inner_reader.getData();
    CSVReader center_reader(csv_path_center);
    way_point_data_[1] = center_reader.getData();
    CSVReader outer_reader(csv_path_outer);
    way_point_data_[2] = outer_reader.getData();

    dynamic_map_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("dynamic_map", 1);
    scan_sub_ = node_handle_.subscribe("scan", 5, &FollowWaypoints::scan_callback, this);
//    pose_sub_ = node_handle_.subscribe(localized_pose_topic, 5, &FollowWaypoints::pose_callback, this);
    drive_pub_ = node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    global_way_point_viz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("global_waypointa", 5);
    local_way_point_viz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("local_waypoint_markers", 5);
//    current_pose_viz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("current_pose_marker", 5);

    ROS_INFO("WayPoints Recieved");

    ros::Duration(2.0).sleep();

    ROS_INFO("Launching Drive Thread");
    std::thread drive_pub_thread(&FollowWaypoints::drive_thread, this);
    drive_pub_thread.detach();

    ROS_INFO("RTDAV_planner node Initialized");
}

/// The scan callback updates the occupancy grid
/// @param scan_msg - pointer to the incoming scan message
void FollowWaypoints::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    ROS_DEBUG("SCAN CALLBACK");
    geometry_msgs::TransformStamped tf_laser_to_map;
    try
    {
        tf_laser_to_map = tf_buffer_.lookupTransform("map", "laser", ros::Time(0));
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.01).sleep();
    }

    const auto translation = tf_laser_to_map.transform.translation;
    const double yaw = tf::getYaw(tf_laser_to_map.transform.rotation);

    const auto start = static_cast<int>(scan_msg->ranges.size()/6);
    const auto end = static_cast<int>(5*scan_msg->ranges.size()/6);
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

        if(x_base_link > local_map_distance_ || y_base_link > local_map_distance_) continue;

        // laser hit x, y in base_link frame
        const double x_map = x_base_link*cos(yaw) - y_base_link*sin(yaw) + translation.x;
        const double y_map = x_base_link*sin(yaw) + y_base_link*cos(yaw) + translation.y;

        std::vector<int> index_of_expanded_obstacles = get_expanded_row_major_indices(x_map, y_map);

        for(const auto& index: index_of_expanded_obstacles)
        {
            if(input_map_.data[index] != 100)
            {
                input_map_.data[index] = 100;
                new_obstacles_.emplace_back(index);
            }
        }
    }

    clear_obstacles_count_++;
    if(clear_obstacles_count_ > 10)
    {
        for(const auto index: new_obstacles_)
        {
            input_map_.data[index] = 0;
        }
        new_obstacles_.clear();
        clear_obstacles_count_ = 0;
        ROS_INFO("Obstacles Cleared");
    }

    dynamic_map_pub_.publish(input_map_);
    ROS_INFO("Map Updated");
}

///// The pose callback when subscribed to particle filter's inferred pose (RRT* Main Loop)
///// @param pose_msg - pointer to the incoming pose message
//void FollowWaypoints::pose_callback(const nav_msgs::OdometryConstPtr &pose_msg)
//{
//    pose_mutex.lock();
//    current_pose_.position.x = pose_msg->pose.pose.position.x;
//    current_pose_.position.y = pose_msg->pose.pose.position.y;
//    current_pose_.position.z = pose_msg->pose.pose.position.z;
//    pose_mutex.unlock();
////    if(visualization_enabled_)
////    {
////        add_way_point_visualization({current_pose_.position.x, current_pose_.position.x},
////                                    &current_pose_viz_pub_, "map", "current_pose", 1, 0.0, 1.0, 0.0,
////                                    1, 0.3, 0.2, 0.2);
////    }
//}

/// Subscribes to the current pose, follows the next waypoint and updates the steering angle accordingly
/// @param pose_msg - Localized Pose of the Robot
void FollowWaypoints::drive_thread()
{
    ros::Duration(2.0).sleep();
    while(ros::ok())
    {
        if (visualization_enabled_ && !visualized_)
        {
            for (int i = 0; i < 3; i++)
            {
                visualize_waypoint_data(way_point_data_[i], &global_way_point_viz_pub_, "map", i);
            }
            visualized_ = true;
        }
        // Transform Points in all racelines to the current frame
        const auto transformed_way_points = transform_waypoints_to_car_frame();

        // Find the best waypoint to track (at lookahead distance) for all racelines
        std::array<int, 3> goal_points_index{};
        std::array<double, 3> current_lookahead{};
        for (int i = 0; i < 3; i++)
        {
            const auto goalpoint_index_and_lookahead = get_global_trackpoint(transformed_way_points[i], lookahead_distance_, i);
            goal_points_index[i] = goalpoint_index_and_lookahead.first;
            current_lookahead[i] = goalpoint_index_and_lookahead.second;
        }

        geometry_msgs::TransformStamped map_to_base_link;
        map_to_base_link = tf_buffer_.lookupTransform("base_link", "map", ros::Time(0));

        std::array<geometry_msgs::Pose, 3> goal_points_car_frame{};
        std::array<geometry_msgs::Pose, 3> goal_points_map_frame{};
        for (int i = 0; i < 3; i++)
        {
            geometry_msgs::Pose goal_way_point;
            goal_way_point.position.x = way_point_data_[i][goal_points_index[i]][0];
            goal_way_point.position.y = way_point_data_[i][goal_points_index[i]][1];
            goal_way_point.position.z = 0;
            goal_way_point.orientation.x = 0;
            goal_way_point.orientation.y = 0;
            goal_way_point.orientation.z = 0;
            goal_way_point.orientation.w = 1;
            goal_points_map_frame[i] = goal_way_point;
            tf2::doTransform(goal_way_point, goal_points_car_frame[i], map_to_base_link);
        }

        // Select One WayPoint
        const auto raceline_index = choose_raceline(goal_points_map_frame, goal_points_car_frame);

        ROS_DEBUG("Following Index %i", raceline_index);

        if (visualization_enabled_)
        {
            for (int i = 0; i < 3; i++)
            {
                if(i != raceline_index)
                {
                    add_way_point_visualization({goal_points_car_frame[i].position.x, goal_points_car_frame[i].position.y},
                                                &local_way_point_viz_pub_, "base_link", "local_waypoint", i, 0.0, 0.0, 1.0,
                                                0.3, 0.2, 0.2, 0.2);
                }
                else
                {
                    add_way_point_visualization({goal_points_car_frame[i].position.x, goal_points_car_frame[i].position.y},
                                                &local_way_point_viz_pub_, "base_link", "local_waypoint", i, 1.0, 0.0, 0.0,
                                                0.3, 0.2, 0.2, 0.2);
                }
            }
        }

        // const double input = solver_.solve_mpc({goal_way_point.position.y, goal_way_point.position.x});

        // Calculate curvature/steering angle
        const double input = 2 * (goal_points_car_frame[raceline_index].position.y) / pow(current_lookahead[raceline_index],2);

        publish_corrected_speed_and_steering(input);
    }
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

    if(steering_angle > 0.15)
    {
        if (steering_angle > 0.25)
        {
            drive_msg.drive.speed = low_speed_;
        }
        else
        {
            drive_msg.drive.speed = medium_speed_;
        }
    }
    else if(steering_angle < -0.15)
    {
        if (steering_angle < -0.25)
        {
            drive_msg.drive.speed = low_speed_;
        }
        else
        {
            drive_msg.drive.speed = medium_speed_;
        }
    }

    std::cout << "speed: " << drive_msg.drive.speed << " ";
    std::cout << "angle: " << drive_msg.drive.steering_angle << "\n";
    drive_pub_.publish(drive_msg);
}

/// Transforms all the waypoints from the map to the car frame
/// @return waypoints transformed in the car frame
std::array<std::vector<std::array<double, 2>>, 3> FollowWaypoints::transform_waypoints_to_car_frame()
{
    geometry_msgs::TransformStamped map_to_base_link;
    map_to_base_link = tf_buffer_.lookupTransform("base_link", "map", ros::Time(0));

    std::array<std::vector<std::array<double, 2>>, 3> transformed_racelines;

    for(int i=0; i<3; i++)
    {
        std::vector<std::array<double, 2>> transformed_waypoints;
        for(const auto& reference_way_point: way_point_data_[i])
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
            transformed_waypoints.emplace_back(transformed_waypoint);
        }
        transformed_racelines[i] = transformed_waypoints;
    }

    return transformed_racelines;
}

/// Returns the Closest Trackpoint at a lookahead distance from the current pose of the vehicle
/// @param waypoints
/// @param lookahead_distance
/// @param i - current raceline index being searched for
/// @return index of the best trackpoint and the lookahead distance
std::pair<size_t, double> FollowWaypoints::get_global_trackpoint(const std::vector<std::array<double, 2>>& waypoints,
        double lookahead_distance, int i)
{
    double closest_distance = std::numeric_limits<double>::max();
    int best_index = -1;
    lookahead_distance =  sqrt(pow(lookahead_distance,2)+pow(std::abs(i - last_raceline_followed_)*1, 2));


    for(size_t i=0; i <waypoints.size(); ++i)
    {
        if(waypoints[i][0] < lookahead_distance/2) continue;
        double distance = sqrt(waypoints[i][0]*waypoints[i][0] +
                                       waypoints[i][1]*waypoints[i][1]);
        double lookahead_diff = std::abs(distance - lookahead_distance);
        if(lookahead_diff < closest_distance)
        {
            closest_distance = lookahead_diff;
            best_index = i;
        }
    }

    return {best_index, lookahead_distance};
}

/// Returns the row major indeices for the map of an inflated area around a point based on inflation radius
/// @param x_map - x coordinates in map frame
/// @param y_map - y coordinates in map frame
/// @return row major index of the map
std::vector<int> FollowWaypoints::get_expanded_row_major_indices(const double x_map, const double y_map)
{
    std::vector<int> expanded_row_major_indices;
    const auto x_index = static_cast<int>((x_map - input_map_.info.origin.position.x)/input_map_.info.resolution);
    const auto y_index = static_cast<int>((y_map - input_map_.info.origin.position.y)/input_map_.info.resolution);
    for(int i=-inflation_radius_+x_index; i<inflation_radius_+1+x_index; i++)
    {
        for(int j=-inflation_radius_+y_index; j<inflation_radius_+1+y_index; j++)
        {
            expanded_row_major_indices.emplace_back(j*map_cols_ + i);
        }
    }
    return expanded_row_major_indices;
}

/// Returns the row major index for the map
/// @param x_map - x coordinates in map frame
/// @param y_map - y coordinates in map frame
/// @return row major index of the map
int FollowWaypoints::get_row_major_index(const double x_map, const double y_map)
{
    const auto x_index = static_cast<int>((x_map - input_map_.info.origin.position.x)/input_map_.info.resolution);
    const auto y_index = static_cast<int>((y_map - input_map_.info.origin.position.y)/input_map_.info.resolution);
    return y_index*map_cols_ + x_index;
}

/// Chooses a the best raceline to follow
/// @param racelines_map_frame
/// @return
int FollowWaypoints::choose_raceline(const std::array<geometry_msgs::Pose, 3>& goalpoints_map_frame,
                                     const std::array<geometry_msgs::Pose, 3>& goalpoints_car_frame)
{
    auto is_collided = [&](const geometry_msgs::Pose& waypoint){
//        for(int i=1; i<10; i++)
//        {
//            pose_mutex.lock();
//            const double x = (1-0.1)*i*(current_pose_.position.x) + 0.1*i*(waypoint.position.x);
//            const double y = (1-0.1)*i*(current_pose_.position.y) + 0.1*i*(waypoint.position.y);
//            pose_mutex.unlock();
//            const auto index = get_row_major_index(x, y);
//            if(input_map_.data[index] == 100)
//            {
//                return true;
//            }
//
//        }
        const auto index = get_row_major_index(waypoint.position.x, waypoint.position.y);
        if(input_map_.data[index] == 100)
        {
            return true;
        }
        return false;
    };

    auto calculate_input = [&](int raceline_index){
        return 2*(goalpoints_car_frame[raceline_index].position.y) / (lookahead_distance_ * lookahead_distance_);
    };

    if(last_raceline_followed_ == 0)
    {
        if(!is_collided(goalpoints_map_frame[0]))
        {
            return 0;
        }

        double input1 = calculate_input(1);
        double input2 = calculate_input(2);
        if(input1 < input2 && !is_collided(goalpoints_map_frame[1]))
        {
            last_raceline_followed_ = 1;
            return 1;
        }
        else if(!is_collided(goalpoints_map_frame[2]))
        {
            last_raceline_followed_ = 2;
            return 2;
        }

        ROS_WARN("Not able to decide a safe raceline. Choosing outer raceline %i", 2);
        return 2;

    }
    else if(last_raceline_followed_ == 1)
    {
        if(!is_collided(goalpoints_map_frame[1]))
        {
            return 1;
        }

        double input0 = calculate_input(0);
        double input2 = calculate_input(2);

        if(input0 < input2 && !is_collided(goalpoints_map_frame[0]))
        {
            last_raceline_followed_ = 0;
            return 0;
        }
        else if(!is_collided(goalpoints_map_frame[2]))
        {
            last_raceline_followed_ = 2;
            return 2;
        }
        ROS_WARN("Not able to decide a safe raceline. Choosing outer raceline %i", 2);
        return 2;
    }
    else if(last_raceline_followed_ == 2)
    {
        if(!is_collided(goalpoints_map_frame[2]))
        {
            return 2;
        }
        double input1 = calculate_input(1);
        double input0 = calculate_input(0);
        if(input1 < input0 && !is_collided(goalpoints_map_frame[1]))
        {
            last_raceline_followed_ = 1;
            return 1;
        }
        else if(!is_collided(goalpoints_map_frame[0]))
        {
            last_raceline_followed_ = 0;
            return 0;
        }
        ROS_WARN("Not able to decide a safe raceline. Choosing center raceline %i", 1);
        return 1;
    }
    else
    {
        ROS_ERROR("No Last Raceline Followed/ Invalid Raceline %i", last_raceline_followed_);
    }
}

