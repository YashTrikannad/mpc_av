//
// Created by yash on 10/20/19.
//

#ifndef SRC_CSV_READER_H
#define SRC_CSV_READER_H

#include <ros/ros.h>

#include <fstream>
#include <utility>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>

struct trajectory
{
    double goal_x, goal_y, goal_heading;
    double steering_input;
    std::vector<geometry_msgs::Pose> states;
};

/// A Class to read csv data files
class CSVReader
{
    std::string fileName;
    std::string delimeter;

public:
    explicit CSVReader(std::string filename, std::string delm = ",") :
            fileName(std::move(filename)), delimeter(std::move(delm))
    {}

    ///
    /// Function to fetch data from a CSV WayPoints File
    std::vector<std::array<double, 2>> getData()
    {
        std::ifstream file(fileName);
        if (!file)
        {
            throw std::runtime_error("Invalid Path for csv file.");
        }
        std::vector<std::array<double, 2>> dataList;

        std::string line = "";
        // Iterate through each line and split the content using delimeter
        while (getline(file, line))
        {
            std::vector<std::string> vec;
            boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
            std::array<double, 2> trackpoint{};
            trackpoint[0] = std::stod(vec[0]);
            trackpoint[1] = std::stod(vec[1]);

            dataList.emplace_back(trackpoint);
        }
        // Close the File
        file.close();

        return dataList;
    }

    ///
    /// Function to fetch data from a CSV Trajectories File
    std::vector<trajectory> getTrajectoriesData()
    {
        std::ifstream file(fileName);
        if (!file)
        {
            throw std::runtime_error("Invalid Path for csv file.");
        }
        std::vector<trajectory> dataList;

        std::string line = "";
        // Iterate through each line and split the content using delimeter
        while (getline(file, line))
        {
            std::vector<std::string> goal_vec;
            boost::algorithm::split(goal_vec, line, boost::is_any_of(delimeter));

            trajectory curr_traj;
            curr_traj.goal_x = std::stod(goal_vec[0]);
            curr_traj.goal_y = std::stod(goal_vec[1]);
            curr_traj.goal_heading = std::stod(goal_vec[2]);

            line = "";
            getline(file, line);
            std::vector<std::string> input_vec;
            boost::algorithm::split(input_vec, line, boost::is_any_of(delimeter));
            curr_traj.steering_input = std::stod(input_vec[0]);

            line = "";
            getline(file, line);
            std::vector<std::string> state_vec;
            boost::algorithm::split(state_vec, line, boost::is_any_of(delimeter));
            for(int i=0; i<state_vec.size()-1; i=i+2)
            {
                geometry_msgs::Pose traj_point;
                traj_point.position.x = std::stod(state_vec[i]);
                traj_point.position.y = std::stod(state_vec[i+1]);
                traj_point.position.z = 0;
                traj_point.orientation.x = 0;
                traj_point.orientation.y = 0;
                traj_point.orientation.z = 0;
                traj_point.orientation.w = 1;
                curr_traj.states.emplace_back(traj_point);
            }
            dataList.emplace_back(curr_traj);
        }

        // Close the File
        file.close();

        return dataList;
    }
};

#endif //SRC_CSV_READER_H
