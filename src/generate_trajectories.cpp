#include <vector>
#include <array>
#include <fstream>
#include <ros/ros.h>
#include <iostream>

#include <eigen3/Eigen/Dense>

#include "cvxgen/solver.h"
#include "mpc_av/mpc_solver.h"

/// Global Variable for CVXGEN solvers
Vars vars;
Params params;
Workspace work;
Settings settings;

static const double x_min = 1.0;
static const double x_max = 2.5;
static const double y_min =-1.5;
static const double y_max = 1.5;
static const double x_dt = 0.2;
static const double y_dt = 0.1;
static const int n_samples = 10;
static const std::vector<double> theta_range = {-0.2,-0.15,-0.1,-0.05, 0.0, 0.05, 0.1, 0.15, 0.2};

/// Format of CSV Generated
/// x_curr, y_curr,
/// input_steering_angle,
/// x1, y1, x2, y2, ...., x10, y10

/// Do Parabolic Curve Fitting
/// @param x - input x's
/// @param y - input y's
/// @return a, b, c where a*x^2 + b*x + c
std::array<double, 4> polyRegression(const std::vector<double>& x, const std::vector<double>& y)
{
    Eigen::MatrixXd X(x.size(), 4);
    Eigen::VectorXd Y(y.size());
    for(int i=0; i<x.size(); i++)
    {
        X(i, 0) = x[i]*x[i]*x[i];
        X(i, 1) = x[i]*x[i];
        X(i, 2) = x[i];
        X(i, 3) = 1;
        Y(i) = y[i];
    }
    Eigen::MatrixXd Xt = X.transpose();
    Eigen::Vector4d W = (Xt*X).inverse()*(Xt)*Y;
    return std::array<double, 4>{W(0), W(1), W(2), W(3)};
}

/// Get Equidistant Samples
/// @param x - Non Equidistant Samples
/// @param y - Non Equidistant Samples
/// @return Equidistant Samples of (x, y)
std::array<std::array<double, 2>, n_samples> get_equidistant_samples(const std::vector<double>& x, const std::vector<double>& y)
{
    const auto weights = polyRegression(x, y);
    std::array<std::array<double, 2>, n_samples> equidistant_samples{};

    const double xdt = x[x.size()-1]/n_samples;
    for(int i=1; i<n_samples+1; i++)
    {
        const double sample_x = i*xdt;
        equidistant_samples[i-1][0] = sample_x;
        equidistant_samples[i-1][1] = weights[0]*pow(sample_x, 3) + weights[1]*pow(sample_x, 2) + weights[2]*sample_x + weights[3];
    }

    return equidistant_samples;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "generate_trajectories");
    ros::NodeHandle nh;

    double B;
    nh.getParam("B", B);

    std::array<double, 4> Q{};
    nh.getParam("Q0", Q[0]);
    nh.getParam("Q1", Q[1]);
    nh.getParam("Q2", Q[2]);
    nh.getParam("Q3", Q[3]);

    std::array<double, 4> Qf{};
    nh.getParam("Q0", Qf[0]);
    nh.getParam("Q1", Qf[1]);
    nh.getParam("Q2", Qf[2]);
    nh.getParam("Q3", Qf[3]);

    std::array<double, 4> R{};
    nh.getParam("Q0", R[0]);
    nh.getParam("Q1", R[1]);
    nh.getParam("Q2", R[2]);
    nh.getParam("Q3", R[3]);

    double S;
    nh.getParam("S", S);

    std::ofstream trajectories_file;
    trajectories_file.open("/home/yash/yasht_ws/src/mpc_av/trajectories.csv");

    double x_curr = x_min;
    double y_curr = y_min;

    mpc::MPCSolver solver(Q, Qf, R, B, S);
    settings.verbose = 0;
    settings.eps = 1e-4;

    while(x_curr < x_max)
    {
        while(y_curr < y_max)
        {
            for(const auto& theta: theta_range)
            {
                const std::array<double, 3> current_goal{y_curr, x_curr, theta};
                const double input = solver.solve_mpc(current_goal);
                trajectories_file << x_curr << ", " << y_curr << ", " << theta << ", \n";
                trajectories_file << input << ", \n";
                std::vector<double> x;
                std::vector<double> y;
                x.emplace_back(0);
                y.emplace_back(0);
                for(int i=1; i<11; i++)
                {
                    x.emplace_back(vars.x[i][1]);
                    y.emplace_back(vars.x[i][0]);
                }
                const auto equidistant_samples = get_equidistant_samples(x, y);
                for(const auto& sample: equidistant_samples)
                {
                    trajectories_file << sample[0] << ", " << sample[1] << ", ";
                }

                trajectories_file << "\n";
            }
            y_curr = y_curr + y_dt;
        }
        y_curr = y_min;
        x_curr = x_curr + x_dt;
    }

    trajectories_file.close();
}

