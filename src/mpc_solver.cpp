//
// Created by yash on 11/15/19.
//

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "mpc_av/mpc_solver.h"

mpc::MPCSolver::MPCSolver(const std::array<double, 4>& Q,
                          const std::array<double, 4>& Qf,
                          const std::array<double, 4>& R,
                          double B,
                          double S)
{
    set_defaults();
    setup_indexing();
    params.A[0] = 1;
    params.A[1] = 0;
    params.A[2] = 0;
    params.A[3] = 1;
    params.B[0] = B;
    params.Q[0] = Q[0];
    params.Q[1] = Q[1];
    params.Q[2] = Q[2];
    params.Q[3] = Q[3];
    params.Q_final[0] = Qf[0];
    params.Q_final[1] = Qf[1];
    params.Q_final[2] = Qf[2];
    params.Q_final[3] = Qf[3];
    params.R[0] = R[0];
    params.R[1] = R[1];
    params.R[2] = R[2];
    params.R[3] = R[3];
    params.S[0] = S;
    params.x_0[0] = 0;
    params.x_0[1] = 0;
    params.u_max[0] = 0.4189;
    params.u_max[1] = 5.0;
    ROS_DEBUG("Model Loaded");
}


double mpc::MPCSolver::solve_mpc(const std::array<double, 3>& required_goalpoint)
{
    update_model(required_goalpoint);
    solve();
    if(!work.converged)
    {
        ROS_ERROR("MPC Solution did not converge");
    }
    ROS_DEBUG("Model Solved");

//    std::cout << "Final Position: " << vars.x_11[0] << " " << vars.x_11[1] << std::endl;
//    std::cout << "Input at T = 1: " << *vars.u_1 << std::endl;
//    std::cout << "*vars.u_1[1]" << vars.u_1[1] << std::endl;

    return *vars.u_0;
}

void mpc::MPCSolver::update_model(const std::array<double, 3>& required_goalpoint)
{
    params.w[0] = required_goalpoint[0];
    params.w[1] = required_goalpoint[1];
    params.u_f[0] = required_goalpoint[2];
    ROS_DEBUG("Model Updated");
}
