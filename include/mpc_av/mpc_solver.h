//
// Created by yash on 11/15/19.
//

#ifndef SRC_MPC_SOLVER_H
#define SRC_MPC_SOLVER_H

#include <geometry_msgs/TransformStamped.h>

#include "cvxgen/solver.h"

namespace mpc
{

class MPCSolver
{
public:
    MPCSolver(const std::array<double, 4>& Q,
              const std::array<double, 4>& Qf,
              const std::array<double, 4>& R,
              double B,
              double S);

    /// Solves the mpc
    /// @param required_goalwaypoint (y, x) in car's frame
    /// @return steering angle
    double solve_mpc(const std::array<double, 3>& required_goalwaypoint);

private:

    /// Loads the model parameters into the mpc solver
    void update_model(const std::array<double, 3>& required_goalpoint);
};

} // mpc
#endif //SRC_MPC_SOLVER_H