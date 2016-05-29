#include <itomp_exec/trajectory/trajectory.h>


namespace itomp_exec
{

// Trajectory
Trajectory::Trajectory()
{
}

void Trajectory::setOptimizationVariables(const Eigen::VectorXd& variables)
{
}

Eigen::VectorXd Trajectory::getOptimizationVariables()
{
    Eigen::VectorXd variables;
    return variables;
}

Eigen::VectorXd Trajectory::getOptimizationVariableLowerLimits()
{
    Eigen::VectorXd lower_limits;
    return lower_limits;
}

Eigen::VectorXd Trajectory::getOptimizationVariableUpperLimits()
{
    Eigen::VectorXd upper_limits;
    return upper_limits;
}

// TrajectoryDerivative
TrajectoryDerivative::TrajectoryDerivative()
{
}

TrajectoryDerivative& TrajectoryDerivative::operator += (const TrajectoryDerivative& rhs)
{
    return *this;
}

Eigen::VectorXd TrajectoryDerivative::getOptimizationVariables()
{
    Eigen::VectorXd variables;
    return variables;
}

}
