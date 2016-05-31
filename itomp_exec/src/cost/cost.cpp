#include <itomp_exec/cost/cost.h>


namespace itomp_exec
{

const double Cost::derivative_eps = 1e-7;


Cost::Cost(double weight)
    : weight_(weight)
{
}

void Cost::initialize(const ITOMPPlannerNode& planner_node)
{
}

double Cost::cost(const Trajectory& trajectory)
{
    return 0.0;
}

TrajectoryDerivative Cost::derivative(const Trajectory& trajectory)
{
    return TrajectoryDerivative(trajectory);
}

}
