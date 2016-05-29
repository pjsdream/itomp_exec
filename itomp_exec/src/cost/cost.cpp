#include <itomp_exec/cost/cost.h>


namespace itomp_exec
{

Cost::Cost(double weight)
    : weight_(weight)
{
}

double Cost::cost(TrajectoryConstPtr trajectory)
{
    return 0.0;
}

TrajectoryDerivative Cost::derivative(TrajectoryConstPtr trajectory)
{
    TrajectoryDerivative derivative;
    return derivative;
}

}
