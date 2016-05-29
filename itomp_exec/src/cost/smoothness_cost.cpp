#include <itomp_exec/cost/smoothness_cost.h>


namespace itomp_exec
{

SmoothnessCost::SmoothnessCost(double weight)
    : Cost(weight)
{
}

double SmoothnessCost::cost(TrajectoryConstPtr trajectory)
{
    return 0.0;
}

TrajectoryDerivative SmoothnessCost::derivative(const TrajectoryConstPtr trajectory)
{
    TrajectoryDerivative derivative;
    return derivative;
}

}
