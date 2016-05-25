#include <itomp_exec/cost/cost.h>


namespace itomp_exec
{

// TrajectoryDerivative
TrajectoryDerivative::TrajectoryDerivative()
{
}

// Cost
Cost::Cost()
{
}

double Cost::cost()
{
    return 0.0;
}

TrajectoryDerivative Cost::derivative()
{
    TrajectoryDerivative derivative;
    return derivative;
}

}
