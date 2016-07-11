#include <itomp_exec/cost/time_cost.h>
#include <itomp_exec/optimization/itomp_optimizer.h>


namespace itomp_exec
{

TimeCost::TimeCost(ITOMPOptimizer& optimizer, double weight)
    : Cost(optimizer, weight)
{
}

void TimeCost::addCost()
{
    ITOMPOptimizer& optimizer = getOptimizer();
    const double weight = getWeight();

    double& cost = optimizer.cost();

}

void TimeCost::addDerivative()
{
    ITOMPOptimizer& optimizer = getOptimizer();
    const double weight = getWeight();

    Eigen::MatrixXd& milestone_derivative = optimizer.milestoneDerivative();
}

}
