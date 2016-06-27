#include <itomp_exec/cost/cost.h>
#include <itomp_exec/optimization/itomp_optimizer.h>


namespace itomp_exec
{

Cost::Cost(ITOMPOptimizer& optimizer, double weight)
    : optimizer_(optimizer)
    , weight_(weight)
{
}

void Cost::addCost()
{
}

void Cost::addDerivative()
{
}

}
