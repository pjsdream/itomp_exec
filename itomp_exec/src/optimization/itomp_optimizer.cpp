#include <itomp_exec/optimization/itomp_optimizer.h>


namespace itomp_exec
{

ITOMPOptimizer::ITOMPOptimizer(const TrajectoryPtr& trajectory)
    : trajectory_(trajectory)
{
}

ITOMPOptimizer::~ITOMPOptimizer()
{
    trajectory_.reset();
}

void ITOMPOptimizer::optimize()
{
}

}
