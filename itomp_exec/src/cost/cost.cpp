#include <itomp_exec/cost/cost.h>
#include <itomp_exec/optimization/itomp_optimizer.h>


namespace itomp_exec
{

Cost::Cost(double weight)
    : weight_(weight)
{
}

void Cost::printInfo()
{
    ROS_INFO("Type: N/A");
    ROS_INFO("Weight: %lf", weight_);
}

}
