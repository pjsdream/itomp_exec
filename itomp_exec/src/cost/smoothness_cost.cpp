#include <itomp_exec/cost/smoothness_cost.h>


namespace itomp_exec
{

SmoothnessCost::SmoothnessCost(double weight)
    : Cost(weight)
{
}

void SmoothnessCost::printInfo()
{
    ROS_INFO("Type: smoothness cost");
    ROS_INFO("Weight: %lf", weight_);
}

}
