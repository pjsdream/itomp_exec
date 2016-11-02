#include <itomp_exec/cost/collision_cost.h>


namespace itomp_exec
{

CollisionCost::CollisionCost(double weight)
    : Cost(weight)
{
}

void CollisionCost::printInfo()
{
    ROS_INFO("Type: collision cost");
    ROS_INFO("Weight: %lf", weight_);
}

}
