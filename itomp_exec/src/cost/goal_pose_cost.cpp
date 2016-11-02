#include <itomp_exec/cost/goal_pose_cost.h>


namespace itomp_exec
{

GoalPoseCost::GoalPoseCost(double weight)
    : Cost(weight)
{
}

void GoalPoseCost::printInfo()
{
    ROS_INFO("Type: goal pose cost");
    ROS_INFO("Weight: %lf", weight_);
}

}
