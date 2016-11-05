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

void GoalPoseCost::setGoalPose(const std::string& link_name, const Eigen::Affine3d& offset, const Eigen::Affine3d& goal_pose)
{
    link_name_ = link_name;
    offset_ = offset;
    goal_pose_ = goal_pose;
}

}
