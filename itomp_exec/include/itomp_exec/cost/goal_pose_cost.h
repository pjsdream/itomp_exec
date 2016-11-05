#ifndef ITOMP_EXEC_GOAL_POSE_COST_H
#define ITOMP_EXEC_GOAL_POSE_COST_H


#include <itomp_exec/cost/cost.h>

#include <Eigen/Dense>


namespace itomp_exec
{

class GoalPoseCost : public Cost
{
public:

    GoalPoseCost(double weight = 1.);

    virtual void printInfo();

    void setGoalPose(const std::string& link_name, const Eigen::Affine3d& offset, const Eigen::Affine3d& goal_pose);

private:

    std::string link_name_;
    Eigen::Affine3d offset_;
    Eigen::Affine3d goal_pose_;
};

}


#endif // ITOMP_EXEC_GOAL_POSE_COST_H
