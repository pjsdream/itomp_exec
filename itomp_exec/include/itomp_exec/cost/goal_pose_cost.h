#ifndef ITOMP_EXEC_GOAL_POSE_COST_H
#define ITOMP_EXEC_GOAL_POSE_COST_H


#include <itomp_exec/cost/cost.h>


namespace itomp_exec
{

class GoalPoseCost : public Cost
{
    ITOMP_COST_DERIVED_CLASS_DECL(GoalPose)
    
public:
        
    GoalPoseCost(double weight = 1.0);
    
    virtual void initialize(const ITOMPPlannerNode& planner_node);
    
    virtual double cost(const Trajectory& trajectory);
    virtual TrajectoryDerivative derivative(const Trajectory& trajectory);
    
private:
    
    // pair of link index (optimization) and 
    std::vector<std::pair<int, Eigen::Vector3d> > goal_link_positions_;
    std::vector<std::pair<int, Eigen::Quaterniond> > goal_link_orientations_;
};

}


#endif // ITOMP_EXEC_GOAL_POSE_COST_H
