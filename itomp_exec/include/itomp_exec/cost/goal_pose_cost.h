#ifndef ITOMP_EXEC_GOAL_POSE_COST_H
#define ITOMP_EXEC_GOAL_POSE_COST_H


#include <itomp_exec/cost/cost.h>
#include <moveit/robot_state/robot_state.h>


namespace itomp_exec
{

class GoalPoseCost : public Cost
{
private:

    static const double ratio_cosine_to_meter_;
    static const double ratio_radian_per_sec_to_meter_;
    
public:
        
    GoalPoseCost(ITOMPOptimizer& optimizer, double weight = 1.0);

    virtual void addCost();
    virtual void addDerivative();

private:
};

}


#endif // ITOMP_EXEC_GOAL_POSE_COST_H
