#ifndef ITOMP_EXEC_GOAL_POSE_COST_H
#define ITOMP_EXEC_GOAL_POSE_COST_H


#include <itomp_exec/cost/cost.h>


namespace itomp_exec
{

class GoalPoseCost : public Cost
{
public:

    GoalPoseCost(double weight = 1.);

    virtual void printInfo();

private:
};

}


#endif // ITOMP_EXEC_GOAL_POSE_COST_H
