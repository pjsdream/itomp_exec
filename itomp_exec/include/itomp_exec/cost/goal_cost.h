#ifndef ITOMP_EXEC_GOAL_COST_H
#define ITOMP_EXEC_GOAL_COST_H


#include <itomp_exec/cost/cost.h>


namespace itomp_exec
{

class GoalCost : public Cost
{
    ITOMP_COST_DERIVED_CLASS_DECL(Goal)
    
public:
        
    GoalCost(double weight = 1.0);
    
private:
};

}


#endif // ITOMP_EXEC_GOAL_COST_H
