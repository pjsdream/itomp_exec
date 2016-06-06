#ifndef ITOMP_EXEC_FUTURE_OBSTACLE_COST
#define ITOMP_EXEC_FUTURE_OBSTACLE_COST


#include <itomp_exec/cost/cost.h>
#include <pcml/FutureObstacleDistributions.h>


namespace itomp_exec
{

class FutureObstacleCost : public Cost
{
    ITOMP_COST_DERIVED_CLASS_DECL(FutureObstacle)
    
public:
        
    FutureObstacleCost(double weight = 1.0);
    
    virtual void initialize(const ITOMPPlannerNode& planner_node);
    
    virtual double cost(const Trajectory& trajectory);
    
private:
    
};

}


#endif // ITOMP_EXEC_FUTURE_OBSTACLE_COST
