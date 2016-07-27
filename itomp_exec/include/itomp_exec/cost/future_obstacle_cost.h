#ifndef ITOMP_EXEC_FUTURE_OBSTACLE_COST
#define ITOMP_EXEC_FUTURE_OBSTACLE_COST


#include <itomp_exec/cost/cost.h>
#include <pcml/FutureObstacleDistributions.h>


namespace itomp_exec
{

class FutureObstacleCost : public Cost
{
public:
        
    FutureObstacleCost(ITOMPOptimizer& optimizer, double weight = 1.0);

    inline virtual std::string getString()
    {
        return "FutureObstacleCost";
    }

    virtual void addCost();
    virtual void addDerivative();

private:
    
};

}


#endif // ITOMP_EXEC_FUTURE_OBSTACLE_COST
