#ifndef ITOMP_EXEC_COLLISION_COST
#define ITOMP_EXEC_COLLISION_COST


#include <itomp_exec/cost/cost.h>


namespace itomp_exec
{

class CollisionCost : public Cost
{
public:
        
    CollisionCost(ITOMPOptimizer& optimizer, double weight = 1.0);

    inline virtual std::string getString()
    {
        return "CollisionCost";
    }

    virtual void addCost();
    virtual void addDerivative();

private:
};

}


#endif // ITOMP_EXEC_COLLISION_COST
