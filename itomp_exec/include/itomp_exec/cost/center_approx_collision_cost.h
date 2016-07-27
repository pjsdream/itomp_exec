#ifndef ITOMP_EXEC_CENTER_APPROX_COLLISION_COST
#define ITOMP_EXEC_CENTER_APPROX_COLLISION_COST


#include <itomp_exec/cost/cost.h>


namespace itomp_exec
{

class CenterApproxCollisionCost : public Cost
{
public:
        
    CenterApproxCollisionCost(ITOMPOptimizer& optimizer, double weight = 1.0);

    inline virtual std::string getString()
    {
        return "CenterApproxCollisionCost";
    }

    virtual void addCost();

private:
};

}


#endif // ITOMP_EXEC_CENTER_APPROX_COLLISION_COST
