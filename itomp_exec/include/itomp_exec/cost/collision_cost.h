#ifndef ITOMP_EXEC_COLLISION_COST
#define ITOMP_EXEC_COLLISION_COST


#include <itomp_exec/cost/cost.h>


namespace itomp_exec
{

class CollisionCost : public Cost
{
    ITOMP_COST_DERIVED_CLASS_DECL(Collision)
    
public:
        
    CollisionCost(double weight = 1.0);
    
private:
};

}


#endif // ITOMP_EXEC_COLLISION_COST
