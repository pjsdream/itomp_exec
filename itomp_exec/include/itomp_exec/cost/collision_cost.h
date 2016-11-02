#ifndef ITOMP_EXEC_COLLISION_COST
#define ITOMP_EXEC_COLLISION_COST


#include <itomp_exec/cost/cost.h>


namespace itomp_exec
{

class CollisionCost : public Cost
{
public:

    CollisionCost(double weight = 1.);

    virtual void printInfo();

private:
};

}


#endif // ITOMP_EXEC_COLLISION_COST
