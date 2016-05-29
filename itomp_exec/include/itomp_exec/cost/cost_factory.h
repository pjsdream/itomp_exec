#ifndef ITOMP_EXEC_COST_FACTORY_H
#define ITOMP_EXEC_COST_FACTORY_H


#include <itomp_exec/cost/cost.h>


namespace itomp_exec
{

class CostFactory
{
public:
    
    static Cost* newCost(const std::string& name, double weight);
};

}

#endif // ITOMP_EXEC_COST_FACTORY_H
