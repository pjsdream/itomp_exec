#ifndef ITOMP_EXEC_COST_FACTORY_H
#define ITOMP_EXEC_COST_FACTORY_H


#include <itomp_exec/cost/cost.h>


namespace itomp_exec
{

class CostFactory
{
public:
    
    static Cost* newCost(const std::string& name, double weight);
    
    static std::string camelCasedToUnderscored(const std::string& name);
    
    static inline bool isUpper(char x)
    {
        return 'A' <= x && x <= 'Z';
    }
    
    static inline char toLower(char x)
    {
        return isUpper(x) ? x-'A'+'a' : x;
    }
};

}

#endif // ITOMP_EXEC_COST_FACTORY_H
