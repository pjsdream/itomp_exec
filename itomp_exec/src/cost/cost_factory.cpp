/** To add new cost class,
 *  1. Include header file
 *  2. Add a ITOMP_COST_DERIVED_CLASS_FACTORY line in CostFactory::newCost() function
 */

#include <itomp_exec/cost/cost_factory.h>
#include <ros/console.h>

// include cost class
#include <itomp_exec/cost/collision_cost.h>
#include <itomp_exec/cost/goal_pose_cost.h>
#include <itomp_exec/cost/smoothness_cost.h>


#define ITOMP_COST_DERIVED_CLASS_FACTORY(type) \
    { \
        if (camelCasedToUnderscored( type##Cost::getDescription_() ) == name + "_cost") return new type##Cost(weight); \
    }


namespace itomp_exec
{

Cost* CostFactory::newCost(const std::string& name, double weight)
{
    // add lines for object generation
    ITOMP_COST_DERIVED_CLASS_FACTORY(Collision);
    ITOMP_COST_DERIVED_CLASS_FACTORY(GoalPose);
    ITOMP_COST_DERIVED_CLASS_FACTORY(Smoothness);
    
    ROS_ERROR("Unknown cost function type [%s]", name.c_str());
    
    return 0;
}

std::string CostFactory::camelCasedToUnderscored(const std::string& name)
{
    std::string result;
    
    for (int i=0; i<name.size(); i++)
    {
        if (i && isUpper(name[i]))
            result += std::string("_") + toLower(name[i]);
        else
            result += toLower(name[i]);
    }
    
    return result;
}

}
