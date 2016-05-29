/** To add new cost class,
 *  1. Include header file
 *  2. Add a ITOMP_COST_DERIVED_CLASS_FACTORY line in CostFactory::newCost() function
 */

#include <itomp_exec/cost/cost_factory.h>
#include <ros/console.h>

// include cost class
#include <itomp_exec/cost/collision_cost.h>
#include <itomp_exec/cost/goal_cost.h>
#include <itomp_exec/cost/smoothness_cost.h>


#define ITOMP_COST_DERIVED_CLASS_FACTORY(type) \
    { \
        std::string class_name = type##Cost::getDescription_(); \
        std::transform(class_name.begin(), class_name.end(), class_name.begin(), ::tolower); \
        std::string requested_name = name; \
        std::transform(requested_name.begin(), requested_name.end(), requested_name.begin(), ::tolower); \
        if (class_name == name + "cost") return new type##Cost(weight); \
    }


namespace itomp_exec
{

Cost* CostFactory::newCost(const std::string& name, double weight)
{
    // add lines for object generation
    ITOMP_COST_DERIVED_CLASS_FACTORY(Collision);
    ITOMP_COST_DERIVED_CLASS_FACTORY(Goal);
    ITOMP_COST_DERIVED_CLASS_FACTORY(Smoothness);
    
    ROS_ERROR("Unknown cost function type [%s]", name.c_str());
    
    return 0;
}

}
