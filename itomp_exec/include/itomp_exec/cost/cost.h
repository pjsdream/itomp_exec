#ifndef ITOMP_EXEC_COST_H
#define ITOMP_EXEC_COST_H


/** To add new cost class,
 *  1. Derive Cost class
 *  2. At the beginning of the class definition, add ITOMP_COST_DERIVED_CLASS_DECL(type)
 *     e.g.,
 *     class CollisionCost : public Cost
 *     {
 *         ITOMP_COST_DERIVED_CLASS_DECL(Collision)
 *         ...
 *     };
 *  3. Add an object generating line in cost_factory.cpp
 */


#include <itomp_exec/trajectory/trajectory.h>


// Cost derived class 
#define ITOMP_COST_DERIVED_CLASS_DECL(type) \
        friend class CostFactory; \
    public: \
        virtual inline std::string getDescription() { return getDescription_(); } \
    private: \
        static inline std::string getDescription_() { return std::string(#type) + std::string("Cost"); }

namespace itomp_exec
{

class ITOMPPlannerNode;

class Cost
{
private:
    
    static const double derivative_eps;
    
public:
    
    Cost(double weight = 1.0);
    
    virtual std::string getDescription() { return "Cost"; }
    
    virtual void initialize(const ITOMPPlannerNode& planner_node);
    
    virtual double cost(const Trajectory& trajectory);
    virtual TrajectoryDerivative derivative(const Trajectory& trajectory);
    
protected:
    
    double weight_;
    
private:
};

}


#endif // ITOMP_EXEC_COST_H
