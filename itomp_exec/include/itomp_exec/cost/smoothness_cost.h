#ifndef ITOMP_EXEC_SMOOTHNESS_COST_H
#define ITOMP_EXEC_SMOOTHNESS_COST_H


#include <itomp_exec/cost/cost.h>
#include <ecl/geometry/polynomial.hpp>


namespace itomp_exec
{

class SmoothnessCost : public Cost
{
    ITOMP_COST_DERIVED_CLASS_DECL(Smoothness)
    
private:
        
    static std::vector<double> gaussian_quadrature2_weights_;
    static std::vector<double> gaussian_quadrature2_abscissa_;
    
public:
        
    SmoothnessCost(double weight = 1.0);
    
    virtual void initialize(const ITOMPPlannerNode& planner_node);
    
    virtual double cost(const Trajectory& trajectory);
    virtual TrajectoryDerivative derivative(const Trajectory& trajectory);
    
private:
    
    double gaussianQuadratureQuadraticPolynomial(double t0, double t1, const ecl::QuadraticPolynomial& poly);
    
    // for gradient computation
    // (1/d^3) D H2 D [p0 v0 p1 v1]^T
    Eigen::Matrix4d H2_;
};

}


#endif // ITOMP_EXEC_SMOOTHNESS_COST_H
