#include <itomp_exec/cost/smoothness_cost.h>
#include <itomp_exec/planner/itomp_planner_node.h>
#include <ecl/geometry.hpp>
#include <ecl/containers.hpp>


namespace itomp_exec
{

std::vector<double> SmoothnessCost::gaussian_quadrature2_weights_ =
{
    0.8888888888888888,
    0.5555555555555556,
    0.5555555555555556,
};

std::vector<double> SmoothnessCost::gaussian_quadrature2_abscissa_ =
{
    0.0000000000000000,
    -0.7745966692414834,
    0.7745966692414834,
};

SmoothnessCost::SmoothnessCost(double weight)
    : Cost(weight)
{
}

void SmoothnessCost::initialize(const ITOMPPlannerNode& planner_node)
{
    // for gradient computation
    H2_.setZero();
    for (int i=0; i<3; i++)
    {
        const double& w = gaussian_quadrature2_weights_[i];
        const double& t = 0.5 + 0.5 * gaussian_quadrature2_abscissa_[i];
        const Eigen::Vector4d h(12.*t - 6., 6.*t - 4., -12.*t + 6., 6.*t - 2.);
        H2_ += w * h * h.transpose();
    }
}

double SmoothnessCost::cost(const Trajectory& trajectory)
{
    double cost = 0.;
    
    const Eigen::MatrixXd& milestone_variables = trajectory.getMilestoneVariables();
    const Eigen::VectorXd& milestone_start_variables = trajectory.getMilestoneStartVariables();
    const int num_milestones = milestone_variables.cols();
    const int num_rows = milestone_variables.rows();
    
    for (int i=0; i<num_milestones; i++)
    {
        const Eigen::VectorXd& v0 = i==0 ? milestone_start_variables : milestone_variables.col(i-1);
        const Eigen::VectorXd& v1 = milestone_variables.col(i);
        const double t0 = trajectory.getMilestoneTimeFromIndex(i-1);
        const double t1 = trajectory.getMilestoneTimeFromIndex(i);
        
        for (int j=0; j<num_rows; j += 2)
        {
            ecl::CubicPolynomial cubic = ecl::CubicPolynomial::DerivativeInterpolation(t0, v0(j), v0(j+1), t1, v1(j), v1(j+1));
            ecl::LinearFunction acc = cubic.derivative().derivative();
            const double a0 = acc.coefficients()[0];
            const double a1 = acc.coefficients()[1];
            
            ecl::QuadraticPolynomial acc_square;
            acc_square.coefficients() << a0*a0, 2*a0*a1, a1*a1;
            
            cost += gaussianQuadratureQuadraticPolynomial(t0, t1, acc_square);
        }
    }
    
    return cost;
}

TrajectoryDerivative SmoothnessCost::derivative(const Trajectory& trajectory)
{
    const Eigen::MatrixXd& milestone_variables = trajectory.getMilestoneVariables();
    const Eigen::VectorXd& milestone_start_variables = trajectory.getMilestoneStartVariables();
    const int num_milestones = milestone_variables.cols();
    const int num_rows = milestone_variables.rows();
    
    Eigen::MatrixXd derivative(milestone_variables.rows(), milestone_variables.cols());
    
    for (int i=0; i<num_milestones; i++)
    {
        const Eigen::VectorXd& v0 = i==0 ? milestone_start_variables : milestone_variables.col(i-1);
        const Eigen::VectorXd& v1 = milestone_variables.col(i);
        const double t0 = trajectory.getMilestoneTimeFromIndex(i-1);
        const double t1 = trajectory.getMilestoneTimeFromIndex(i);
        
        for (int j=0; j<num_rows; j += 2)
        {
            const Eigen::Vector4d v(v0(j), v0(j+1), v1(j), v1(j+1));
            
            const double dinv = 1. / (t1-t0);
            const Eigen::Vector4d diag(1., dinv, 1., dinv);
            const Eigen::DiagonalMatrix<double, 4> D(diag);
            const Eigen::Vector4d term_derivative = (dinv * dinv * dinv) * D * H2_ * D * v;
            
            if (i)
            {
                derivative(j  , i-1) += term_derivative(0);
                derivative(j+1, i-1) += term_derivative(1);
            }
            derivative(j  , i  ) += term_derivative(2);
            derivative(j+1, i  ) += term_derivative(3);
        }
    }
    
    return TrajectoryDerivative(derivative);
}

double SmoothnessCost::gaussianQuadratureQuadraticPolynomial(double t0, double t1, const ecl::QuadraticPolynomial& poly)
{
    const double mid = (t0 + t1) * 0.5;
    const double radius = (t1 - t0) * 0.5;
    
    return (t1-t0) * 0.5 * (
                  gaussian_quadrature2_weights_[0] * poly( mid + radius * gaussian_quadrature2_abscissa_[0] )
                + gaussian_quadrature2_weights_[1] * poly( mid + radius * gaussian_quadrature2_abscissa_[1] )
                + gaussian_quadrature2_weights_[2] * poly( mid + radius * gaussian_quadrature2_abscissa_[2] )
            );
}

}
