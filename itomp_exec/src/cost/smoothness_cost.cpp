#include <itomp_exec/cost/smoothness_cost.h>
#include <itomp_exec/planner/itomp_planner_node.h>
#include <ecl/geometry/polynomial.hpp>
#include <itomp_exec/util/gaussian_quadrature.h>


namespace itomp_exec
{

SmoothnessCost::SmoothnessCost(ITOMPOptimizer& optimizer, double weight)
    : Cost(optimizer, weight)
{
    H2_.setZero();
    for (int i=0; i<3; i++)
    {
        const double w = gaussianQuadratureWeight2(i);
        const double t = 0.5 + 0.5 * gaussianQuadratureAbscissa2(i);
        const Eigen::Vector4d h(12.*t - 6., 6.*t - 4., -12.*t + 6., 6.*t - 2.);
        H2_ += w * h * h.transpose();
    }
}

void SmoothnessCost::addCost()
{
    ITOMPOptimizer& optimizer = getOptimizer();
    const double weight = getWeight();

    double& cost = optimizer.cost();

    const int num_joints = optimizer.getNumJoints();
    const int num_milestones = optimizer.getNumMilestones();
    const double trajectory_duration = optimizer.getTrajectoryDuration();

    for (int i=0; i<num_joints; i++)
    {
        for (int j=0; j<num_milestones; j++)
        {
            const double t0 = (double)j / num_milestones * trajectory_duration;
            const double t1 = (double)(j+1) / num_milestones * trajectory_duration;
            const ecl::CubicPolynomial& poly = optimizer.getCubicPolynomial(i, j);

            const ecl::LinearFunction acc = poly.derivative().derivative();
            const double a0 = acc.coefficients()[0];
            const double a1 = acc.coefficients()[1];

            ecl::QuadraticPolynomial acc_square;
            acc_square.coefficients() << a0*a0, 2*a0*a1, a1*a1;

            // normalize with trajectory duration
            cost += gaussianQuadratureQuadraticPolynomial(t0, t1, acc_square) * trajectory_duration * weight;
        }
    }
}

void SmoothnessCost::addDerivative()
{
    ITOMPOptimizer& optimizer = getOptimizer();
    const double weight = getWeight();

    Eigen::MatrixXd& milestone_derivative = optimizer.milestoneDerivative();

    const int num_joints = optimizer.getNumJoints();
    const int num_milestones = optimizer.getNumMilestones();
    const double trajectory_duration = optimizer.getTrajectoryDuration();
    const Eigen::VectorXd& start_milestone = optimizer.getStartMilestone();
    const Eigen::MatrixXd& milestones = optimizer.getMilestones();

    for (int i=0; i<num_milestones; i++)
    {
        const Eigen::VectorXd& variables0 = i==0 ? start_milestone : milestones.col(i-1);
        const Eigen::VectorXd& variables1 = milestones.col(i);
        const double t0 = (double)i / num_milestones * trajectory_duration;
        const double t1 = (double)(i+1) / num_milestones * trajectory_duration;

        for (int j=0; j<num_joints; j++)
        {
            const double& p0 = variables0(j);
            const double& v0 = variables0(num_joints + j);
            const double& p1 = variables1(j);
            const double& v1 = variables1(num_joints + j);

            const Eigen::Vector4d v(p0, v0, p1, v1);

            const double dinv = 1. / (t1-t0);
            const Eigen::Vector4d diag(1., t1-t0, 1., t1-t0);
            const Eigen::DiagonalMatrix<double, 4> D(diag);
            Eigen::Vector4d term_derivative = (dinv * dinv * dinv) * D * H2_ * D * v;

            // normalize with trajectory duration
            term_derivative *= weight * trajectory_duration;

            if (i)
            {
                milestone_derivative(j, i-1) += term_derivative(0);
                milestone_derivative(num_joints + j, i-1) += term_derivative(1);
            }
            milestone_derivative(j, i) += term_derivative(2);
            milestone_derivative(num_joints + j, i) += term_derivative(3);
        }
    }
}

}
