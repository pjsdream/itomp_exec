#include <itomp_exec/cost/cost.h>
#include <itomp_exec/optimization/itomp_optimizer.h>


namespace itomp_exec
{

Cost::Cost(ITOMPOptimizer& optimizer, double weight)
    : optimizer_(optimizer)
    , weight_(weight)
{
}

void Cost::addCost()
{
}

void Cost::addDerivative()
{
    //Numerical::addNumericalDerivative(&addCost);
}

void Cost::addDerivativeByInterpolationIndex(int joint_index, int interpolation_index, double derivative)
{
    Eigen::MatrixXd& milestone_derivative = optimizer_.milestoneDerivative();

    const std::pair<int, int> interpolation_index_position = optimizer_.getInterpolationIndexPosition(interpolation_index);
    const int milestone_index0 = interpolation_index_position.first - 1;
    const int milestone_index1 = milestone_index0 + 1;
    const int interpolation_sample_index = interpolation_index_position.second;
    const Eigen::Vector4d variable_derivative = optimizer_.getInterpolatedCurveBasis(interpolation_sample_index) * derivative;

    if (milestone_index0 != -1)
    {
        milestone_derivative(joint_index, milestone_index0) += variable_derivative(0);
        milestone_derivative(joint_index, milestone_index0) += variable_derivative(1);
    }
    milestone_derivative(joint_index, milestone_index1) += variable_derivative(2);
    milestone_derivative(joint_index, milestone_index1) += variable_derivative(3);
}

}
