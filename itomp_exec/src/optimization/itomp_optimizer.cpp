#include <itomp_exec/optimization/itomp_optimizer.h>
#include <visualization_msgs/MarkerArray.h>
#include <itomp_exec/util/gaussian_quadrature.h>
#include <itomp_exec/optimization/optimization_stop_strategy.h>
#include <itomp_exec/optimization/optimization_search_strategy.h>
#include <itomp_exec/optimization/find_min_box_constrained_thread_safe.h>
#include <eigen_conversions/eigen_msg.h>

#include <random>

#include <ros/ros.h>

#include <omp.h>


namespace itomp_exec
{

const Eigen::VectorXd ItompOptimizer::convertDlibToEigenVector(const column_vector& v)
{
    return Eigen::Map<Eigen::VectorXd>((double*)v.begin(), v.size());
}

const ItompOptimizer::column_vector ItompOptimizer::convertEigenToDlibVector(const Eigen::MatrixXd& v)
{
    const int n = v.rows() * v.cols();
    column_vector r(n);
    memcpy(r.begin(), v.data(), sizeof(double) * n);
    return r;
}

ItompOptimizer::ItompOptimizer(ItompTrajectory* trajectory)
    : trajectory_(trajectory)
    , trajectory_publisher_(0)
{
}

ItompOptimizer::~ItompOptimizer()
{
}

void ItompOptimizer::enableVisualizeTrajectoryEachStep(ros::Publisher* publisher)
{
    trajectory_publisher_ = publisher;
}

void ItompOptimizer::disableVisualizeTrajectoryEachStep()
{
    trajectory_publisher_ = 0;
}

void ItompOptimizer::setCostFunction(int id, Cost *cost)
{
    cost_functions_[id] = cost;
}

void ItompOptimizer::printCostFunctions()
{
    ROS_INFO("Cost functions:");

    for (std::map<int, Cost*>::iterator it = cost_functions_.begin(); it != cost_functions_.end(); it++)
    {
        ROS_INFO("Cost function id %d", it->first);
        it->second->printInfo();
    }

    ROS_INFO("");
}

double ItompOptimizer::optimizationCost(const column_vector& variables)
{
    // TODO
    return 0;
}

ItompOptimizer::column_vector ItompOptimizer::optimizationCostDerivative(const column_vector& variables)
{
    // TODO
    column_vector derivative(variables.size());

    for (int i=0; i<derivative.size(); i++)
    {
    }

    return derivative;
}

void ItompOptimizer::initializeOptimization()
{
    const PlanningRobotModel* robot_model = trajectory_->getRobotModel();
    const int num_joints = robot_model->numJoints();
    const int num_waypoints = trajectory_->numWaypoints();
    const std::vector<PlanningRobotModel::JointLimit>& joint_limits = robot_model->getJointLimits();

    optimization_variables_lower_.resize(num_joints * 2 * num_waypoints + 1);
    optimization_variables_upper_.resize(num_joints * 2 * num_waypoints + 1);

    // goal reach time
    goal_time_ = 0.;
    optimization_variables_lower_(0) = 0;
    optimization_variables_upper_(0) = trajectory_->getTrajectoryDuration();

    for (int i=0; i<num_joints; i++)
    {
        for (int j=0; j<num_waypoints; j++)
        {
            // position
            optimization_variables_lower_(1 + j * 2 * num_joints + i) = joint_limits[i].min_position;
            optimization_variables_upper_(1 + j * 2 * num_joints + i) = joint_limits[i].max_position;

            // velocity
            optimization_variables_lower_(1 + j * 2 * num_joints + i + num_joints) = joint_limits[i].min_velocity;
            optimization_variables_upper_(1 + j * 2 * num_joints + i + num_joints) = joint_limits[i].max_velocity;
        }
    }
}

void ItompOptimizer::encodeOptimizationVariables()
{
    optimization_variables_.set_size( optimization_variables_lower_.rows() );
    optimization_variables_(0) = goal_time_;

    // column major order
    int k = 1;
    const Eigen::MatrixXd& waypoints = trajectory_->getWaypointVariables();
    for (int j=0; j<waypoints.cols(); j++)
        for (int i=0; i<waypoints.rows(); i++)
            optimization_variables_(k++) = waypoints(i, j);
}

void ItompOptimizer::decodeOptimizationVariables()
{
    goal_time_ = optimization_variables_(0);

    // column major order
    int k = 1;
    Eigen::MatrixXd& waypoints = trajectory_->getWaypointVariables();
    for (int j=0; j<waypoints.cols(); j++)
        for (int i=0; i<waypoints.rows(); i++)
            waypoints(i, j) = optimization_variables_(k++);
}

void ItompOptimizer::optimize()
{
    optimization_start_time_ = ros::Time::now();

    initializeOptimization();

    encodeOptimizationVariables();
    column_vector lower = convertEigenToDlibVector(optimization_variables_lower_);
    column_vector upper = convertEigenToDlibVector(optimization_variables_upper_);

    dlib::objective_delta_stop_strategy stop_strategy(1e-5, 1000000);

    find_min_box_constrained_thread_safe(
                itomp_exec::bfgs_search_strategy(),
                stop_strategy,
                std::bind(&ItompOptimizer::optimizationCost, this, std::placeholders::_1),
                std::bind(&ItompOptimizer::optimizationCostDerivative, this, std::placeholders::_1),
                optimization_variables_,
                lower,
                upper
                );

    while (true)
    {
        Thread::self()->testCancel();
    }
}

void ItompOptimizer::optimizeThreadCleanup()
{
    ROS_INFO("Optimization elapsed time: %lf sec", (ros::Time::now() - optimization_start_time_).toSec());
    decodeOptimizationVariables();
}

}
