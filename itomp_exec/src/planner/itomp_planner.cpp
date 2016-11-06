#include <itomp_exec/planner/itomp_planner.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_state/conversions.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <eigen_conversions/eigen_msg.h>

#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <functional>

#include <std_msgs/Float64MultiArray.h>


namespace itomp_exec
{

ItompPlanner::ItompPlanner(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
    , robot_model_(0)
{
    trajectory_ = new ItompTrajectory();
    optimizer_ = new ItompOptimizer(trajectory_);

    SmoothnessCost* smoothness_cost = new SmoothnessCost();
    CollisionCost* collision_cost = new CollisionCost();

    cost_functions_[0] = smoothness_cost;
    cost_functions_[1] = collision_cost;

    optimizer_->setCostFunction(0, smoothness_cost);
    optimizer_->setCostFunction(1, collision_cost);

    initializePublishers();
}

ItompPlanner::~ItompPlanner()
{
    delete cost_functions_[0];
    delete cost_functions_[1];

    delete optimizer_;
}

void ItompPlanner::initializePublishers()
{
    optimizer_visualize_trajectory_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("itomp_optimizer_trajectory", 1);
    visualize_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("itomp", 1);

    // wait for initializing publisher
    ros::Duration(0.5).sleep();
}

void ItompPlanner::setRobotModel(const PlanningRobotModel *robot_model)
{
    robot_model_ = robot_model;
    trajectory_->setRobotModel(robot_model_);
}

void ItompPlanner::setRobotStartState(const PlanningRobotState& robot_start_state)
{
    trajectory_->setRobotStartState(robot_start_state);
}

void ItompPlanner::setRobotBaseTransform(const Eigen::Affine3d& robot_base_transform)
{
    robot_base_transform_ = robot_base_transform;
}

void ItompPlanner::setPlanningScene(const PlanningScene* planning_scene)
{
    planning_scene_ = planning_scene;
}

void ItompPlanner::setTimestep(double timestep)
{
    timestep_ = timestep;
    trajectory_->setTimestep(timestep);
}

void ItompPlanner::setTrajectoryDuration(double duration)
{
    trajectory_->setTrajectoryDuration(duration);
}

void ItompPlanner::setNumWaypoints(int num_waypoints)
{
    trajectory_->setNumWaypoints(num_waypoints);
}

void ItompPlanner::setCostFunction(int id, Cost* cost)
{
    cost_functions_[id] = cost;
    optimizer_->setCostFunction(id, cost);
}

void ItompPlanner::printCostFunctions()
{
    optimizer_->printCostFunctions();
}

static void optimizeSingleTrajectoryCleanup(void* optimizer)
{
    ItompOptimizer* casted_optimizer = (ItompOptimizer*)optimizer;
    casted_optimizer->optimizeThreadCleanup();
}

static void* optimizeSingleTrajectory(void* optimizer)
{
    Thread::self()->cleanupPush(optimizeSingleTrajectoryCleanup, optimizer);

    ItompOptimizer* casted_optimizer = (ItompOptimizer*)optimizer;
    casted_optimizer->optimize();

    return NULL;
}

void ItompPlanner::planForOneTimestep()
{
    const double optimization_time_limit = 0.8 * timestep_;
    optimizer_thread_ = new Thread(&optimizeSingleTrajectory, optimizer_);

    // cancel optimizer thread after optimization
    // the created threads are canceled
    ros::Duration(optimization_time_limit).sleep();
    optimizer_thread_->cancel();
    optimizer_thread_->join();
    delete optimizer_thread_;
}

void ItompPlanner::enableVisualizeTrajectoryEachStep()
{
    optimizer_->enableVisualizeTrajectoryEachStep(&optimizer_visualize_trajectory_publisher_);
}

void ItompPlanner::disableVisualizeTrajectoryEachStep()
{
    optimizer_->disableVisualizeTrajectoryEachStep();
}

void ItompPlanner::visualizePlanningScene()
{
    planning_scene_->visualize(&visualize_publisher_);
}

void ItompPlanner::visualizeRobotBoundingSpheres(const PlanningRobotState& robot_state, const std::string& ns)
{
    robot_state.visualizeBoundingSpheres(&visualize_publisher_, ns);
}

}
