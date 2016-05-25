#include <itomp_exec/planner/itomp_planner_node.h>


namespace itomp_exec
{

ITOMPPlannerNode::ITOMPPlannerNode(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
    loadParams();
}

void ITOMPPlannerNode::loadParams()
{
    node_handle_.param("trajectory_duration", options_.trajectory_duration, 5.0);
    node_handle_.param("num_milestones", options_.num_milestones, 10);
    node_handle_.param("num_interpolation_samples", options_.num_interpolation_samples, 10);
}

void ITOMPPlannerNode::printParams()
{
    ROS_INFO("ITOMP-exec parameters:");
    
    ROS_INFO(" trajectory_duration: %lf", options_.trajectory_duration);
    ROS_INFO(" num_milestones: %d", options_.num_milestones);
    ROS_INFO(" num_interpolation_samples: %d", options_.num_interpolation_samples);
}

void ITOMPPlannerNode::setRobotModel(const robot_model::RobotModelConstPtr& model)
{
    robot_model_ = model;
    trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(model));
}

void ITOMPPlannerNode::setPlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene)
{
    planning_scene_ = planning_scene;
}

void ITOMPPlannerNode::setMotionPlanRequest(const planning_interface::MotionPlanRequest& req)
{
    req_ = req;
}

bool ITOMPPlannerNode::plan(planning_interface::MotionPlanResponse& res)
{
    res_ = &res;
    execution_while_planning_ = false;
    return true;
}

bool ITOMPPlannerNode::planAndExecute(planning_interface::MotionPlanResponse& res)
{
    res_ = &res;
    execution_while_planning_ = true;
    return true;
}

void ITOMPPlannerNode::recursePlan()
{
}

}
