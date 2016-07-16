#include <itomp_exec/planner/itomp_planner_node.h>
#include <itomp_exec/robot/bounding_sphere_robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/RobotState.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <itomp_exec/util/joint_state_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <ros/ros.h>

#include <stdlib.h>
#include <time.h>


class ITOMPUr5
{
private:
    
    static const std::string planning_group_;
    static const std::string endeffector_name_;

    struct Pose
    {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
    };

public:
    
    ITOMPUr5(const ros::NodeHandle& nh = ros::NodeHandle("~"));

    void moveTorso(double position, bool wait_for_execution = true);

    void runScenario();
    void runMovingArmScenario();
    
private:

    void initializeCurrentState(robot_state::RobotState& state);
    void initializeCurrentState(moveit_msgs::RobotState& start_state);
    
    ros::NodeHandle nh_;
    itomp_exec::ITOMPPlannerNode planner_;

    robot_model::RobotModelConstPtr moveit_robot_model_;
    itomp_exec::BoundingSphereRobotModelPtr robot_model_;

    ros::Publisher display_trajectory_publisher_;

    // joint state listener
    itomp_exec::JointStateListener joint_state_listener_;
};



const std::string ITOMPUr5::planning_group_ = "manipulator";
const std::string ITOMPUr5::endeffector_name_ = "ee_link";



ITOMPUr5::ITOMPUr5(const ros::NodeHandle& nh)
    : nh_(nh)
    , planner_(nh)
{
    // initialize planner
    planner_.printParams();
    planner_.printControllers();
    planner_.printCostWeights();
    
    // load robot model
    moveit_robot_model_ = planner_.getMoveitRobotModel();
    robot_model_ = planner_.getRobotModel();

    // publisher initialization
    display_trajectory_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("display_planned_path", 1);
    ros::Duration(0.5).sleep();
}

void ITOMPUr5::initializeCurrentState(robot_state::RobotState& state)
{
    // /joint_states topic is publishing one of controller group
    // one is a group of two fingers, and the other one has others
    sensor_msgs::JointState start_joint_state;
    std::set<std::string> joint_set;
    
    const std::vector<const robot_model::JointModel*>& active_joint_models = moveit_robot_model_->getActiveJointModels();
    for (int i=0; i<active_joint_models.size(); i++)
        joint_set.insert( active_joint_models[i]->getName() );
    
    while (!joint_set.empty())
    {
        sensor_msgs::JointState current_joint_state = joint_state_listener_.getJointStates();
        
        for (int i=0; i<current_joint_state.name.size(); i++)
        {
            const std::string joint_name = current_joint_state.name[i];
            if (joint_set.find(joint_name) != joint_set.end())
            {
                joint_set.erase(joint_name);
                
                start_joint_state.header = current_joint_state.header;
                start_joint_state.name.push_back(joint_name);
                
                if (!current_joint_state.position.empty())
                    start_joint_state.position.push_back( current_joint_state.position[i] );
                
                if (!current_joint_state.velocity.empty())
                    start_joint_state.velocity.push_back( current_joint_state.velocity[i] );
                
                if (!current_joint_state.effort.empty())
                    start_joint_state.effort.push_back( current_joint_state.effort[i] );
            }
        }
    }
    
    state.setVariableValues(start_joint_state);
    state.update();
}

void ITOMPUr5::initializeCurrentState(moveit_msgs::RobotState& start_state)
{
    robot_state::RobotState state( moveit_robot_model_ );
    initializeCurrentState(state);
    moveit::core::robotStateToRobotStateMsg(state, start_state, false);
}

void ITOMPUr5::runScenario()
{
#if 0
    int goal_type = 0;
    int goal_index = 0;
    
    while (true)
    {
        ROS_INFO("Goal index: [%d, %d]", goal_type, goal_index);
        planning_interface::MotionPlanRequest req;
        req.group_name = planning_group_;
        
        // initialize start state with current robot state
        initializeCurrentState(req.start_state);
        
        // initialize start state with current state
        //initializeDefaultState(req.start_state);

        // visualize start state
        moveit_msgs::DisplayRobotState start_state_display_msg;
        start_state_display_msg.state = req.start_state;
        
        // goal pose setting
        /*
        moveit_msgs::PositionConstraint goal_position_constraint;
        goal_position_constraint.link_name = endeffector_name_;
        Eigen::Vector3d goal_ee_position(0.5, -0.5, 1.0);
        tf::vectorEigenToMsg(goal_ee_position, goal_position_constraint.target_point_offset);
        
        moveit_msgs::OrientationConstraint goal_orientation_constraint;
        goal_orientation_constraint.link_name = endeffector_name_;
        Eigen::Quaterniond goal_ee_orientation(0.707106781, 0.0, 0.707106781, 0.0);
        tf::quaternionEigenToMsg(goal_ee_orientation, goal_orientation_constraint.orientation);
        
        moveit_msgs::Constraints goal_constraints;
        goal_constraints.position_constraints.push_back(goal_position_constraint);
        goal_constraints.orientation_constraints.push_back(goal_orientation_constraint);
        req.goal_constraints.push_back(goal_constraints);
        
        planner_.setMotionPlanRequest(req);
        */
        moveit_msgs::PositionConstraint goal_position_constraint;
        goal_position_constraint.link_name = endeffector_name_;
        if (goal_type == 0)
            tf::vectorEigenToMsg(start_poses_[goal_index].position, goal_position_constraint.target_point_offset);
        else
            tf::vectorEigenToMsg(target_poses_[goal_index].position, goal_position_constraint.target_point_offset);
        
        moveit_msgs::OrientationConstraint goal_orientation_constraint;
        goal_orientation_constraint.link_name = endeffector_name_;
        if (goal_type == 0)
            tf::quaternionEigenToMsg(start_poses_[goal_index].orientation, goal_orientation_constraint.orientation);
        else
            tf::quaternionEigenToMsg(target_poses_[goal_index].orientation, goal_orientation_constraint.orientation);
        
        moveit_msgs::Constraints goal_constraints;
        goal_constraints.position_constraints.push_back(goal_position_constraint);
        goal_constraints.orientation_constraints.push_back(goal_orientation_constraint);
        req.goal_constraints.push_back(goal_constraints);
        
        planner_.setMotionPlanRequest(req);
        
        // trajectory duration
        double trajectory_duration;
        if (goal_type == 0 && goal_index == 0)
            trajectory_duration = 5.0;
        else
            trajectory_duration = 2.0 + goal_index;

        // plan and execute
        planning_interface::MotionPlanResponse res;
        planner_.setTrajectoryDuration(trajectory_duration);
        planner_.planAndExecute(res);

        // visualize robot trajectory
        moveit_msgs::MotionPlanResponse response_msg;
        res.getMessage(response_msg);

        moveit_msgs::DisplayTrajectory display_trajectory_msg;
        display_trajectory_msg.trajectory_start = response_msg.trajectory_start;
        display_trajectory_msg.trajectory.push_back( response_msg.trajectory );
        display_trajectory_msg.model_id = "model";
        //display_trajectory_publisher_.publish(display_trajectory_msg);

        // once
        break;
    }
#endif
}

void ITOMPUr5::runMovingArmScenario()
{
    int goal_index = 0;
    Pose start_pose;
    start_pose.position = Eigen::Vector3d(0.5, -0.5, 0.2);
    start_pose.orientation = Eigen::Quaterniond(1., 0., 0., 0.);
    Pose target_pose;
    target_pose.position = Eigen::Vector3d(0.5, 0.5, 0.2);
    target_pose.orientation = Eigen::Quaterniond(1., 0., 0., 0.);
    
    while (true)
    {
        ROS_INFO("Goal index: [%d]", goal_index);
        planning_interface::MotionPlanRequest req;
        req.group_name = planning_group_;
        
        // initialize start state with current robot state
        initializeCurrentState(req.start_state);
        
        // initialize start state with current state
        //initializeDefaultState(req.start_state);

        // visualize start state
        moveit_msgs::DisplayRobotState start_state_display_msg;
        start_state_display_msg.state = req.start_state;
        //start_state_publisher_.publish(start_state_display_msg);
        
        // goal pose setting
        moveit_msgs::PositionConstraint goal_position_constraint;
        goal_position_constraint.link_name = endeffector_name_;
        if (goal_index == 0)
            tf::vectorEigenToMsg(start_pose.position, goal_position_constraint.target_point_offset);
        else
            tf::vectorEigenToMsg(target_pose.position, goal_position_constraint.target_point_offset);
        
        moveit_msgs::OrientationConstraint goal_orientation_constraint;
        goal_orientation_constraint.link_name = endeffector_name_;
        if (goal_index == 0)
            tf::quaternionEigenToMsg(start_pose.orientation, goal_orientation_constraint.orientation);
        else
            tf::quaternionEigenToMsg(target_pose.orientation, goal_orientation_constraint.orientation);
        
        moveit_msgs::Constraints goal_constraints;
        goal_constraints.position_constraints.push_back(goal_position_constraint);
        //goal_constraints.orientation_constraints.push_back(goal_orientation_constraint);
        req.goal_constraints.push_back(goal_constraints);
        
        planner_.setMotionPlanRequest(req);

        // plan and execute
        planning_interface::MotionPlanResponse res;
        planner_.planAndExecute(res);

        // visualize robot trajectory
        moveit_msgs::MotionPlanResponse response_msg;
        res.getMessage(response_msg);

        moveit_msgs::DisplayTrajectory display_trajectory_msg;
        display_trajectory_msg.trajectory_start = response_msg.trajectory_start;
        display_trajectory_msg.trajectory.push_back( response_msg.trajectory );
        display_trajectory_msg.model_id = "model";
        display_trajectory_publisher_.publish(display_trajectory_msg);
        
        // move endeffector vertically using IK to pick or place
        if (goal_index == 0)
        {
            goal_index = 1;
        }
        else
        {
            goal_index = 0;
        }
    }
}

int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);
    
    srand(time(NULL));
    
    ros::init(argc, argv, "itomp_ur5");
    
    ITOMPUr5 itomp_ur5;

    itomp_ur5.runMovingArmScenario();

    return 0;
}
