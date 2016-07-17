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


/*
 * target locations from apc_demo/src/apc_demo_m2_move_group.cpp

   setPose(x, y, z, qx, qy, qz, qw)

    geometry_msgs::Pose observe_pose[13];
    setPose(observe_pose[1], 0.551, 0.585, 0.940,0.689, 0.176, -0.131, 0.691);
    setPose(observe_pose[3], 0.550, 0.062, 0.963,0.678, 0.154, -0.187, 0.694);
    setPose(observe_pose[5], 0.506, 0.333, 0.698,0.687, 0.145, -0.170, 0.692);
    setPose(observe_pose[7], 0.539, 0.604, 0.478,0.685, 0.174, -0.175, 0.685);
    setPose(observe_pose[10], 0.524, 0.591, 0.427,-0.700, -0.084, 0.050, 0.707);
    //setPose(observe_pose[12], 0.553, 0.028, 0.426,-0.737, -0.086, 0.006, 0.670);
    setPose(observe_pose[12],0.553, 0.028, 0.428,-0.738, -0.087, 0.003, 0.669);
    //setPose(observe_pose[5], 0.470, 0.320, 0.942,0.652, 0.072, -0.236, 0.717);

    geometry_msgs::Pose pre_dropping_pose[13];
    setPose(pre_dropping_pose[1],-0.35,0.15,0.25,-0.500, 0.500, 0.500, 0.500);
    setPose(pre_dropping_pose[3],-0.35,0.15,0.25,-0.500, 0.500, 0.500, 0.500);
    setPose(pre_dropping_pose[5],-0.35,0.15,0.25,-0.500, 0.500, 0.500, 0.500);
    setPose(pre_dropping_pose[7],-0.35,0.35,0.25,-0.500, 0.500, 0.500, 0.500);
    setPose(pre_dropping_pose[10],-0.35,0.35,0.25,-0.500, 0.500, 0.500, 0.500);
    setPose(pre_dropping_pose[12],-0.35,0.35,0.25,-0.500, 0.500, 0.500, 0.500);
 */

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

    void loadTargetPoses();

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

    // target poess
    std::vector<Pose> bin_poses_;
    std::vector<Pose> drop_poses_;

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

    loadTargetPoses();
}

void ITOMPUr5::loadTargetPoses()
{
    XmlRpc::XmlRpcValue poses;

    if (nh_.getParam("poses", poses))
    {
        for (int i=0; i<poses.size(); i++)
        {
            XmlRpc::XmlRpcValue xml_pose = poses[i];

            std::string type_string;
            Eigen::Vector3d position;
            Eigen::Quaterniond orientation;

            for (XmlRpc::XmlRpcValue::iterator it = xml_pose.begin(); it != xml_pose.end(); it++)
            {
                const std::string key = static_cast<std::string>(it->first);

                if (key == "type")
                    type_string = static_cast<std::string>(it->second);

                else if (key == "position")
                {
                    XmlRpc::XmlRpcValue value = it->second;
                    for (int i=0; i<3; i++)
                        position(i) = static_cast<double>(value[i]);
                }

                else if (key == "orientation")
                {
                    XmlRpc::XmlRpcValue value = it->second;
                    orientation.w() = static_cast<double>(value[0]);
                    orientation.x() = static_cast<double>(value[1]);
                    orientation.y() = static_cast<double>(value[2]);
                    orientation.z() = static_cast<double>(value[3]);
                }
            }

            Pose pose;
            pose.position = position;
            pose.orientation = orientation;

            if (type_string == "bin")
            {
                bin_poses_.push_back(pose);
            }
            else if (type_string == "drop")
            {
                pose.orientation = Eigen::Quaterniond(0, 0, 0, 1) * pose.orientation;
                drop_poses_.push_back(pose);
            }
        }
    }
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
    int goal_type = 1;
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
        goal_position_constraint.weight = 1.;
        if (goal_type == 0)
            tf::vectorEigenToMsg(drop_poses_[goal_index].position, goal_position_constraint.target_point_offset);
        else
            tf::vectorEigenToMsg(bin_poses_[goal_index].position, goal_position_constraint.target_point_offset);
        
        moveit_msgs::OrientationConstraint goal_orientation_constraint;
        goal_orientation_constraint.link_name = endeffector_name_;
        goal_orientation_constraint.weight = 1.0;
        if (goal_type == 0)
            tf::quaternionEigenToMsg(drop_poses_[goal_index].orientation, goal_orientation_constraint.orientation);
        else
            tf::quaternionEigenToMsg(bin_poses_[goal_index].orientation, goal_orientation_constraint.orientation);
        
        moveit_msgs::Constraints goal_constraints;
        goal_constraints.position_constraints.push_back(goal_position_constraint);
        goal_constraints.orientation_constraints.push_back(goal_orientation_constraint);
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
        //display_trajectory_publisher_.publish(display_trajectory_msg);

        if (goal_type == 0)
        {
            goal_type = 1;
        }
        else
        {
            goal_type = 0;

            // goal index random assignment
            /*
            goal_achieved[goal_index] = true;

            std::vector<int> indices;
            for (int i=0; i<start_poses_.size(); i++)
            {
                if (!goal_achieved[i])
                    indices.push_back(i);
            }

            if (indices.empty())
                break;

            goal_index = indices[ rand() % indices.size() ];
            */

            // goal index assignment in order
            if (goal_index == bin_poses_.size() - 1)
                goal_index = 0;
            else
                goal_index++;
        }

        // once
        //break;
    }
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
        goal_position_constraint.weight = 1.;
        if (goal_index == 0)
            tf::vectorEigenToMsg(start_pose.position, goal_position_constraint.target_point_offset);
        else
            tf::vectorEigenToMsg(target_pose.position, goal_position_constraint.target_point_offset);
        
        moveit_msgs::OrientationConstraint goal_orientation_constraint;
        goal_orientation_constraint.link_name = endeffector_name_;
        goal_orientation_constraint.weight = 0.1;
        if (goal_index == 0)
            tf::quaternionEigenToMsg(start_pose.orientation, goal_orientation_constraint.orientation);
        else
            tf::quaternionEigenToMsg(target_pose.orientation, goal_orientation_constraint.orientation);
        
        moveit_msgs::Constraints goal_constraints;
        goal_constraints.position_constraints.push_back(goal_position_constraint);
        goal_constraints.orientation_constraints.push_back(goal_orientation_constraint);
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

    itomp_ur5.runScenario();
    //itomp_ur5.runMovingArmScenario();

    return 0;
}
