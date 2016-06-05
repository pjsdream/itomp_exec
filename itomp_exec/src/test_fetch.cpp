#include <itomp_exec/planner/itomp_planner_node.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/RobotState.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <control_msgs/GripperCommandGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <Eigen/Dense>
#include <ros/ros.h>

#include <stdlib.h>
#include <time.h>


class TestFetch
{
private:
    
    struct Pose
    {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
    };
    
public:
    
    TestFetch(const ros::NodeHandle& nh = ros::NodeHandle("~"));

    void moveGripper(double gap, bool wait_for_execution = true);
    void openGripper(bool wait_for_execution = true);
    void closeGripper(bool wait_for_execution = true);
    
    void moveTorso(double position, bool wait_for_execution = true);
    
    void runScenario();
    
private:
    
    void loadTablePoses();
    
    static void initializeTuckState(moveit_msgs::RobotState& start_state);
    static void initializeDefaultState(moveit_msgs::RobotState& start_state);
    
    ros::NodeHandle nh_;
    ros::Publisher start_state_publisher_;
    ros::Publisher display_trajectory_publisher_;
    itomp_exec::ITOMPPlannerNode planner_;
    
    // gripper actionlib client
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client_;
    
    // torso actionlib client
    // rostopic pub -1 /torso_controller/follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal '{header:{}, goal_id:{}, goal:{trajectory : {joint_names: [torso_lift_joint], points: [{positions : [0.35], time_from_start: [5.0, 0.0]}]}}}'
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_client_;
    
    std::vector<Pose> start_poses_;
    std::vector<Pose> target_poses_;
    
    robot_model::RobotModelConstPtr robot_model_;
};

TestFetch::TestFetch(const ros::NodeHandle& nh)
    : nh_(nh)
    , planner_(nh)
    , gripper_client_("gripper_controller/gripper_action")
    , torso_client_("torso_controller/follow_joint_trajectory")
{
    planner_.printParams();
    planner_.printControllers();
    planner_.printCostWeights();
    
    loadTablePoses();
    
    // load robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();
    
    // publisher initialization
    start_state_publisher_ = nh_.advertise<moveit_msgs::DisplayRobotState>("start_state", 1);
    display_trajectory_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("display_planned_path", 1);
    ros::Duration(0.5).sleep();
}

void TestFetch::moveGripper(double gap, bool wait_for_execution)
{
    control_msgs::GripperCommandGoal goal;
    goal.command.position = gap;
    goal.command.max_effort = 1000.;
    gripper_client_.sendGoal(goal);
    
    if (wait_for_execution)
        gripper_client_.waitForResult();
}

void TestFetch::closeGripper(bool wait_for_execution)
{
    moveGripper(0.00, wait_for_execution);
}

void TestFetch::openGripper(bool wait_for_execution)
{
    moveGripper(0.10, wait_for_execution);
}

void TestFetch::moveTorso(double position, bool wait_for_execution)
{
    const double time = 1.0;
    
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(position);
    point.time_from_start = ros::Duration(time);
    
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("torso_lift_joint");
    goal.trajectory.points.push_back(point);
    
    torso_client_.sendGoal(goal);
    
    if (wait_for_execution)
        torso_client_.waitForResult();
}

void TestFetch::initializeTuckState(moveit_msgs::RobotState& start_state)
{
    start_state.joint_state.name.push_back("shoulder_pan_joint");
    start_state.joint_state.name.push_back("shoulder_lift_joint");
    start_state.joint_state.name.push_back("upperarm_roll_joint");
    start_state.joint_state.name.push_back("elbow_flex_joint");
    start_state.joint_state.name.push_back("forearm_roll_joint");
    start_state.joint_state.name.push_back("wrist_flex_joint");
    start_state.joint_state.name.push_back("wrist_roll_joint");
    
    start_state.joint_state.position.push_back(1.32);
    start_state.joint_state.position.push_back(1.40);
    start_state.joint_state.position.push_back(-0.2);
    start_state.joint_state.position.push_back(1.72);
    start_state.joint_state.position.push_back(0.0);
    start_state.joint_state.position.push_back(1.66);
    start_state.joint_state.position.push_back(0.0);
    
    start_state.is_diff = true;
}

void TestFetch::initializeDefaultState(moveit_msgs::RobotState& start_state)
{
    start_state.joint_state.name.push_back("shoulder_pan_joint");
    start_state.joint_state.name.push_back("shoulder_lift_joint");
    start_state.joint_state.name.push_back("upperarm_roll_joint");
    start_state.joint_state.name.push_back("elbow_flex_joint");
    start_state.joint_state.name.push_back("forearm_roll_joint");
    start_state.joint_state.name.push_back("wrist_flex_joint");
    start_state.joint_state.name.push_back("wrist_roll_joint");
    
    start_state.joint_state.position.push_back(0.0);
    start_state.joint_state.position.push_back(0.0);
    start_state.joint_state.position.push_back(0.0);
    start_state.joint_state.position.push_back(0.0);
    start_state.joint_state.position.push_back(0.0);
    start_state.joint_state.position.push_back(0.0);
    start_state.joint_state.position.push_back(0.0);
    
    start_state.is_diff = true;
}

void TestFetch::loadTablePoses()
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
            
            if (type_string == "start")
            {
                start_poses_.push_back(pose);
            }
            else if (type_string == "target")
            {
                target_poses_.push_back(pose);
            }
        }
    }
}

void TestFetch::runScenario()
{
    planning_interface::MotionPlanRequest req;
    req.group_name = "arm";
    
    int goal_type = 0;
    int goal_index = 0;
    std::vector<char> goal_achieved(start_poses_.size(), false);
    
    while (true)
    {
        if (goal_type == 1)
            closeGripper();
        
        req.goal_constraints.clear();
        
        // initialize start state with current robot state
        sensor_msgs::JointStateConstPtr start_joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
        robot_state::RobotState start_state( robot_model_ );
        start_state.setVariableValues(*start_joint_state);
        start_state.update();
        moveit::core::robotStateToRobotStateMsg(start_state, req.start_state, false);
        
        // initialize start state with current state
        //initializeDefaultState(req.start_state);

        // visualize start state
        moveit_msgs::DisplayRobotState start_state_display_msg;
        start_state_display_msg.state = req.start_state;
        start_state_publisher_.publish(start_state_display_msg);
        
        // goal pose setting
        /*
        moveit_msgs::PositionConstraint goal_position_constraint;
        goal_position_constraint.link_name = "wrist_roll_link";
        Eigen::Vector3d goal_ee_position(0.5, -0.5, 1.0);
        tf::vectorEigenToMsg(goal_ee_position, goal_position_constraint.target_point_offset);
        
        moveit_msgs::OrientationConstraint goal_orientation_constraint;
        goal_orientation_constraint.link_name = "wrist_roll_link";
        Eigen::Quaterniond goal_ee_orientation(0.707106781, 0.0, 0.707106781, 0.0);
        tf::quaternionEigenToMsg(goal_ee_orientation, goal_orientation_constraint.orientation);
        
        moveit_msgs::Constraints goal_constraints;
        goal_constraints.position_constraints.push_back(goal_position_constraint);
        goal_constraints.orientation_constraints.push_back(goal_orientation_constraint);
        req.goal_constraints.push_back(goal_constraints);
        
        planner_.setMotionPlanRequest(req);
        */
        moveit_msgs::PositionConstraint goal_position_constraint;
        goal_position_constraint.link_name = "wrist_roll_link";
        if (goal_type == 0)
            tf::vectorEigenToMsg(start_poses_[goal_index].position, goal_position_constraint.target_point_offset);
        else
            tf::vectorEigenToMsg(target_poses_[goal_index].position, goal_position_constraint.target_point_offset);
        
        moveit_msgs::OrientationConstraint goal_orientation_constraint;
        goal_orientation_constraint.link_name = "wrist_roll_link";
        if (goal_type == 0)
            tf::quaternionEigenToMsg(start_poses_[goal_index].orientation, goal_orientation_constraint.orientation);
        else
            tf::quaternionEigenToMsg(target_poses_[goal_index].orientation, goal_orientation_constraint.orientation);
        
        moveit_msgs::Constraints goal_constraints;
        goal_constraints.position_constraints.push_back(goal_position_constraint);
        goal_constraints.orientation_constraints.push_back(goal_orientation_constraint);
        req.goal_constraints.push_back(goal_constraints);
        
        planner_.setMotionPlanRequest(req);
        
        // plan and execute
        planning_interface::MotionPlanResponse res;
        planner_.planAndExecute(res);
        //planner_.plan(res);

        // visualize robot trajectory
        moveit_msgs::MotionPlanResponse response_msg;
        res.getMessage(response_msg);

        moveit_msgs::DisplayTrajectory display_trajectory_msg;
        display_trajectory_msg.trajectory_start = response_msg.trajectory_start;
        display_trajectory_msg.trajectory.push_back( response_msg.trajectory );
        display_trajectory_msg.model_id = "model";
        display_trajectory_publisher_.publish(display_trajectory_msg);
        
        if (goal_type == 1)
            openGripper();
        
        // setup the next goal
        if (goal_type == 0)
        {
            goal_type = 1;
        }
        else
        {
            goal_achieved[goal_index] = true;
            
            std::vector<int> indices;
            for (int i=0; i<start_poses_.size(); i++)
            {
                if (!goal_achieved[i])
                    indices.push_back(i);
            }
            
            if (indices.empty())
                break;
            
            goal_type = 0;
            goal_index = indices[ rand() % indices.size() ];
        }
        
        // once
        // break;
    }
}

int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    
    srand(time(NULL));
    
    ros::init(argc, argv, "move_itomp");
    
    TestFetch test_fetch;
    /*    
    test_fetch.closeGripper();
    test_fetch.openGripper();
    */
    
    test_fetch.moveTorso(0.1);
    test_fetch.moveTorso(0.35);
    test_fetch.moveTorso(0.1);
    test_fetch.moveTorso(0.35);
    
    test_fetch.runScenario();
    
    return 0;
}
