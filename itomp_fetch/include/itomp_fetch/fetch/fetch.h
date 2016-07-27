#ifndef ITOMP_FETCH_FETCH_H
#define ITOMP_FETCH_FETCH_H


#include <ros/ros.h>
#include <Eigen/Dense>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <itomp_exec/planner/itomp_planner_node.h>
#include <itomp_exec/robot/bounding_sphere_robot_model.h>
#include <itomp_exec/util/joint_state_listener.h>


namespace itomp_fetch
{

class Fetch
{
private:

    static const std::string planning_group_;
    static const std::string endeffector_name_;
    static const double endeffector_vertical_moving_distance_;
    static const double gripper_picking_distance_;

    struct Pose
    {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
    };

    struct AttachingSphere
    {
        double radius;
        Eigen::Vector3d position;
        std::string attaching_link;
        Eigen::Vector3d attaching_position;
    };

    struct Sphere
    {
        Sphere(double radius, const Eigen::Vector3d& position)
            : radius(radius), position(position)
        {
        }

        double radius;
        Eigen::Vector3d position;
    };

public:

    Fetch(const ros::NodeHandle& nh = ros::NodeHandle("~"));

    void moveGripper(double gap, bool wait_for_execution = true);
    void openGripper(bool wait_for_execution = true);
    void closeGripper(bool wait_for_execution = true);

    void moveTorso(double position, bool wait_for_execution = true);

    void attachSphere(int sphere_index);
    void detachSphere(int sphere_index);

    void runScenario();
    void runMovingArmScenario();
    void runMovingArmScenarioOMPL();

private:

    void loadTablePoses();

    void moveEndeffectorVertically(double distance); // using ITOMP, with short duration
    void moveEndeffectorVerticallyIK(double distance, bool wait_for_execution = true); // setFromIK result is very different from initial state
    void moveEndeffectorVerticallyTarget(double distance, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, bool wait_for_execution = true); // using ITOMP, to target distance

    void initializeCurrentState(robot_state::RobotState& state);
    void initializeCurrentState(moveit_msgs::RobotState& start_state);
    static void initializeTuckState(moveit_msgs::RobotState& start_state);
    static void initializeDefaultState(moveit_msgs::RobotState& start_state);

    ros::NodeHandle nh_;
    ros::Publisher start_state_publisher_;
    ros::Publisher goal_state_publisher_;
    ros::Publisher display_trajectory_publisher_;
    ros::Publisher target_positions_publisher_;
    itomp_exec::ITOMPPlannerNode planner_;

    // gripper actionlib client
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client_;

    // torso actionlib client
    // rostopic pub -1 /torso_controller/follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal '{header:{}, goal_id:{}, goal:{trajectory : {joint_names: [torso_lift_joint], points: [{positions : [0.35], time_from_start: [5.0, 0.0]}]}}}'
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_client_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_client_;

    std::vector<Pose> start_poses_;
    std::vector<Pose> target_poses_;

    // attaching spheres
    std::vector<AttachingSphere> attaching_spheres_;

    robot_model::RobotModelConstPtr moveit_robot_model_;
    itomp_exec::BoundingSphereRobotModelPtr robot_model_;

    // joint state listener
    itomp_exec::JointStateListener joint_state_listener_;
};

}


#endif // ITOMP_FETCH_FETCH_H
