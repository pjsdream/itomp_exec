#include <itomp_fetch/fetch/fetch.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>


namespace itomp_fetch
{

Fetch::Fetch(const ros::NodeHandle& nh)
    : nh_(nh)
    , gripper_client_("gripper_controller/gripper_action")
    , torso_client_("torso_controller/follow_joint_trajectory")
    , arm_client_("arm_controller/follow_joint_trajectory")
{
    // action
    ROS_INFO("Waiting for controller servers");
    gripper_client_.waitForServer();
    torso_client_.waitForServer();
    arm_client_.waitForServer();

    // publisher initialization
    ros::Duration(0.5).sleep();
}

void Fetch::moveGripper(double gap, bool wait_for_execution)
{
    control_msgs::GripperCommandGoal goal;
    goal.command.position = gap;
    goal.command.max_effort = 1000.;
    gripper_client_.sendGoal(goal);

    if (wait_for_execution)
        gripper_client_.waitForResult();
}

void Fetch::closeGripper(bool wait_for_execution)
{
    moveGripper(0.00, wait_for_execution);
}

void Fetch::openGripper(bool wait_for_execution)
{
    moveGripper(0.10, wait_for_execution);
}

void Fetch::moveTorso(double position, bool wait_for_execution)
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
    {
        if (!torso_client_.waitForResult())
            ROS_ERROR("Torso action client failed to finish trajectory execution");
    }
}

}
