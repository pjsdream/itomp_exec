#ifndef ITOMP_FETCH_FETCH_H
#define ITOMP_FETCH_FETCH_H


#include <ros/ros.h>
#include <Eigen/Dense>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <itomp_exec/planner/itomp_planner.h>
#include <itomp_exec/robot/bounding_sphere_robot_model.h>
#include <itomp_exec/util/joint_state_listener.h>


namespace itomp_fetch
{

class Fetch
{
public:

    Fetch(const ros::NodeHandle& nh = ros::NodeHandle("~"));

    void moveGripper(double gap, bool wait_for_execution = true);
    void openGripper(bool wait_for_execution = true);
    void closeGripper(bool wait_for_execution = true);

    void moveTorso(double position, bool wait_for_execution = true);

private:

    ros::NodeHandle nh_;

    // gripper actionlib client
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client_;

    // torso actionlib client
    // rostopic pub -1 /torso_controller/follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal '{header:{}, goal_id:{}, goal:{trajectory : {joint_names: [torso_lift_joint], points: [{positions : [0.35], time_from_start: [5.0, 0.0]}]}}}'
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_client_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_client_;

    // joint state listener
    itomp_exec::JointStateListener joint_state_listener_;
};

}


#endif // ITOMP_FETCH_FETCH_H
