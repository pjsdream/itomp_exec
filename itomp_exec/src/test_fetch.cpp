#include <itomp_exec/planner/itomp_planner_node.h>
#include <moveit_msgs/RobotState.h>
#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>

void initializeTuckState(moveit_msgs::RobotState& start_state)
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

int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    
    ros::init(argc, argv, "move_itomp");
    
    ros::NodeHandle nh("~");
    itomp_exec::ITOMPPlannerNode planner(nh);
    
    planner.printParams();
    planner.printControllers();
    planner.printCostWeights();
    
    planning_interface::MotionPlanRequest req;
    req.group_name = "arm";
    
    initializeTuckState(req.start_state);
    
    moveit_msgs::PositionConstraint goal_position_constraint;
    goal_position_constraint.link_name = "wrist_roll_link";
    Eigen::Vector3d goal_ee_position(1.0, 0.0, 0.0);
    tf::vectorEigenToMsg(goal_ee_position, goal_position_constraint.target_point_offset);
    
    moveit_msgs::OrientationConstraint goal_orientation_constraint;
    goal_orientation_constraint.link_name = "wrist_roll_link";
    Eigen::Quaterniond goal_ee_orientation(1.0, 0.0, 0.0, 0.0);
    tf::quaternionEigenToMsg(goal_ee_orientation, goal_orientation_constraint.orientation);
    
    moveit_msgs::Constraints goal_constraints;
    goal_constraints.position_constraints.push_back(goal_position_constraint);
    goal_constraints.orientation_constraints.push_back(goal_orientation_constraint);
    req.goal_constraints.push_back(goal_constraints);
    
    planner.setMotionPlanRequest(req);
    
    planning_interface::MotionPlanResponse res;
    planner.planAndExecute(res);
    
    return 0;
}
