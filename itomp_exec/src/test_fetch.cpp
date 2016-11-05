#include <itomp_exec/planner/itomp_planner.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

#include <itomp_exec/cost/goal_pose_cost.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <stdlib.h>
#include <time.h>


int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);
    
    srand(time(NULL));
    
    ros::init(argc, argv, "itomp_fetch");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("test_fetch");

    ROS_INFO("loading robot model");
    robot_model_loader::RobotModelLoader moveit_robot_model_loader("robot_description");
    robot_model::RobotModelPtr moveit_robot_model = moveit_robot_model_loader.getModel();
    robot_model::RobotState moveit_robot_state(moveit_robot_model);
    moveit_robot_state.setToDefaultValues();
    itomp_exec::PlanningRobotModel planning_robot_model( moveit_robot_state, "arm" );

    itomp_exec::PlanningRobotState planning_robot_state( &planning_robot_model );
    planning_robot_state.setZero();

    Eigen::Affine3d robot_base_transform = Eigen::Affine3d::Identity();

    ROS_INFO("loading planning scene");
    Eigen::Affine3d table_pose = Eigen::Affine3d::Identity();
    Eigen::Affine3d box_pose = Eigen::Affine3d::Identity();
    table_pose.translate(Eigen::Vector3d(1, 0, 0.25)).scale(Eigen::Vector3d(0.5, 1, 0.25));
    box_pose.translate(Eigen::Vector3d(0.8, 0.5, 0.525)).scale(0.025);
    itomp_exec::PlanningScene planning_scene;
    planning_scene.addStaticObstacle("package://itomp_fetch/env/cube.dae", table_pose);
    planning_scene.addObject("package://itomp_fetch/env/cube.dae", box_pose);

    ROS_INFO("setting up planner");
    itomp_exec::ItompPlanner planner;
    planner.setRobotModel(&planning_robot_model);
    planner.setRobotStartState(planning_robot_state);
    planner.setRobotBaseTransform(robot_base_transform);
    planner.setPlanningScene(&planning_scene);
    planner.setTimestep(0.5);
    planner.setTrajectoryDuration(3.0);
    planner.setNumWaypoints(6);

    planner.enableVisualizeTrajectoryEachStep();

    ROS_INFO("add goal cost function");
    itomp_exec::GoalPoseCost goal_pose_cost(1.);
    Eigen::Affine3d goal_pose;
    goal_pose.translate(Eigen::Vector3d(0.8, 0.5, 0.525)).rotate(Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d(0, 1, 0)));
    goal_pose_cost.setGoalPose("gripper_link", Eigen::Affine3d::Identity(), goal_pose);
    planner.setCostFunction(2, &goal_pose_cost);

    ROS_INFO("");
    planner.printCostFunctions();
    ROS_INFO("");

    planner.visualizePlanningScene();
    planner.planForOneTimestep();

    ros::shutdown();
    return 0;
}
