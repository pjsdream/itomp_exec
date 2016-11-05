#include <itomp_exec/planner/itomp_planner.h>

#include <moveit/robot_model_loader/robot_model_loader.h>


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

    ROS_INFO("loading planning scene");
    itomp_exec::PlanningScene planning_scene;
    planning_scene.addStaticObstacle("package://itomp_exec/env/table.dae", Eigen::Affine3d::Identity());

    ROS_INFO("setting up planner");
    itomp_exec::ItompPlanner planner;
    planner.setRobotModel(&planning_robot_model);
    planner.setPlanningScene(&planning_scene);
    planner.setTimestep(0.5);
    planner.setTrajectoryDuration(3.0);
    planner.setNumWaypoints(6);

    planner.enableVisualizeTrajectoryEachStep();

    planner.printCostFunctions();

    planner.planForOneTimestep();

    ros::shutdown();
    return 0;
}
