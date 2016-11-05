#ifndef ITOMP_EXEC_ITOMP_TRAJECTORY_H
#define ITOMP_EXEC_ITOMP_TRAJECTORY_H


#include <itomp_exec/robot/planning_robot_model.h>
#include <itomp_exec/robot/planning_robot_state.h>

#include <Eigen/Dense>


namespace itomp_exec
{

// Trajectory for time interval [0, T]. ([\Delta$ t, \Delta t + T] from outside planner's view)
class ItompTrajectory
{
public:

    ItompTrajectory();
    ~ItompTrajectory();

    void setRobotModel(const PlanningRobotModel* robot_model);
    void setRobotStartState(const PlanningRobotState& robot_start_state);

    void setNumWaypoints(int num_waypoints);
    void setTimestep(double timestep);
    void setTrajectoryDuration(double trajectory_duration);
    void setWaypointVariables(const Eigen::MatrixXd& waypoints);

    const PlanningRobotModel* getRobotModel() const;
    int numWaypoints() const;
    double getTrajectoryDuration() const;
    Eigen::MatrixXd& getWaypointVariables();
    const Eigen::MatrixXd& getWaypointVariables() const;

    std::vector<PlanningRobotState> getInterpolatedFirstTimestepTrajectory(int num_interpolation_midpoints);
    std::vector<PlanningRobotState> getInterpolatedWholeTrajectory(int num_interpolation_midpoints);
    void extendByOneTimestep();

private:

    PlanningRobotState planningJointValuesToRobotState();

    const PlanningRobotModel* robot_model_;

    int num_waypoints_;
    double timestep_;
    double trajectory_duration_;
    int num_joints_;

    Eigen::MatrixXd Q_; // waypoints, including the start configuration in the first column
};

}


#endif // ITOMP_EXEC_ITOMP_TRAJECTORY_H
