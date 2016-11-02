#ifndef ITOMP_EXEC_ITOMP_TRAJECTORY_H
#define ITOMP_EXEC_ITOMP_TRAJECTORY_H


#include <itomp_exec/robot/robot_model.h>
#include <itomp_exec/robot/robot_state.h>

#include <Eigen/Dense>


namespace itomp_exec
{

// Trajectory for time interval [0, T]. ([\Delta$ t, \Delta t + T] from outside planner's view)
class ItompTrajectory
{
public:

    ItompTrajectory(const RobotModel* robot_model);
    ~ItompTrajectory();

    void setRobotStartState(const RobotState& robot_start_state);
    void setPlanningGroup(const std::string& planning_group);

    void setNumWaypoints(int num_waypoints);
    void setTimestep(double timestep);
    void setTrajectoryDuration(double trajectory_duration);

    void initializeTrajectory();

    std::vector<RobotState> getInterpolatedFirstTimestepTrajectory(int num_interpolation_midpoints);
    std::vector<RobotState> getInterpolatedWholeTrajectory(int num_interpolation_midpoints);
    void extendByOneTimestep();

private:

    RobotState planningJointValuesToRobotState();

    const RobotModel* robot_model_;

    int num_waypoints_;
    double timestep_;
    double trajectory_duration_;
    int num_joints_;

    Eigen::VectorXd full_joint_initial_values_; // joint values, including non-planning joints
    Eigen::MatrixXd Q_; // waypoints, including the start configuration in the first column
    Eigen::VectorXd planning_joint_lower_limits_; // joint value limits followed, by velocity limits
    Eigen::VectorXd planning_joint_upper_limits_;
};

}


#endif // ITOMP_EXEC_ITOMP_TRAJECTORY_H
