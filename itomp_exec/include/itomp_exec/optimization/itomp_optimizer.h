#ifndef ITOMP_EXEC_ITOMP_OPTIMIZER_H
#define ITOMP_EXEC_ITOMP_OPTIMIZER_H


#include <itomp_exec/robot/bounding_sphere_robot_model.h>
#include <itomp_exec/robot/robot_state.h>
#include <itomp_exec/scene/planning_scene.h>
#include <itomp_exec/trajectory/itomp_trajectory.h>
#include <dlib/optimization.h>
#include <Eigen/Dense>
#include <moveit_msgs/RobotTrajectory.h>
#include <ecl/geometry/polynomial.hpp>
#include <itomp_exec/cost/cost_functions.h>

#include <ros/ros.h>


namespace itomp_exec
{

class ItompOptimizer
{
private:

    typedef dlib::matrix<double,0,1> column_vector;
    
    // dlib/Eigen conversion utilities
    static const Eigen::VectorXd convertDlibToEigenVector(const column_vector& v);
    static const column_vector convertEigenToDlibVector(const Eigen::MatrixXd& v);

public:
    
    ItompOptimizer(ItompTrajectory* trajectory);
    ~ItompOptimizer();

private:

    ItompTrajectory* trajectory_;
};

}


#endif // ITOMP_EXEC_ITOMP_OPTIMIZER_H
