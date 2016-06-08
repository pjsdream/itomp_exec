#ifndef ITOMP_EXEC_ITOMP_OPTIMIZER_H
#define ITOMP_EXEC_ITOMP_OPTIMIZER_H


#include <itomp_exec/robot/robot_model.h>
#include <itomp_exec/robot/robot_state.h>
#include <dlib/optimization.h>
#include <Eigen/Dense>

#include <ros/ros.h>


namespace itomp_exec
{

class ITOMPOptimizer
{
private:
    
    typedef dlib::matrix<double,0,1> column_vector;
    
public:
    
    ITOMPOptimizer();
    ~ITOMPOptimizer();
    
    inline void setOptimizationTimeLimit(double time)
    {
        optimization_time_limit_ = time;
    }

    void setPlanningRobotStartState(const RobotState& start_state, const std::string& planning_group_name, double trajectory_duration, int num_milestones);

    void stepForward(double time);

    void optimize();

    Eigen::VectorXd getOptimizationVariables();
    Eigen::VectorXd getOptimizationVariableLowerLimits();
    Eigen::VectorXd getOptimizationVariableUpperLimits();

    double cost();

    void setVisualizationTopic(ros::NodeHandle node_handle, const std::string& topic);
    void visualizeMilestones();
    void visualizeInterpolationSamples();
    
private:

    // optimization
    Eigen::VectorXd start_milestone_;
    Eigen::MatrixXd milestones_;
    double trajectory_duration_;
    double optimization_time_limit_; //!< if 0, no time limit. optimization ends until it finds a minimum.

    Eigen::VectorXd optimization_variable_lower_limits_;
    Eigen::VectorXd optimization_variable_upper_limits_;
    
    std::vector<int> planning_group_joint_indices_;
    
    // robot
    RobotState start_state_;

    // dlib function value/derivative evaluation
    double optimizationCost(const column_vector& variables);
    const column_vector optimizationCostDerivative(const column_vector& variables);
    static const Eigen::VectorXd convertDlibToEigenVector(const column_vector& v);
    static const column_vector convertEigenToDlibVector(const Eigen::VectorXd& v);
    
    // ros publisher for visualization
    ros::Publisher milestone_visualization_publisher_;
};

}


#endif // ITOMP_EXEC_ITOMP_OPTIMIZER_H
