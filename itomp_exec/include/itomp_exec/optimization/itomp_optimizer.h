#ifndef ITOMP_EXEC_ITOMP_OPTIMIZER_H
#define ITOMP_EXEC_ITOMP_OPTIMIZER_H


#include <itomp_exec/robot/bounding_sphere_robot_model.h>
#include <itomp_exec/robot/robot_state.h>
#include <dlib/optimization.h>
#include <Eigen/Dense>
#include <moveit_msgs/RobotTrajectory.h>
#include <ecl/geometry/polynomial.hpp>

#include <ros/ros.h>


namespace itomp_exec
{

class ITOMPOptimizer
{
private:
    
    typedef dlib::matrix<double,0,1> column_vector;
    
    // dlib/Eigen conversion utilities
    static const Eigen::VectorXd convertDlibToEigenVector(const column_vector& v);
    static const column_vector convertEigenToDlibVector(const Eigen::VectorXd& v);
    
    struct CostWeights
    {
        CostWeights()
            : smoothness_cost_weight(0.)
            , goal_pose_cost_weight(0.)
            , collision_cost_weight(0.)
        {}
            
        double smoothness_cost_weight;
        double goal_pose_cost_weight;
        double collision_cost_weight;
    };
    
    struct GoalLinkPose
    {
        GoalLinkPose()
            : position_weight(0.)
            , orientation_weight(0.)
        {}
        
        double position_weight;
        Eigen::Vector3d position;
        double orientation_weight;
        Eigen::Quaterniond orientation;
    };
    
public:
    
    ITOMPOptimizer();
    ~ITOMPOptimizer();
    
    void setCostWeight(const std::string& cost_type, double weight);
    
    inline void setUseNumericalDerivative(bool flag = true)
    {
        use_numerical_derivative_ = flag;
    }

    inline void setOptimizationTimeLimit(double time)
    {
        optimization_time_limit_ = time;
    }

    inline void setNumInterpolationSamples(int num_interpolation_samples)
    {
        num_interpolation_samples_ = num_interpolation_samples;
    }

    inline void setRobotModel(const BoundingSphereRobotModelPtr& robot_model)
    {
        robot_model_ = robot_model;
    }

    void setPlanningRobotStartState(const RobotState& start_state, double trajectory_duration, int num_milestones);
    
    void addGoalLinkPosition(const std::string& link_name, const Eigen::Vector3d& goal_position);
    void addGoalLinkOrientation(const std::string& link_name, const Eigen::Quaterniond& goal_orientation);
    
    double clampPosition(double value, int joint_index) const;
    double clampVelocity(double value, int joint_index) const;

    void stepForward(double time);

    void optimize();

    Eigen::VectorXd getOptimizationVariables();
    Eigen::VectorXd getOptimizationVariableLowerLimits();
    Eigen::VectorXd getOptimizationVariableUpperLimits();

    // visualize
    void setVisualizationTopic(ros::NodeHandle node_handle, const std::string& topic);
    void visualizeMilestones();
    void visualizeInterpolationSamples();
    void visualizeInterpolationSamplesCollisionSpheres();
    
    // robot trajectory conversion
    void getRobotTrajectoryIntervalMsg(moveit_msgs::RobotTrajectory& msg, double t0, double t1, int num_states);
    
private:

    // optimization
    // milestones are stored as column vectors [position'; velocities'] of dimension (2 * num_joints_)
    int num_joints_; //!< the number of planning joint
    int num_milestones_;
    Eigen::VectorXd start_milestone_;
    Eigen::MatrixXd milestones_; //!< changed while optimizing
    double trajectory_duration_;
    double optimization_time_limit_; //!< if 0, no time limit. optimization ends until it finds a minimum.
    int num_interpolation_samples_;
    
    Eigen::VectorXd optimization_variable_lower_limits_;
    Eigen::VectorXd optimization_variable_upper_limits_;
    
    // optimization objectives
    std::vector<GoalLinkPose> goal_link_poses_;
    
    // robot
    BoundingSphereRobotModelPtr robot_model_;
    RobotState start_state_;
    
    // numerical derivative
    bool use_numerical_derivative_;
    double numerical_derivative_eps_;

    // dlib function value/derivative evaluation
    CostWeights cost_weights_;
    void milestoneInitializeWithDlibVector(const column_vector& variables);
    double optimizationCost(const column_vector& variables);
    const column_vector optimizationCostDerivative(const column_vector& variables);
    const column_vector optimizationCostNumericalDerivative(const column_vector& variables);
    
    // temporary variables for optimization
    void allocateOptimizationResources();
    void precomputeOptimizationResources();
    void precomputeCubicPolynomials();
    void precomputeInterpolation(); //!< must be called after cubic polynomial precomputation
    void precomputeGoalLinkTransforms();
    void precomputeInterpolatedVariableTransforms();
    std::vector<std::vector<ecl::CubicPolynomial> > cubic_polynomials_;
    Eigen::MatrixXd interpolated_variables_; //!< all interpolated robot states, including start state
    std::vector<Eigen::Affine3d> goal_link_transforms_;
    std::vector<std::vector<Eigen::Affine3d> > interpolated_variable_link_transforms_; //!< [interpolation index][joint index]
    
    // ros publisher for visualization
    ros::Publisher visualization_publisher_;
};

}


#endif // ITOMP_EXEC_ITOMP_OPTIMIZER_H
