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
    
    static double ratio_cosine_to_meter_;
    static double ratio_radian_per_sec_to_meter_;

    typedef dlib::matrix<double,0,1> column_vector;
    
    // dlib/Eigen conversion utilities
    static const Eigen::VectorXd convertDlibToEigenVector(const column_vector& v);
    static const column_vector convertEigenToDlibVector(const Eigen::MatrixXd& v);
    
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

    void addStaticObstalceSphere(double radius, const Eigen::Vector3d& position);

    void setRobotModel(const BoundingSphereRobotModelPtr& robot_model);

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
    std::vector<int> planning_joint_indices_; //!< planning group joint index to whole joint set index
    
    Eigen::VectorXd optimization_variable_lower_limits_;
    Eigen::VectorXd optimization_variable_upper_limits_;
    
    // optimization objectives
    std::vector<GoalLinkPose> goal_link_poses_;
    
    // robot
    BoundingSphereRobotModelPtr robot_model_;
    RobotState start_state_;

    // obstacles
    BoundingSphereRobotModel::Spheres static_obstacle_spheres_;

    // numerical derivative
    bool use_numerical_derivative_;
    double numerical_derivative_eps_;

    // dlib function value/derivative evaluation
    CostWeights cost_weights_;
    void milestoneInitializeWithDlibVector(const column_vector& variables);
    double optimizationCost(const column_vector& variables);
    const column_vector optimizationCostDerivative(const column_vector& variables);
    const column_vector optimizationCostNumericalDerivative(const column_vector& variables);
    
    // temporary variables for optimization with respect to optimization variables
    void allocateOptimizationResources();
    void precomputeOptimizationResources();
    void precomputeCubicPolynomials();
    void precomputeInterpolation(); //!< must be called after cubic polynomial precomputation
    void precomputeGoalLinkTransforms();
    void precomputeInterpolatedVariableTransforms();
    void precomputeInterpolatedCollisionSpheres();
    std::vector<std::vector<ecl::CubicPolynomial> > cubic_polynomials_;
    Eigen::MatrixXd interpolated_variables_; //!< all interpolated robot states, including start state
    std::vector<Eigen::Affine3d> goal_link_transforms_;
    std::vector<std::vector<Eigen::Affine3d> > interpolated_variable_link_transforms_; //!< [interpolation_index][link_index]
    std::vector<std::vector<BoundingSphereRobotModel::Spheres> > interpolated_collision_spheres_; //!< [interpolation_index][link_index]
    Eigen::MatrixX4d interpolated_curve_bases_; //!< (interpolation_index, basis_index)  ex. row(0) = [B0 B1 B2 B3](t0). Bases are unnormalized i.e. B(t) = (t1-t0) H((t-t0)/(t1-t0))
    std::vector<std::pair<int, int> > interpolation_index_position_; //!< [interpolation_index] = ( milestone index, interpolation_index )

    // temporary variables for optimization
    void initializeSmoothnessCostDerivaiveAuxilaryMatrix();
    void initializeJointsAffectingLinkTransforms();
    Eigen::Matrix4d H2_; //!< for smoothness cost derivative computation
    std::vector<std::vector<bool> > joints_affecting_link_transforms_; //!< [joint_index][link_index]
    
    // ros publisher for visualization
    ros::Publisher visualization_publisher_;
};

}


#endif // ITOMP_EXEC_ITOMP_OPTIMIZER_H
