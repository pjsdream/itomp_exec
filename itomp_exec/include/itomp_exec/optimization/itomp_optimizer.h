#ifndef ITOMP_EXEC_ITOMP_OPTIMIZER_H
#define ITOMP_EXEC_ITOMP_OPTIMIZER_H


#include <itomp_exec/robot/bounding_sphere_robot_model.h>
#include <itomp_exec/robot/robot_state.h>
#include <itomp_exec/scene/planning_scene.h>
#include <dlib/optimization.h>
#include <Eigen/Dense>
#include <moveit_msgs/RobotTrajectory.h>
#include <ecl/geometry/polynomial.hpp>
#include <itomp_exec/cost/cost_functions.h>

#include <ros/ros.h>


namespace itomp_exec
{

class ITOMPOptimizer
{
private:

    typedef dlib::matrix<double,0,1> column_vector;
    
    // dlib/Eigen conversion utilities
    static const Eigen::VectorXd convertDlibToEigenVector(const column_vector& v);
    static const column_vector convertEigenToDlibVector(const Eigen::MatrixXd& v);

public:

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
    
    void clearCostWeights();
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

    inline void setPlanningScene(const PlanningScene* planning_scene)
    {
        planning_scene_ = planning_scene;
    }

    inline void setPlanningTimestep(double planning_timestep)
    {
        planning_timestep_ = planning_timestep;
    }

    inline void setDynamicObstacleMaxSpeed(double max_speed)
    {
        dynamic_obstacle_max_speed_ = max_speed;
    }

    inline void setDynamicObstacleDuration(double duration)
    {
        dynamic_obstacle_duration_ = duration;
    }

    inline void setRobotRootLinkTransform(const Eigen::Affine3d& transform)
    {
        root_link_transform_ = transform;
    }

    inline int getNumJoints() const
    {
        return num_joints_;
    }

    inline int getNumRobotJoints() const
    {
        return robot_model_->getNumJoints();
    }

    inline int getNumMilestones() const
    {
        return num_milestones_;
    }

    inline double getTrajectoryDuration() const
    {
        return trajectory_duration_;
    }

    inline double getDynamicObstacleDuration() const
    {
        return dynamic_obstacle_duration_;
    }

    inline double getPlanningTimestep() const
    {
        return planning_timestep_;
    }

    inline const ecl::CubicPolynomial& getCubicPolynomial(int joint_index, int milestone_index) const
    {
        return cubic_polynomials_[joint_index][milestone_index];
    }

    inline const Eigen::VectorXd& getStartMilestone() const
    {
        return start_milestone_;
    }

    inline const Eigen::MatrixXd& getMilestones() const
    {
        return milestones_;
    }

    inline const GoalLinkPose& getGoalLinkPose(int joint_index) const
    {
        return goal_link_poses_[joint_index];
    }

    inline const Eigen::Affine3d& getGoalLinkTransform(int joint_index) const
    {
        return goal_link_transforms_[joint_index];
    }

    inline int getPlanningJointIndex(int variable_index) const
    {
        return planning_joint_indices_[variable_index];
    }

    inline bool doesJointAffectLinkTransform(int joint_index_affecting, int link_index_affected) const
    {
        return joints_affecting_link_transforms_[joint_index_affecting][link_index_affected];
    }

    inline const BoundingSphereRobotModel& getRobotModel() const
    {
        return *robot_model_;
    }

    inline int getNumInterpolatedConfigurations() const
    {
        return num_interpolated_configurations_;
    }

    inline int getNumInterpolationSamples() const
    {
        return num_interpolation_samples_;
    }

    inline const Spheres& getInterpolatedLinkCollisionSpheres(int interpolation_index, int link_index) const
    {
        return interpolated_link_collision_spheres_[interpolation_index][link_index];
    }

    inline double getInterpolatedTime(int interpolation_index) const
    {
        return interpolated_times_[interpolation_index];
    }

    inline const Spheres& getStaticObstacleSpheres() const
    {
        return static_obstacle_spheres_;
    }

    inline const Spheres& getDynamicObstacleSpheres() const
    {
        return dynamic_obstacle_spheres_;
    }

    inline double getDynamicObstacleMaxSpeed() const
    {
        return dynamic_obstacle_max_speed_;
    }

    inline const Eigen::Affine3d& getInterpolatedLinkTransform(int interpolation_index, int link_index) const
    {
        return interpolated_link_transforms_[interpolation_index][link_index];
    }

    inline const std::pair<int, int>& getInterpolationIndexPosition(int interpolation_index) const
    {
        return interpolation_index_position_[interpolation_index];
    }

    inline const Eigen::Vector4d getInterpolatedCurveBasis(int interpolation_index) const
    {
        return interpolated_curve_bases_.row(interpolation_index).transpose();
    }

    inline const Eigen::VectorXd getInterpolatedVariables(int interpolation_index) const
    {
        return interpolated_variables_.col( interpolation_index );
    }

    inline double getOptimizationTimeLimit() const
    {
        return optimization_time_limit_;
    }

    // cost variables to be changed by cost functions
    inline double& cost()
    {
        return cost_;
    }

    inline Eigen::MatrixXd& milestoneDerivative()
    {
        return milestone_derivative_;
    }

    void setRobotModel(const BoundingSphereRobotModelPtr& robot_model);

    void setPlanningRobotStartState(const RobotState& start_state, double trajectory_duration, int num_milestones);
    void setPlanningRobotStartGoalStates(const RobotState& start_state, const RobotState& goal_state, double trajectory_duration, int num_milestones);
    void initializeRandomMilestones();
    void initializeLastMilestoneFromIK(const robot_model::RobotModel& moveit_robot_model, const robot_model::JointModelGroup *group);
    
    void clearGoalLinkPoses();
    void addGoalLinkPosition(const std::string& link_name, const Eigen::Vector3d& goal_position, double weight = 1.);
    void addGoalLinkOrientation(const std::string& link_name, const Eigen::Quaterniond& goal_orientation, double weight = 1.);

    double clampPosition(double value, int joint_index) const;
    double clampVelocity(double value, int joint_index) const;

    void stepForward(double time);
    void extend(double time);

    bool reachedGoalPose(double tolerance = 1e-6);

    void copyTrajectory(const ITOMPOptimizer& optimizer);

    // optimization functions with pthread capability
    // optimize() must be called in a created pthread; not in the main thread
    void optimize();
    void optimizeThreadCleanup();

    double trajectoryCost();

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

    // DEBUG: test gradients numerically
    void testGradients();
    
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

    // cost functions and cost/derivative variables to be changed by cost functions
    std::vector<Cost*> cost_functions_;
    double cost_;
    Eigen::MatrixXd milestone_derivative_;
    
    // robot
    BoundingSphereRobotModelPtr robot_model_;
    RobotState start_state_;
    Eigen::Affine3d root_link_transform_;

    // planning scene
    const PlanningScene* planning_scene_;
    Spheres static_obstacle_spheres_;

    // original ITOMP with conservative bound
    // scaling the radius: r(t) = r(0) * c * t
    // lookup t \in [delta t, 2 delta t]
    Spheres dynamic_obstacle_spheres_;
    double planning_timestep_;
    double dynamic_obstacle_max_speed_;
    double dynamic_obstacle_duration_;

    // variables and time stamp for optimization thread
    column_vector optimization_variables_;
    ros::Time optimization_start_time_;

    // TODO: ITOMP with motion prediction

    // numerical derivative
    bool use_numerical_derivative_;
    double numerical_derivative_eps_;

    // dlib function value/derivative evaluation
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
    void precomputeInterpolatedLinkTransforms();
    void precomputeInterpolatedLinkCollisionSpheres();
    int num_interpolated_configurations_;
    std::vector<std::vector<ecl::CubicPolynomial> > cubic_polynomials_; //!< [joint_index][milestone_index]
    Eigen::MatrixXd interpolated_variables_; //!< all interpolated robot states, including start state
    std::vector<Eigen::Affine3d> goal_link_transforms_;
    std::vector<std::vector<Eigen::Affine3d> > interpolated_link_transforms_; //!< [interpolation_index][link_index]
    std::vector<std::vector<Spheres> > interpolated_link_collision_spheres_; //!< [interpolation_index][link_index]
    Eigen::MatrixX4d interpolated_curve_bases_; //!< (interpolation_index, basis_index)  ex. row(0) = [B0 B1 B2 B3](t0). Bases are unnormalized i.e. B(t) = (t1-t0) H((t-t0)/(t1-t0))
    std::vector<std::pair<int, int> > interpolation_index_position_; //!< [interpolation_index] = ( milestone index, interpolation_index )
    std::vector<double> interpolated_times_; //!< uniform time samples between 0 and trajectory_duration

    // temporary variables for optimization
    void initializeJointsAffectingLinkTransforms();
    std::vector<std::vector<bool> > joints_affecting_link_transforms_; //!< [joint_index][link_index]
    
    // ros publisher for visualization
    ros::Publisher visualization_publisher_;
};

}


#endif // ITOMP_EXEC_ITOMP_OPTIMIZER_H
