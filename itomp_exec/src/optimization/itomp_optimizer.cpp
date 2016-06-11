#include <itomp_exec/optimization/itomp_optimizer.h>
#include <itomp_exec/planner/itomp_planner_node.h>
#include <visualization_msgs/MarkerArray.h>
#include <itomp_exec/util/gaussian_quadrature.h>

#include <functional>

#include <ros/ros.h>


namespace itomp_exec
{

ITOMPOptimizer::ITOMPOptimizer()
    : trajectory_duration_(0.)
    , use_numerical_derivative_(true)
    , numerical_derivative_eps_(1e-5)
{
}

ITOMPOptimizer::~ITOMPOptimizer()
{
}

void ITOMPOptimizer::setCostWeight(const std::string& cost_type, double weight)
{
    if (cost_type == "smoothness")
        cost_weights_.smoothness_cost_weight = weight;
    
    else if (cost_type == "goal_pose")
        cost_weights_.goal_pose_cost_weight = weight;
    
    else if (cost_type == "collision")
        cost_weights_.collision_cost_weight = weight;
    
    else
    {
        ROS_WARN("ITOMPOptimizer: Unknown cost type [%s]", cost_type.c_str());
    }
}

const Eigen::VectorXd ITOMPOptimizer::convertDlibToEigenVector(const column_vector& v)
{
    return Eigen::Map<Eigen::VectorXd>((double*)v.begin(), v.size());
}

const ITOMPOptimizer::column_vector ITOMPOptimizer::convertEigenToDlibVector(const Eigen::VectorXd& v)
{
    const int n = v.rows();
    column_vector r(n);
    memcpy(r.begin(), v.data(), sizeof(double) * n);
    return r;
}

void ITOMPOptimizer::setPlanningRobotStartState(const RobotState& start_state, double trajectory_duration, int num_milestones)
{
    trajectory_duration_ = trajectory_duration;
    num_joints_ = start_state.getNumPlanningJoints();
    num_milestones_ = num_milestones;
    
    // state stores all joint values as well as values of joints in planning group
    start_state_ = start_state;
    
    // initialize start milestone
    start_milestone_.resize(num_joints_ * 2);
    start_milestone_.block(0, 0, num_joints_, 1) = start_state.getPlanningJointPositions();
    start_milestone_.block(num_joints_, 0, num_joints_, 1) = start_state.getPlanningJointVelocities();
    
    // optimization variable limits
    const Eigen::VectorXd& joint_position_lower_limits = start_state.getPlanningJointPositionLowerLimits();
    const Eigen::VectorXd& joint_velocity_lower_limits = start_state.getPlanningJointVelocityLowerLimits();
    Eigen::VectorXd joint_position_velocity_lower_limits( joint_position_lower_limits.rows() + joint_velocity_lower_limits.rows() );
    joint_position_velocity_lower_limits << joint_position_lower_limits, joint_velocity_lower_limits;
    optimization_variable_lower_limits_ = joint_position_velocity_lower_limits.replicate( num_milestones, 1 );
    
    const Eigen::VectorXd& joint_position_upper_limits = start_state.getPlanningJointPositionUpperLimits();
    const Eigen::VectorXd& joint_velocity_upper_limits = start_state.getPlanningJointVelocityUpperLimits();
    Eigen::VectorXd joint_position_velocity_upper_limits( joint_position_upper_limits.rows() + joint_velocity_upper_limits.rows() );
    joint_position_velocity_upper_limits << joint_position_upper_limits, joint_velocity_upper_limits;
    optimization_variable_upper_limits_ = joint_position_velocity_upper_limits.replicate( num_milestones, 1 );
    
    // interpolate initial milestones
    milestones_.resize(num_joints_ * 2, num_milestones);
    for (int i=0; i<num_joints_; i++)
    {
        const ecl::CubicPolynomial poly = ecl::CubicDerivativeInterpolation(0., start_milestone_(i), start_milestone_(num_joints_ + i),
                                                                            trajectory_duration, start_milestone_(i), 0.);
        
        for (int j=0; j<num_milestones_; j++)
        {
            const double t = (double)(j+1) / (num_milestones_) * trajectory_duration_;
            milestones_(i, j) = clampPosition(poly(t), i);
            milestones_(num_joints_ + i, j) = clampVelocity(poly.derivative(t), i);
        }
    }
    
    // optimization objective
    goal_link_poses_.resize(robot_model_->getNumJoints());
    
    // the optimizer parameters are all set, now allocate memories for optimization
    allocateOptimizationResources();
}

void ITOMPOptimizer::addGoalLinkPosition(const std::string& link_name, const Eigen::Vector3d& goal_position)
{
    const int joint_index = robot_model_->getJointIndexByLinkName(link_name);
    goal_link_poses_[joint_index].position_weight = 1.0;
    goal_link_poses_[joint_index].position = goal_position;
}

void ITOMPOptimizer::addGoalLinkOrientation(const std::string& link_name, const Eigen::Quaterniond& goal_orientation)
{
    const int joint_index = robot_model_->getJointIndexByLinkName(link_name);
    goal_link_poses_[joint_index].orientation_weight = 1.0;
    goal_link_poses_[joint_index].orientation = goal_orientation;
}

double ITOMPOptimizer::clampPosition(double value, int joint_index) const
{
    if (value <= optimization_variable_lower_limits_[joint_index])
        return optimization_variable_lower_limits_[joint_index];
    
    if (value >= optimization_variable_upper_limits_[joint_index])
        return optimization_variable_upper_limits_[joint_index];
    
    return value;
}

double ITOMPOptimizer::clampVelocity(double value, int joint_index) const
{
    if (value <= optimization_variable_lower_limits_[num_joints_ + joint_index])
        return optimization_variable_lower_limits_[num_joints_ + joint_index];
    
    if (value >= optimization_variable_upper_limits_[num_joints_ + joint_index])
        return optimization_variable_upper_limits_[num_joints_ + joint_index];
    
    return value;
}

void ITOMPOptimizer::stepForward(double time)
{
    if (trajectory_duration_ <= time + 1e-6)
    {
        trajectory_duration_ = 0.;
        return;
    }
    
    // update milestones
    precomputeCubicPolynomials();
    
    int poly_index = 0;
    for (int i=-1; i<num_milestones_; i++)
    {
        const double u = (double)(i+1) / num_milestones_;
        const double t = (1.-u) * time + u * trajectory_duration_;
        
        while ((double)(poly_index + 1) / num_milestones_ * trajectory_duration_ < t)
            poly_index++;
        
        for (int j=0; j<num_joints_; j++)
        {
            const ecl::CubicPolynomial& poly = cubic_polynomials_[j][poly_index];
            
            if (i==-1)
            {
                start_milestone_(j) = clampPosition(poly(t), j);
                start_milestone_(num_joints_ + j) = clampVelocity(poly.derivative(t), j);
            }
            else
            {
                milestones_(j, i) = clampPosition(poly(t), j);
                milestones_(num_joints_ + j, i) = clampVelocity(poly.derivative(t), j);
            }
        }
    }

    trajectory_duration_ -= time;
}

void ITOMPOptimizer::optimize()
{
    ros::WallTime start_time = ros::WallTime::now();
    
    column_vector initial_variables = convertEigenToDlibVector( getOptimizationVariables() );
    column_vector lower = convertEigenToDlibVector( getOptimizationVariableLowerLimits() );
    column_vector upper = convertEigenToDlibVector( getOptimizationVariableUpperLimits() );
    
    const int optimization_max_iter = 10;
    
    while ((ros::WallTime::now() - start_time).toSec() < optimization_time_limit_)
    {
        if (use_numerical_derivative_)
        {
            dlib::find_min_box_constrained(
                        dlib::bfgs_search_strategy(),
                        dlib::objective_delta_stop_strategy(1e-7, optimization_max_iter),
                        std::bind(&ITOMPOptimizer::optimizationCost, this, std::placeholders::_1),
                        std::bind(&ITOMPOptimizer::optimizationCostNumericalDerivative, this, std::placeholders::_1),
                        initial_variables,
                        lower,
                        upper
                        );
        }
        else
        {
            dlib::find_min_box_constrained(
                        dlib::bfgs_search_strategy(),
                        dlib::objective_delta_stop_strategy(1e-7, optimization_max_iter),
                        std::bind(&ITOMPOptimizer::optimizationCost, this, std::placeholders::_1),
                        std::bind(&ITOMPOptimizer::optimizationCostDerivative, this, std::placeholders::_1),
                        initial_variables,
                        lower,
                        upper
                        );
        }
        
        // derivative test
        /*
        const double c = optimizationCost(initial_variables);
        column_vector d = optimizationCostNumericalDerivative(initial_variables);
        const double cd = optimizationCost(initial_variables + d * numerical_derivative_eps_);
        printf("%lf -> %lf, (diff: %lf)\n", c, cd, (cd - c) / numerical_derivative_eps_);
        */
        
        // visualize trajectory
        //visualizeMilestones();
        //visualizeInterpolationSamples();
        //visualizeInterpolationSamplesCollisionSpheres();
    }
    
    milestoneInitializeWithDlibVector(initial_variables);
}

Eigen::VectorXd ITOMPOptimizer::getOptimizationVariables()
{
    return Eigen::Map<Eigen::VectorXd>(milestones_.data(), milestones_.rows() * milestones_.cols());
}

Eigen::VectorXd ITOMPOptimizer::getOptimizationVariableLowerLimits()
{
    return optimization_variable_lower_limits_;
}

Eigen::VectorXd ITOMPOptimizer::getOptimizationVariableUpperLimits()
{
    return optimization_variable_upper_limits_;
}

// dlib functions
void ITOMPOptimizer::milestoneInitializeWithDlibVector(const column_vector& variables)
{
    memcpy(milestones_.data(), variables.begin(), sizeof(variables(0)) * variables.size());
}

double ITOMPOptimizer::optimizationCost(const column_vector& variables)
{
    milestoneInitializeWithDlibVector(variables);
    precomputeOptimizationResources();
    
    double cost = 0.;
    
    // smoothness cost
    if (cost_weights_.smoothness_cost_weight != 0.)
    {
        for (int i=0; i<num_joints_; i++)
        {
            for (int j=0; j<num_milestones_; j++)
            {
                const double t0 = (double)j / num_milestones_ * trajectory_duration_;
                const double t1 = (double)(j+1) / num_milestones_ * trajectory_duration_;
                const ecl::CubicPolynomial& poly = cubic_polynomials_[i][j];
                
                ecl::LinearFunction acc = poly.derivative().derivative();
                const double a0 = acc.coefficients()[0];
                const double a1 = acc.coefficients()[1];
                
                ecl::QuadraticPolynomial acc_square;
                acc_square.coefficients() << a0*a0, 2*a0*a1, a1*a1;
                
                // normalize with trajectory duration
                cost += gaussianQuadratureQuadraticPolynomial(t0, t1, acc_square) * trajectory_duration_ * cost_weights_.smoothness_cost_weight;
            }
        }
    }
    
    // goal pose cost
    if (cost_weights_.goal_pose_cost_weight != 0.)
    {
        for (int i=0; i<goal_link_poses_.size(); i++)
        {
            const double& position_weight = goal_link_poses_[i].position_weight;
            const double& orientation_weight = goal_link_poses_[i].orientation_weight;
            
            if (position_weight != 0.)
            {
                const Eigen::Vector3d& link_position = goal_link_transforms_[i].translation();
                const Eigen::Vector3d& target_position = goal_link_poses_[i].position;
                
                cost += (link_position - target_position).squaredNorm() * cost_weights_.goal_pose_cost_weight;
            }
            
            if (orientation_weight != 0.)
            {
                Eigen::Quaterniond link_orientation( goal_link_transforms_[i].linear() );
                Eigen::Quaterniond target_orientation = goal_link_poses_[i].orientation;
                
                const double ratio_cosine_to_meter = 1.0;
                cost += (1 - std::abs(link_orientation.dot(target_orientation))) * ratio_cosine_to_meter * cost_weights_.goal_pose_cost_weight;
            }
        }
        
        // penalize velocities at last state
        const double ratio_radian_per_sec_to_meter = 10.0;
        for (int i=0; i<num_joints_; i++)
        {
            const double v = milestones_(num_joints_ + i, num_milestones_ - 1);
            cost += (v * v) * ratio_radian_per_sec_to_meter * cost_weights_.goal_pose_cost_weight;
        }
    }
    
    // TODO: collision cost
    
    return cost;
}

const ITOMPOptimizer::column_vector ITOMPOptimizer::optimizationCostDerivative(const column_vector& variables)
{
    milestoneInitializeWithDlibVector(variables);
    precomputeOptimizationResources();
    
    // TODO
    Eigen::VectorXd v(variables.size());
    v.setZero();
    return convertEigenToDlibVector( v );
}

const ITOMPOptimizer::column_vector ITOMPOptimizer::optimizationCostNumericalDerivative(const column_vector& variables)
{
    const int num_variables = variables.size();
    
    column_vector delta_variables = variables;
    column_vector derivative(num_variables);
    
    for (int i=0; i<num_variables; i++)
    {
        delta_variables(i) += numerical_derivative_eps_;
        const double cost_forward = optimizationCost(delta_variables);
        delta_variables(i) -= 2. * numerical_derivative_eps_;
        const double cost_backward = optimizationCost(delta_variables);
        delta_variables(i) += numerical_derivative_eps_;
        
        derivative(i) = (cost_forward - cost_backward) / 2. / numerical_derivative_eps_;
    }
    
    return derivative;
}

void ITOMPOptimizer::allocateOptimizationResources()
{
    cubic_polynomials_.resize(num_joints_);
    for (int i=0; i<num_joints_; i++)
        cubic_polynomials_[i].resize(num_milestones_);
    
    interpolated_variables_.resize(num_joints_ * 2, num_milestones_ * (num_interpolation_samples_ + 1) + 1);
    interpolated_variable_link_transforms_.resize(num_milestones_ * (num_interpolation_samples_ + 1) + 1);
}

void ITOMPOptimizer::precomputeOptimizationResources()
{
    precomputeCubicPolynomials();
    precomputeInterpolation();
    precomputeGoalLinkTransforms();
    precomputeInterpolatedVariableTransforms();
}

void ITOMPOptimizer::precomputeCubicPolynomials()
{
    for (int i=0; i<num_joints_; i++)
    {
        for (int j=0; j<num_milestones_; j++)
        {
            const double t0 = (double)j / num_milestones_ * trajectory_duration_;
            const double p0 = j==0 ? start_milestone_(i) : milestones_(i, j-1);
            const double v0 = j==0 ? start_milestone_(num_joints_ + i) : milestones_(num_joints_ + i, j-1);
            const double t1 = (double)(j+1) / num_milestones_ * trajectory_duration_;
            const double p1 = milestones_(i, j);
            const double v1 = milestones_(num_joints_ + i, j);
            
            cubic_polynomials_[i][j] = ecl::CubicDerivativeInterpolation(t0, p0, v0, t1, p1, v1);
        }
    }
}

void ITOMPOptimizer::precomputeInterpolation()
{
    interpolated_variables_.col(0) = start_milestone_;
    for (int i=0; i<num_joints_; i++)
    {
        int column_index = 1;
        
        for (int j=0; j<num_milestones_; j++)
        {
            const double t0 = (double)j / num_milestones_ * trajectory_duration_;
            const double t1 = (double)(j+1) / num_milestones_ * trajectory_duration_;
            const ecl::CubicPolynomial& poly = cubic_polynomials_[i][j];
            
            for (int k=0; k <= num_interpolation_samples_; k++)
            {
                const double u = (double)(k+1) / (num_interpolation_samples_ + 1);
                const double t = (1-u) * t0 + u * t1;
                
                interpolated_variables_(i, column_index) = clampPosition(poly(t), i);
                interpolated_variables_(num_joints_ + i, column_index) = clampVelocity(poly.derivative(t), i);
                column_index++;
            }
        }
    }
}

void ITOMPOptimizer::precomputeGoalLinkTransforms()
{
    Eigen::VectorXd joint_positions = start_state_.getDefaultJointPositions();
    const std::vector<int>& planning_joint_indices = start_state_.getPlanningJointIndices();

    for (int i=0; i<num_joints_; i++)
    {
        const int joint_index = planning_joint_indices[i];
        joint_positions(joint_index) = milestones_(i, num_milestones_ - 1);
    }

    robot_model_->getLinkTransforms(joint_positions, goal_link_transforms_);
}

void ITOMPOptimizer::precomputeInterpolatedVariableTransforms()
{
    Eigen::VectorXd joint_positions = start_state_.getDefaultJointPositions();
    const std::vector<int>& planning_joint_indices = start_state_.getPlanningJointIndices();
    
    for (int i=0; i<interpolated_variables_.cols(); i++)
    {
        for (int j=0; j<num_joints_; j++)
        {
            const int joint_index = planning_joint_indices[j];
            joint_positions(joint_index) = interpolated_variables_(j, i);
        }

        robot_model_->getLinkTransforms(joint_positions, interpolated_variable_link_transforms_[i]);
    }
}

void ITOMPOptimizer::setVisualizationTopic(ros::NodeHandle node_handle, const std::string& topic)
{
    visualization_publisher_ = node_handle.advertise<visualization_msgs::MarkerArray>(topic, 1);
}

void ITOMPOptimizer::visualizeMilestones()
{
    // TODO
}

void ITOMPOptimizer::visualizeInterpolationSamples()
{
    visualization_msgs::MarkerArray marker_array;

    for (int i=0; i<interpolated_variable_link_transforms_.size(); i++)
        robot_model_->pushVisualLinkVisualizationMarkers(interpolated_variable_link_transforms_[i], "interpolated_" + std::to_string(i), marker_array);

    visualization_publisher_.publish(marker_array);
}

void ITOMPOptimizer::visualizeInterpolationSamplesCollisionSpheres()
{
    visualization_msgs::MarkerArray marker_array;

    for (int i=0; i<interpolated_variable_link_transforms_.size(); i++)
        robot_model_->pushCollisionSpheresVisualizationMarkers(interpolated_variable_link_transforms_[i], "interpolated_collision_" + std::to_string(i), marker_array);

    visualization_publisher_.publish(marker_array);
}

void ITOMPOptimizer::getRobotTrajectoryIntervalMsg(moveit_msgs::RobotTrajectory& msg, double t0, double t1, int num_states)
{
    precomputeCubicPolynomials();
    
    msg.joint_trajectory.joint_names = start_state_.getPlanningJointNames();
    
    int state_index = 0;
    
    for (int i=0; i<num_milestones_; i++)
    {
        const double milestone_t0 = (double)i / num_milestones_ * trajectory_duration_;
        const double milestone_t1 = (double)(i + 1) / num_milestones_ * trajectory_duration_;
        
        while (state_index < num_states)
        {
            const double u = (double)state_index / (num_states - 1);
            const double t = (1.-u) * t0 + u * t1;
            
            if (milestone_t0 <= t && t<= milestone_t1)
            {
                trajectory_msgs::JointTrajectoryPoint point;
                point.time_from_start = ros::Duration(t);
                for (int j=0; j<num_joints_; j++)
                {
                    const ecl::CubicPolynomial& poly = cubic_polynomials_[j][i];
                    
                    point.positions.push_back( poly(t) );
                    point.velocities.push_back( poly.derivative(t) );
                }
                msg.joint_trajectory.points.push_back(point);
                
                state_index++;
            }
            else break;
        }
        
        if (state_index == num_states)
            break;
    }
}

}
