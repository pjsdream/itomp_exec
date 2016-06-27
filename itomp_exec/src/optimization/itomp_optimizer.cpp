#include <itomp_exec/optimization/itomp_optimizer.h>
#include <itomp_exec/planner/itomp_planner_node.h>
#include <visualization_msgs/MarkerArray.h>
#include <itomp_exec/util/gaussian_quadrature.h>
#include <itomp_exec/optimization/optimization_stop_strategy.h>
#include <itomp_exec/optimization/optimization_search_strategy.h>
#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>

#include <omp.h>


namespace itomp_exec
{

ITOMPOptimizer::ITOMPOptimizer()
    : trajectory_duration_(0.)
    , use_numerical_derivative_(true)
    , numerical_derivative_eps_(1e-5)
    , planning_scene_(0)
{
}

ITOMPOptimizer::~ITOMPOptimizer()
{
    for (int i=0; i<cost_functions_.size(); i++)
        delete cost_functions_[i];
}

void ITOMPOptimizer::setCostWeight(const std::string& cost_type, double weight)
{
    if (cost_type == "smoothness")
        cost_functions_.push_back(new SmoothnessCost(*this, weight));
    
    else if (cost_type == "goal_pose")
        cost_functions_.push_back(new GoalPoseCost(*this, weight));
    
    else if (cost_type == "collision")
        cost_functions_.push_back(new CollisionCost(*this, weight));
    
    else
        ROS_WARN("ITOMPOptimizer: Unknown cost type [%s]", cost_type.c_str());
}

const Eigen::VectorXd ITOMPOptimizer::convertDlibToEigenVector(const column_vector& v)
{
    return Eigen::Map<Eigen::VectorXd>((double*)v.begin(), v.size());
}

const ITOMPOptimizer::column_vector ITOMPOptimizer::convertEigenToDlibVector(const Eigen::MatrixXd& v)
{
    const int n = v.rows() * v.cols();
    column_vector r(n);
    memcpy(r.begin(), v.data(), sizeof(double) * n);
    return r;
}

void ITOMPOptimizer::setRobotModel(const BoundingSphereRobotModelPtr& robot_model)
{
    robot_model_ = robot_model;

    // optimization objective
    goal_link_poses_.resize(robot_model_->getNumJoints());
}

void ITOMPOptimizer::setPlanningRobotStartState(const RobotState& start_state, double trajectory_duration, int num_milestones)
{
    trajectory_duration_ = trajectory_duration;
    num_joints_ = start_state.getNumPlanningJoints();
    num_milestones_ = num_milestones;

    // planning joint indices
    planning_joint_indices_ = start_state.getPlanningJointIndices();

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
    milestone_derivative_.resize(num_joints_ * 2, num_milestones);
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

    initializeJointsAffectingLinkTransforms();

    // the optimizer parameters are all set, now allocate memories for optimization
    allocateOptimizationResources();
}

void ITOMPOptimizer::setPlanningRobotStartGoalStates(const RobotState& start_state, const RobotState& goal_state, double trajectory_duration, int num_milestones)
{
    setPlanningRobotStartState(start_state, trajectory_duration, num_milestones);
    
    // overwrite the interpolated milestone variables
    milestones_.resize(num_joints_ * 2, num_milestones);
    for (int i=0; i<num_joints_; i++)
    {
        const ecl::CubicPolynomial poly = ecl::CubicDerivativeInterpolation(0., start_milestone_(i), start_milestone_(num_joints_ + i),
                                                                            trajectory_duration, goal_state.getPlanningJointPositions()(i), goal_state.getPlanningJointVelocities()(i));
        
        for (int j=0; j<num_milestones_; j++)
        {
            const double t = (double)(j+1) / (num_milestones_) * trajectory_duration_;
            milestones_(i, j) = clampPosition(poly(t), i);
            milestones_(num_joints_ + i, j) = clampVelocity(poly.derivative(t), i);
        }
    }
}

void ITOMPOptimizer::clearGoalLinkPoses()
{
    for (int i=0; i<goal_link_poses_.size(); i++)
    {
        goal_link_poses_[i].position_weight = 0.;
        goal_link_poses_[i].orientation_weight = 0.;
    }
}

void ITOMPOptimizer::addGoalLinkPosition(const std::string& link_name, const Eigen::Vector3d& goal_position)
{
    const int joint_index = robot_model_->getJointIndexByLinkName(link_name);
    goal_link_poses_[joint_index].position_weight = 1.0;
    goal_link_poses_[joint_index].position = root_link_transform_ * goal_position;
}

void ITOMPOptimizer::addGoalLinkOrientation(const std::string& link_name, const Eigen::Quaterniond& goal_orientation)
{
    const int joint_index = robot_model_->getJointIndexByLinkName(link_name);
    goal_link_poses_[joint_index].orientation_weight = 1.0;
    goal_link_poses_[joint_index].orientation = root_link_transform_.linear() * goal_orientation;
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
    ros::Time start_time = ros::Time::now();

    // update static obstacle spheres
    static_obstacle_spheres_ = planning_scene_->getStaticSphereObstacles();

    // update dynamic obstacle spheres at current time
    dynamic_obstacle_spheres_ = planning_scene_->getDynamicSphereObstacles();
    
    column_vector initial_variables = convertEigenToDlibVector( getOptimizationVariables() );
    column_vector lower = convertEigenToDlibVector( getOptimizationVariableLowerLimits() );
    column_vector upper = convertEigenToDlibVector( getOptimizationVariableUpperLimits() );

    // stop strategy is both time limit and objective delta
    coupled_stop_strategy<itomp_exec::time_limit_stop_strategy, dlib::objective_delta_stop_strategy>
            stop_strategy(itomp_exec::time_limit_stop_strategy(optimization_time_limit_, start_time), dlib::objective_delta_stop_strategy(1e-5, 1000000));

    stop_strategy.be_verbose();

    if (use_numerical_derivative_)
    {
        dlib::find_min_box_constrained(
                    itomp_exec::bfgs_search_strategy(),
                    stop_strategy,
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
                    itomp_exec::bfgs_search_strategy(),
                    stop_strategy,
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
    visualizeInterpolationSamples();
    //visualizeInterpolationSamplesCollisionSpheres();
    
    milestoneInitializeWithDlibVector(initial_variables);

    ROS_INFO("Optimization elapsed time: %lf sec", (ros::Time::now() - start_time).toSec());
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

    cost_ = 0.;

    for (int i=0; i<cost_functions_.size(); i++)
        cost_functions_[i]->addCost();

    return cost_;
}

const ITOMPOptimizer::column_vector ITOMPOptimizer::optimizationCostDerivative(const column_vector& variables)
{
    milestoneInitializeWithDlibVector(variables);
    precomputeOptimizationResources();

    milestone_derivative_.setZero();

    for (int i=0; i<cost_functions_.size(); i++)
        cost_functions_[i]->addDerivative();

    return convertEigenToDlibVector( milestone_derivative_ );
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
    
    num_interpolated_configurations_ = num_milestones_ * (num_interpolation_samples_ + 1) + 1;

    interpolated_variables_.resize(num_joints_ * 2, num_interpolated_configurations_);
    interpolated_variable_link_transforms_.resize(num_interpolated_configurations_);

    interpolated_collision_spheres_.resize(num_interpolated_configurations_);

    interpolation_index_position_.resize(num_interpolated_configurations_);

    interpolated_curve_bases_.resize(num_interpolation_samples_ + 1, Eigen::NoChange);

    interpolated_times_.resize(num_interpolated_configurations_);
}

void ITOMPOptimizer::precomputeOptimizationResources()
{
    precomputeCubicPolynomials();
    precomputeInterpolation();
    precomputeGoalLinkTransforms();
    precomputeInterpolatedVariableTransforms();
    precomputeInterpolatedCollisionSpheres();
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
    interpolation_index_position_[0] = std::make_pair(0, 0);

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

                interpolation_index_position_[column_index] = std::make_pair(j, k);

                interpolated_times_[column_index] = t;

                column_index++;
            }
        }
    }

    // curve bases for each interpolation samples
    const double t = trajectory_duration_ / num_milestones_;
    for (int i=0; i <= num_interpolation_samples_; i++)
    {
        const double u = (double)(i+1) / (num_interpolation_samples_ + 1);
        const double u2 = u*u;
        const double u3 = u2*u;

        const double h0 = 2. * u3 - 3. * u2 + 1.;
        const double h1 = u3 - 2. * u2 + u;
        const double h2 = -2. * u3 + 3. * u2;
        const double h3 = u3 - u2;

        interpolated_curve_bases_.row(i) = Eigen::Vector4d(h0, h1 * t, h2, h3 * t).transpose();
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

    for (int i=0; i<goal_link_transforms_.size(); i++)
        goal_link_transforms_[i] = root_link_transform_ * goal_link_transforms_[i];
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

        for (int j=0; j<interpolated_variable_link_transforms_[i].size(); j++)
            interpolated_variable_link_transforms_[i][j] = root_link_transform_ * interpolated_variable_link_transforms_[i][j];
    }
}

void ITOMPOptimizer::precomputeInterpolatedCollisionSpheres()
{
    // interpolated variables
    for (int i=0; i<interpolated_variable_link_transforms_.size(); i++)
        robot_model_->getCollisionSpheres(interpolated_variable_link_transforms_[i], interpolated_collision_spheres_[i]);
}

void ITOMPOptimizer::initializeJointsAffectingLinkTransforms()
{
    // allocate memory
    joints_affecting_link_transforms_.resize(num_joints_);
    for (int i=0; i<num_joints_; i++)
        joints_affecting_link_transforms_[i].resize(robot_model_->getNumJoints(), false);

    for (int i=0; i<num_joints_; i++)
    {
        std::vector<int> descendant_joint_indices = robot_model_->getDescendantJointIndices(planning_joint_indices_[i]);
        for (int j=0; j<descendant_joint_indices.size(); j++)
            joints_affecting_link_transforms_[i][ descendant_joint_indices[j] ] = true;
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

    // link transforms are w.r.t "map"
    for (int i=0; i<marker_array.markers.size(); i++)
        marker_array.markers[i].header.frame_id = "map";

    visualization_publisher_.publish(marker_array);
}

void ITOMPOptimizer::visualizeInterpolationSamplesCollisionSpheres()
{
    visualization_msgs::MarkerArray marker_array;

    for (int i=0; i<interpolated_variable_link_transforms_.size(); i++)
        robot_model_->pushCollisionSpheresVisualizationMarkers(interpolated_variable_link_transforms_[i], "interpolated_collision_" + std::to_string(i), marker_array);

    // link transforms are w.r.t "map"
    for (int i=0; i<marker_array.markers.size(); i++)
        marker_array.markers[i].header.frame_id = "map";

    visualization_publisher_.publish(marker_array);
}

void ITOMPOptimizer::visualizePlanningScene()
{
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;

    marker.color.a = 1.;
    
    marker.pose.orientation.w = 1.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    
    // static obstacles as green
    marker.ns = "planning_scene_static";
    marker.color.r = 0.;
    marker.color.g = 1.;
    marker.color.b = 0.;
    for (int i=0; i<static_obstacle_spheres_.size(); i++)
    {
        const Sphere& sphere = static_obstacle_spheres_[i];
      
        marker.id = i;
        
        tf::pointEigenToMsg(sphere.position, marker.pose.position);
        marker.scale.x = marker.scale.y = marker.scale.z = sphere.radius * 2.;
        
        marker_array.markers.push_back(marker);
    }

    // dynamic obstacles as red
    marker.ns = "planning_scene_dynamic";
    marker.color.r = 1.;
    marker.color.g = 0.;
    for (int i=0; i<dynamic_obstacle_spheres_.size(); i++)
    {
        const Sphere& sphere = dynamic_obstacle_spheres_[i];

        marker.id = i;

        tf::pointEigenToMsg(sphere.position, marker.pose.position);
        marker.scale.x = marker.scale.y = marker.scale.z = sphere.radius * 2.;

        marker_array.markers.push_back(marker);
    }

    // future dynamic obstacles as transparent red
    marker.ns = "planning_scene_dynamic_future";
    marker.color.a = 0.5;
    for (int i=0; i<dynamic_obstacle_spheres_.size(); i++)
    {
        const Sphere& sphere = dynamic_obstacle_spheres_[i];

        marker.id = i;

        tf::pointEigenToMsg(sphere.position, marker.pose.position);
        marker.scale.x = marker.scale.y = marker.scale.z = (sphere.radius + dynamic_obstacle_duration_ * dynamic_obstacle_max_speed_) * 2.;

        marker_array.markers.push_back(marker);
    }

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
