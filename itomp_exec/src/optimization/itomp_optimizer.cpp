#include <itomp_exec/optimization/itomp_optimizer.h>
#include <itomp_exec/planner/itomp_planner_node.h>
#include <visualization_msgs/MarkerArray.h>
#include <itomp_exec/util/gaussian_quadrature.h>

#include <functional>

#include <ros/ros.h>


namespace itomp_exec
{

double ITOMPOptimizer::ratio_cosine_to_meter_ = 2.;
double ITOMPOptimizer::ratio_radian_per_sec_to_meter_ = 10.;

ITOMPOptimizer::ITOMPOptimizer()
    : trajectory_duration_(0.)
    , use_numerical_derivative_(true)
    , numerical_derivative_eps_(1e-5)
{
    initializeSmoothnessCostDerivaiveAuxilaryMatrix();
}

ITOMPOptimizer::~ITOMPOptimizer()
{
}

void ITOMPOptimizer::addStaticObstalceSphere(double radius, const Eigen::Vector3d& position)
{
    BoundingSphereRobotModel::Sphere sphere;
    sphere.radius = radius;
    sphere.position = position;

    static_obstacle_spheres_.push_back( sphere );
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
        visualizeInterpolationSamples();
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

                cost += (1 - std::abs(link_orientation.dot(target_orientation))) * ratio_cosine_to_meter_ * cost_weights_.goal_pose_cost_weight;
            }
        }
        
        // penalize velocities at last state
        for (int i=0; i<num_joints_; i++)
        {
            const double v = milestones_(num_joints_ + i, num_milestones_ - 1);
            cost += (v * v) * ratio_radian_per_sec_to_meter_ * cost_weights_.goal_pose_cost_weight;
        }
    }
    
    // collision cost (all pairs)
    if (cost_weights_.collision_cost_weight != 0.)
    {
        for (int i=0; i<interpolated_collision_spheres_.size(); i++)
        {
            for (int j=0; j<interpolated_collision_spheres_[i].size(); j++)
            {
                const BoundingSphereRobotModel::Spheres& robot_spheres = interpolated_collision_spheres_[i][j];

                for (int k=0; k<robot_spheres.size(); k++)
                {
                    const BoundingSphereRobotModel::Sphere& robot_sphere = robot_spheres[k];

                    for (int l=0; l<static_obstacle_spheres_.size(); l++)
                    {
                        const BoundingSphereRobotModel::Sphere& obstacle_sphere = static_obstacle_spheres_[l];

                        const double r = robot_sphere.radius + obstacle_sphere.radius;
                        const double d_squared = (robot_sphere.position - obstacle_sphere.position).squaredNorm();

                        if (d_squared < r*r)
                        {
                            cost += (r*r - d_squared) * cost_weights_.collision_cost_weight / num_interpolation_samples_;
                        }
                    }
                }
            }
        }
    }

    return cost;
}

const ITOMPOptimizer::column_vector ITOMPOptimizer::optimizationCostDerivative(const column_vector& variables)
{
    milestoneInitializeWithDlibVector(variables);
    precomputeOptimizationResources();

    Eigen::MatrixXd derivative(milestones_.rows(), milestones_.cols());
    derivative.setZero();

    // smoothness cost
    if (cost_weights_.smoothness_cost_weight != 0.)
    {
        for (int i=0; i<num_milestones_; i++)
        {
            const Eigen::VectorXd& variables0 = i==0 ? start_milestone_ : milestones_.col(i-1);
            const Eigen::VectorXd& variables1 = milestones_.col(i);
            const double t0 = (double)i / num_milestones_ * trajectory_duration_;
            const double t1 = (double)(i+1) / num_milestones_ * trajectory_duration_;

            for (int j=0; j<num_joints_; j++)
            {
                const double& p0 = variables0(j);
                const double& v0 = variables0(num_joints_ + j);
                const double& p1 = variables1(j);
                const double& v1 = variables1(num_joints_ + j);

                const Eigen::Vector4d v(p0, v0, p1, v1);

                const double dinv = 1. / (t1-t0);
                const Eigen::Vector4d diag(1., t1-t0, 1., t1-t0);
                const Eigen::DiagonalMatrix<double, 4> D(diag);
                Eigen::Vector4d term_derivative = (dinv * dinv * dinv) * D * H2_ * D * v;

                // normalize with trajectory duration
                term_derivative *= cost_weights_.smoothness_cost_weight * trajectory_duration_;

                if (i)
                {
                    derivative(j, i-1) += term_derivative(0);
                    derivative(num_joints_ + j, i-1) += term_derivative(1);
                }
                derivative(j, i) += term_derivative(2);
                derivative(num_joints_ + j, i) += term_derivative(3);
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

                for (int j=0; j<num_joints_; j++)
                {
                    if (joints_affecting_link_transforms_[j][i])
                    {
                        const int joint_index = planning_joint_indices_[j];

                        const Eigen::Affine3d& transform_joint = goal_link_transforms_[joint_index];
                        const Eigen::Vector3d& relative_link_position = transform_joint.inverse() * link_position;
                        const Eigen::Vector3d& relative_target_position = transform_joint.inverse() * target_position;

                        switch (robot_model_->getJointType(joint_index))
                        {
                        case RobotModel::REVOLUTE:
                        {
                            const Eigen::Vector3d axis = robot_model_->getJointAxis(joint_index).normalized();
                            // derivative = 2 (axis cross link) dot (link - target)
                            derivative(j, num_milestones_ - 1) += 2. * (axis.cross(relative_link_position)).dot(relative_link_position - relative_target_position) * position_weight * cost_weights_.goal_pose_cost_weight;

                            break;
                        }

                        default:
                            ROS_ERROR("Unsupported joint type [%s] for goal pose cost computation", robot_model_->getJointType(joint_index));
                        }
                    }
                }
            }

            if (orientation_weight != 0.)
            {
                const Eigen::Quaterniond link_orientation( goal_link_transforms_[i].linear() );
                const Eigen::Quaterniond& target_orientation( goal_link_poses_[i].orientation );

                for (int j=0; j<num_joints_; j++)
                {
                    if (joints_affecting_link_transforms_[j][i])
                    {
                        const int joint_index = planning_joint_indices_[j];

                        const Eigen::Matrix3d& rotation_joint = goal_link_transforms_[joint_index].linear();
                        const Eigen::Quaterniond relative_link_orientation( rotation_joint.inverse() * link_orientation.toRotationMatrix() );
                        Eigen::Quaterniond relative_target_orientation( rotation_joint.inverse() * target_orientation.toRotationMatrix() );

                        if (relative_link_orientation.dot(relative_target_orientation) < 0)
                            relative_target_orientation = Eigen::Quaterniond(
                                        -relative_target_orientation.w(),
                                        -relative_target_orientation.x(),
                                        -relative_target_orientation.y(),
                                        -relative_target_orientation.z()
                                        );

                        switch (robot_model_->getJointType(joint_index))
                        {
                        case RobotModel::REVOLUTE:
                        {
                            const Eigen::Vector3d axis = robot_model_->getJointAxis(joint_index).normalized();
                            const Eigen::Quaterniond q_prime(0., 0.5 * axis(0), 0.5 * axis(1), 0.5 * axis(2));
                            // derivative = - ((0, 0.5 axis) times link) dot target
                            derivative(j, num_milestones_ - 1) += ratio_cosine_to_meter_ * (- (q_prime * relative_link_orientation).dot(relative_target_orientation)) * orientation_weight * cost_weights_.goal_pose_cost_weight;

                            break;
                        }

                        default:
                            ROS_ERROR("Unsupported joint type [%d] for goal pose cost computation", robot_model_->getJointType(joint_index));
                        }
                    }
                }
            }
        }

        // penalize velocities at last state
        for (int i=0; i<num_joints_; i++)
        {
            const double v = milestones_(num_joints_ + i, num_milestones_ - 1);
            derivative(num_joints_ + i, num_milestones_ - 1) += 2. * v * ratio_radian_per_sec_to_meter_ * cost_weights_.goal_pose_cost_weight;
        }
    }

    // collision cost (all pairs)
    if (cost_weights_.collision_cost_weight != 0.)
    {
        for (int i=0; i<interpolated_collision_spheres_.size(); i++)
        {
            for (int j=0; j<interpolated_collision_spheres_[i].size(); j++)
            {
                const BoundingSphereRobotModel::Spheres& robot_spheres = interpolated_collision_spheres_[i][j];

                for (int k=0; k<robot_spheres.size(); k++)
                {
                    const BoundingSphereRobotModel::Sphere& robot_sphere = robot_spheres[k];

                    for (int l=0; l<static_obstacle_spheres_.size(); l++)
                    {
                        const BoundingSphereRobotModel::Sphere& obstacle_sphere = static_obstacle_spheres_[l];

                        const double r = robot_sphere.radius + obstacle_sphere.radius;
                        const double d_squared = (robot_sphere.position - obstacle_sphere.position).squaredNorm();

                        if (d_squared < r*r)
                        {
                            //cost += (r*r - d_squared) * cost_weights_.collision_cost_weight / num_interpolation_samples_;

                            for (int m=0; m<num_joints_; m++)
                            {
                                if (joints_affecting_link_transforms_[m][j])
                                {
                                    const int joint_index = planning_joint_indices_[m];

                                    const Eigen::Affine3d& transform_joint = interpolated_variable_link_transforms_[i][joint_index];
                                    const Eigen::Vector3d& relative_robot_position = transform_joint.inverse() * robot_sphere.position;
                                    const Eigen::Vector3d& relative_obstacle_position = transform_joint.inverse() * obstacle_sphere.position;

                                    switch (robot_model_->getJointType(joint_index))
                                    {
                                    case RobotModel::REVOLUTE:
                                    {
                                        const Eigen::Vector3d axis = robot_model_->getJointAxis(joint_index).normalized();
                                        // derivative = 2 (axis cross link) dot (link - target)
                                        const double curve_derivative = - 2. * (axis.cross(relative_robot_position)).dot(relative_robot_position - relative_obstacle_position);

                                        const int milestone_index0 = interpolation_index_position_[i].first - 1;
                                        const int milestone_index1 = interpolation_index_position_[i].first;
                                        const int interpolation_index = interpolation_index_position_[i].second;
                                        const Eigen::Vector4d variable_derivative = interpolated_curve_bases_.row(interpolation_index) * curve_derivative * cost_weights_.collision_cost_weight / num_interpolation_samples_;

                                        if (milestone_index0 != -1)
                                        {
                                            derivative(m, milestone_index0) += variable_derivative(0);
                                            derivative(m, milestone_index0) += variable_derivative(1);
                                        }
                                        derivative(m, milestone_index1) += variable_derivative(2);
                                        derivative(m, milestone_index1) += variable_derivative(3);

                                        break;
                                    }

                                    default:
                                        ROS_ERROR("Unsupported joint type [%d] for goal pose cost computation", robot_model_->getJointType(joint_index));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return convertEigenToDlibVector( derivative );
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
    
    const int num_interpolated_variables = num_milestones_ * (num_interpolation_samples_ + 1) + 1;
    interpolated_variables_.resize(num_joints_ * 2, num_interpolated_variables);
    interpolated_variable_link_transforms_.resize(num_interpolated_variables);

    interpolated_collision_spheres_.resize(num_interpolated_variables);

    interpolation_index_position_.resize(num_interpolated_variables);

    interpolated_curve_bases_.resize(num_interpolation_samples_ + 1, Eigen::NoChange);
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

void ITOMPOptimizer::precomputeInterpolatedCollisionSpheres()
{
    // interpolated variables
    for (int i=0; i<interpolated_variable_link_transforms_.size(); i++)
        robot_model_->getCollisionSpheres(interpolated_variable_link_transforms_[i], interpolated_collision_spheres_[i]);
}

void ITOMPOptimizer::initializeSmoothnessCostDerivaiveAuxilaryMatrix()
{
    H2_.setZero();
    for (int i=0; i<3; i++)
    {
        const double w = gaussianQuadratureWeight2(i);
        const double t = 0.5 + 0.5 * gaussianQuadratureAbscissa2(i);
        const Eigen::Vector4d h(12.*t - 6., 6.*t - 4., -12.*t + 6., 6.*t - 2.);
        H2_ += w * h * h.transpose();
    }
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
