#include <itomp_exec/cost/goal_pose_cost.h>
#include <itomp_exec/planner/itomp_planner_node.h>
#include <moveit/robot_model/robot_model.h>
#include <ros/console.h>


namespace itomp_exec
{

const double GoalPoseCost::ratio_cosine_to_meter_ = 1.0;
const double GoalPoseCost::ratio_radian_per_sec_to_meter_ = 1.0;

GoalPoseCost::GoalPoseCost(double weight)
    : Cost(weight)
{
}

void GoalPoseCost::initialize(const ITOMPPlannerNode& planner_node)
{
    goal_link_positions_ = planner_node.getGoalLinkPositions();
    goal_link_orientations_ = planner_node.getGoalLinkOrientations();
    
    // robot state initialize
    robot_model_ = planner_node.getRobotModel();
    const Trajectory& trajectory_template = planner_node.getTrajectoryTemplate();
    
    robot_state_.reset(new robot_state::RobotState(robot_model_));
    trajectory_template.setRobotStateWithStartState(*robot_state_);
    
    num_planning_group_joints_ = trajectory_template.getNumPlanningGroupJoints();
    planning_group_joint_names_ = trajectory_template.getPlanningGroupJointNames();
    planning_group_joint_models_ = trajectory_template.getPlanningGroupJointModels();
    
    // find joints that affect the link position and orientation
    initializeJointsAffectingGoalLinkPoses(trajectory_template, joints_affecting_goal_link_positions_, goal_link_positions_);
    initializeJointsAffectingGoalLinkPoses(trajectory_template, joints_affecting_goal_link_orientations_, goal_link_orientations_);
}

template<typename T>
void GoalPoseCost::initializeJointsAffectingGoalLinkPoses(const Trajectory& trajectory_template, std::vector<std::vector<char> >& joints_affecting_goal_link_poses, const T& goal_links)
{    
    joints_affecting_goal_link_poses.resize(goal_links.size());
    for (int i=0; i<goal_links.size(); i++)
    {
        joints_affecting_goal_link_poses[i].resize(num_planning_group_joints_, false);
        
        const std::string name = goal_links[i].first;
        const robot_model::LinkModel* link = robot_model_->getLinkModel(name);
        
        const robot_model::JointModel* joint = link->getParentJointModel();
        while (joint != NULL)
        {
            const std::string& joint_name = joint->getName();
            const int trajectory_joint_index = trajectory_template.getJointIndexInPlanningGroup(joint_name);
            
            if (trajectory_joint_index != -1)
                joints_affecting_goal_link_poses[i][trajectory_joint_index] = true;
            
            link = joint->getParentLinkModel();
            if (link == NULL)
                break;
            
            joint = link->getParentJointModel();
        }
    }
}

double GoalPoseCost::cost(const Trajectory& trajectory)
{
    double cost = 0.;
    
    trajectory.setMilestoneVariablesPositionsToRobotState(*robot_state_, trajectory.getNumMilestones() - 1);
    robot_state_->updateLinkTransforms();
    
    // translation-1.0
    for (int i=0; i<goal_link_positions_.size(); i++)
    {
        const Eigen::Vector3d& link_position = robot_state_->getGlobalLinkTransform(goal_link_positions_[i].first).translation();
        const Eigen::Vector3d& target_position = goal_link_positions_[i].second;
        
        cost += (link_position - target_position).squaredNorm();
    }
    
    // orientation
    for (int i=0; i<goal_link_orientations_.size(); i++)
    {
        const Eigen::Quaterniond link_orientation( robot_state_->getGlobalLinkTransform(goal_link_orientations_[i].first).linear() );
        Eigen::Quaterniond target_orientation = goal_link_orientations_[i].second;
        if (target_orientation.dot(link_orientation) < 0)
            target_orientation = Eigen::Quaterniond(
                        -target_orientation.w(),
                        -target_orientation.x(),
                        -target_orientation.y(),
                        -target_orientation.z()
                        );
        
        cost += ratio_cosine_to_meter_ * (1 - std::abs(link_orientation.dot(target_orientation)));
    }
    
    // penalize velocities at last state
    const Eigen::VectorXd& last_milestone_variables = trajectory.getMilestoneVariables( trajectory.getNumMilestones() - 1 );
    for (int i=0; i<num_planning_group_joints_; i++)
    {
        const double& v = last_milestone_variables(2*i+1);
        cost += ratio_radian_per_sec_to_meter_ * (v*v);
    }
    
    return cost * weight_;
}

TrajectoryDerivative GoalPoseCost::derivative(const Trajectory& trajectory)
{
    TrajectoryDerivative derivative(trajectory);
    
    const int num_milestones = trajectory.getNumMilestones();
    
    trajectory.setMilestoneVariablesPositionsToRobotState(*robot_state_, num_milestones - 1);
    robot_state_->update();
    
    // translation
    for (int i=0; i<goal_link_positions_.size(); i++)
    {
        const Eigen::Vector3d& link_position = robot_state_->getGlobalLinkTransform(goal_link_positions_[i].first).translation();
        const Eigen::Vector3d& target_position = goal_link_positions_[i].second;
        
        for (int j=0; j<num_planning_group_joints_; j++)
        {
            if (joints_affecting_goal_link_positions_[i][j])
            {
                const Eigen::Affine3d& transform_joint = robot_state_->getGlobalLinkTransform(planning_group_joint_models_[j]->getChildLinkModel());
                const Eigen::Vector3d& relative_link_position = transform_joint.inverse() * link_position;
                const Eigen::Vector3d& relative_target_position = transform_joint.inverse() * target_position;
                const robot_model::JointModel* joint_model = planning_group_joint_models_[j];
                
                switch (joint_model->getType())
                {
                case robot_model::JointModel::REVOLUTE:
                {
                    const robot_model::RevoluteJointModel* revolute_joint_model = dynamic_cast<const robot_model::RevoluteJointModel*>(joint_model);
                    const Eigen::Vector3d axis = revolute_joint_model->getAxis().normalized();
                    // derivative = 2 (axis cross link) dot (link - target)
                    derivative(2*j, num_milestones - 1) += 2. * (axis.cross(relative_link_position)).dot(relative_link_position - relative_target_position);
                    
                    break;
                }
                    
                default:
                    ROS_ERROR("Unsupported joint type [%s] for goal pose cost computation", joint_model->getTypeName().c_str());
                }
            }
        }
    }
    
    // orientation
    for (int i=0; i<goal_link_orientations_.size(); i++)
    {
        const Eigen::Quaterniond link_orientation( robot_state_->getGlobalLinkTransform(goal_link_orientations_[i].first).linear() );
        const Eigen::Quaterniond& target_orientation = goal_link_orientations_[i].second;
        
        for (int j=0; j<num_planning_group_joints_; j++)
        {
            if (joints_affecting_goal_link_orientations_[i][j])
            {
                const Eigen::Matrix3d& rotation_joint = robot_state_->getGlobalLinkTransform(planning_group_joint_models_[j]->getChildLinkModel()).linear();
                const Eigen::Quaterniond relative_link_orientation( rotation_joint.inverse() * link_orientation.toRotationMatrix() );
                Eigen::Quaterniond relative_target_orientation( rotation_joint.inverse() * target_orientation.toRotationMatrix() );
                const robot_model::JointModel* joint_model = planning_group_joint_models_[j];
                
                if (relative_link_orientation.dot(relative_target_orientation) < 0)
                    relative_target_orientation = Eigen::Quaterniond(
                                -relative_target_orientation.w(),
                                -relative_target_orientation.x(),
                                -relative_target_orientation.y(),
                                -relative_target_orientation.z()
                                );
                
                switch (joint_model->getType())
                {
                case robot_model::JointModel::REVOLUTE:
                {
                    const robot_model::RevoluteJointModel* revolute_joint_model = dynamic_cast<const robot_model::RevoluteJointModel*>(joint_model);
                    const Eigen::Vector3d axis = revolute_joint_model->getAxis().normalized();
                    const Eigen::Quaterniond q_prime(0., 0.5 * axis(0), 0.5 * axis(1), 0.5 * axis(2));
                    // derivative = - ((0, 0.5 axis) times link) dot target
                    derivative(2*j, num_milestones - 1) += ratio_cosine_to_meter_ * (- (q_prime * relative_link_orientation).dot(relative_target_orientation));
                    
                    break;
                }
                    
                default:
                    ROS_ERROR("Unsupported joint type [%s] for goal pose cost computation", joint_model->getTypeName().c_str());
                }
            }
        }
    }
    
    // penalize velocities at last state
    const Eigen::VectorXd& last_milestone_variables = trajectory.getMilestoneVariables( num_milestones - 1 );
    for (int i=0; i<num_planning_group_joints_; i++)
    {
        const double& v = last_milestone_variables(2*i+1);
        derivative(2*i+1, num_milestones - 1) += ratio_radian_per_sec_to_meter_ * (2*v);
    }
    
    return derivative * weight_;
}

}
