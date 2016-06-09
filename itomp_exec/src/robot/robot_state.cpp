#include <itomp_exec/robot/robot_state.h>


namespace itomp_exec
{

RobotState::RobotState()
{
}

void RobotState::RobotState::setPlanningGroup(const std::string& planning_group_name)
{
    planning_group_name_ = planning_group_name;
    
    planning_joint_indices_ = robot_model_->getPlanningGroupJointIndices(planning_group_name);
    
    const int num_planning_joints = planning_joint_indices_.size();
    
    // planning joint names
    std::vector<std::string> joint_names = robot_model_->getJointNames();
    planning_joint_names_.resize(num_planning_joints);
    for (int i=0; i<num_planning_joints; i++)
        planning_joint_names_[i] = joint_names[ planning_joint_indices_[i] ];
    
    // variable limits
    const std::vector<RobotModel::JointLimit>& joint_limits = robot_model_->getJointLimits();
    
    planning_joint_position_lower_limits_.resize(num_planning_joints);
    planning_joint_position_upper_limits_.resize(num_planning_joints);
    planning_joint_velocity_lower_limits_.resize(num_planning_joints);
    planning_joint_velocity_upper_limits_.resize(num_planning_joints);
    
    for (int i=0; i<num_planning_joints; i++)
    {
        planning_joint_position_lower_limits_(i) = joint_limits[ planning_joint_indices_[i] ].min_position;
        planning_joint_position_upper_limits_(i) = joint_limits[ planning_joint_indices_[i] ].max_position;
        planning_joint_velocity_lower_limits_(i) = joint_limits[ planning_joint_indices_[i] ].min_velocity;
        planning_joint_velocity_upper_limits_(i) = joint_limits[ planning_joint_indices_[i] ].max_velocity;
    }
    
    // mapping from all joint indices to joint indices in planning group
    joint_index_to_planning_joint_index_.resize( robot_model_->getNumJoints(), -1 );
    for (int i=0; i<num_planning_joints; i++)
        joint_index_to_planning_joint_index_[ planning_joint_indices_[i] ] = i;
    
    // default values of all joints
    default_joint_positions_ = robot_model_->getJointDefaultPositions();
    default_joint_velocities_ = robot_model_->getJointDefaultVelocities();
    
    // joint values of joints in planning group
    planning_joint_positions_.resize(num_planning_joints);
    planning_joint_velocities_.resize(num_planning_joints);
    for (int i=0; i<num_planning_joints; i++)
    {
        planning_joint_positions_(i) = default_joint_positions_( planning_joint_indices_[i] );
        planning_joint_velocities_(i) = default_joint_velocities_( planning_joint_indices_[i] );
    }
}

void RobotState::initFromMoveitRobotStateMsg(const moveit_msgs::RobotState& msg)
{
    const std::vector<std::string>& joint_names = msg.joint_state.name;
    const std::vector<double>& positions = msg.joint_state.position;
    const std::vector<double>& velocities = msg.joint_state.velocity;
    
    for (int i=0; i<joint_names.size(); i++)
    {
        const int joint_index = robot_model_->getJointIndexByName(joint_names[i]);
        
        // joints not in planning group
        if (joint_index_to_planning_joint_index_[joint_index] == -1)
        {
            if (!positions.empty())
                default_joint_positions_(joint_index) = positions[i];
            if (!velocities.empty())
                default_joint_velocities_(joint_index) = velocities[i];
        }
        
        // joints in planning group
        else
        {
            if (!positions.empty())
                planning_joint_positions_( joint_index_to_planning_joint_index_[joint_index] ) = positions[i];
            if (!velocities.empty())
                planning_joint_velocities_( joint_index_to_planning_joint_index_[joint_index] ) = velocities[i];
        }
    }
}

}
