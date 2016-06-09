#ifndef ITOMP_EXEC_ROBOT_MODEL_H
#define ITOMP_EXEC_ROBOT_MODEL_H


#include <moveit/robot_model/robot_model.h>
#include <Eigen/Dense>
#include <memory>


namespace itomp_exec
{

class RobotModel
{
public:
    
    struct JointLimit
    {
        double min_position;
        double max_position;
        double min_velocity;
        double max_velocity;
    };
    
    enum JointType
    {
        FIXED = 0,
        REVOLUTE,
        PRISMATIC,
    };

public:

    RobotModel();
    ~RobotModel();

    void initFromMoveitRobotModel(robot_model::RobotModelConstPtr robot_model);
    
    inline const std::vector<std::string>& getJointNames() const
    {
        return joint_names_;
    }
    
    inline const std::vector<int>& getPlanningGroupJointIndices(const std::string& planning_group_name) const
    {
        const std::map<std::string, int>::const_iterator it = planning_group_index_map_.find(planning_group_name);
        if (it != planning_group_index_map_.cend())
            return planning_group_joint_indices_[ it->second ];
        return std::vector<int>();
    }

    inline const std::vector<JointLimit>& getJointLimits() const
    {
        return joint_limits_;
    }

    inline const Eigen::VectorXd& getJointDefaultPositions() const
    {
        return joint_default_positions_;
    }

    inline const Eigen::VectorXd& getJointDefaultVelocities() const
    {
        return joint_default_velocities_;
    }
    
    inline int getNumJoints() const
    {
        return num_joints_;
    }
    
    inline int getJointIndexByName(const std::string& joint_name) const
    {
        const std::map<std::string, int>::const_iterator it = joint_index_map_.find(joint_name);
        if (it != joint_index_map_.cend())
            return it->second;
        return -1;
    }
    
    inline int getJointIndexByLinkName(const std::string& link_name) const
    {
        const std::map<std::string, int>::const_iterator it = link_index_map_.find(link_name);
        if (it != link_index_map_.cend())
            return it->second;
        return -1;
    }
    
    /// forward kinematics
    void getLinkTransforms(const Eigen::VectorXd& joint_positions, std::vector<Eigen::Affine3d>& link_transforms) const;

private:
    
    /// returns the joint index
    int initializeJointsRecursive(const robot_model::JointModel* joint_model);
    
    // joint
    int num_joints_;
    std::map<std::string, int> joint_index_map_;
    std::vector<JointType> joint_types_;
    std::vector<JointLimit> joint_limits_;
    std::vector<std::string> joint_names_;
    Eigen::VectorXd joint_default_positions_;
    Eigen::VectorXd joint_default_velocities_;
    Eigen::Matrix3Xd joint_axes_;
    
    // links
    std::map<std::string, int> link_index_map_;
    std::vector<Eigen::Affine3d> link_origin_transforms_;
    std::vector<shapes::Mesh*> link_visual_meshes_;
    std::vector<Eigen::Affine3d> link_visual_transforms_;
    std::vector<std::vector<shapes::ShapeConstPtr> > link_collision_shapes_;
    std::vector<EigenSTL::vector_Affine3d> link_collision_transforms_;
    
    // joint tree structure
    int root_joint_index_;
    std::vector<int> parent_joint_indices_;
    std::vector<std::vector<int> > children_joint_indices_;
    
    // planning group
    std::map<std::string, int> planning_group_index_map_;
    std::vector<std::vector<int> > planning_group_joint_indices_;
};

typedef std::shared_ptr<RobotModel> RobotModelPtr;
typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;

}


#endif // ITOMP_EXEC_ROBOT_MODEL_H
