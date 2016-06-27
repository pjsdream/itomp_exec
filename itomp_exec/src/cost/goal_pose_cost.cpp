#include <itomp_exec/cost/goal_pose_cost.h>
#include <itomp_exec/optimization/itomp_optimizer.h>
#include <moveit/robot_model/robot_model.h>
#include <ros/console.h>


namespace itomp_exec
{

const double GoalPoseCost::ratio_cosine_to_meter_ = 2.;
const double GoalPoseCost::ratio_radian_per_sec_to_meter_ = 10.;

GoalPoseCost::GoalPoseCost(ITOMPOptimizer& optimizer, double weight)
    : Cost(optimizer, weight)
{
}

void GoalPoseCost::addCost()
{
    ITOMPOptimizer& optimizer = getOptimizer();
    const double weight = getWeight();

    double& cost = optimizer.cost();

    const int num_joints = optimizer.getNumJoints();
    const int num_robot_joints = optimizer.getNumRobotJoints();
    const int num_milestones = optimizer.getNumMilestones();
    const Eigen::MatrixXd& milestones = optimizer.getMilestones();

    for (int i=0; i<num_robot_joints; i++)
    {
        const ITOMPOptimizer::GoalLinkPose& goal_link_pose = optimizer.getGoalLinkPose(i);

        const double& position_weight = goal_link_pose.position_weight;
        const double& orientation_weight = goal_link_pose.orientation_weight;

        if (position_weight != 0.)
        {
            const Eigen::Vector3d& link_position = optimizer.getGoalLinkTransform(i).translation();
            const Eigen::Vector3d& target_position = goal_link_pose.position;

            cost += (link_position - target_position).squaredNorm() * weight;
        }

        if (orientation_weight != 0.)
        {
            const Eigen::Quaterniond link_orientation( optimizer.getGoalLinkTransform(i).linear() );
            const Eigen::Quaterniond target_orientation = goal_link_pose.orientation;

            cost += (1 - std::abs(link_orientation.dot(target_orientation))) * ratio_cosine_to_meter_ * weight;
        }
    }

    // penalize velocities at last state
    for (int i=0; i<num_joints; i++)
    {
        const double v = milestones(num_joints + i, num_milestones - 1);
        cost += (v * v) * ratio_radian_per_sec_to_meter_ * weight;
    }
}

void GoalPoseCost::addDerivative()
{
    ITOMPOptimizer& optimizer = getOptimizer();
    const double weight = getWeight();

    Eigen::MatrixXd& milestone_derivative = optimizer.milestoneDerivative();

    const int num_joints = optimizer.getNumJoints();
    const int num_robot_joints = optimizer.getNumRobotJoints();
    const int num_milestones = optimizer.getNumMilestones();
    const Eigen::MatrixXd& milestones = optimizer.getMilestones();
    const BoundingSphereRobotModel& robot_model = optimizer.getRobotModel();

    for (int i=0; i<num_robot_joints; i++)
    {
        const ITOMPOptimizer::GoalLinkPose& goal_link_pose = optimizer.getGoalLinkPose(i);

        const double& position_weight = goal_link_pose.position_weight;
        const double& orientation_weight = goal_link_pose.orientation_weight;

        if (position_weight != 0.)
        {
            const Eigen::Affine3d& goal_link_transform = optimizer.getGoalLinkTransform(i);

            const Eigen::Vector3d& link_position = goal_link_transform.translation();
            const Eigen::Vector3d& target_position = goal_link_pose.position;

            for (int j=0; j<num_joints; j++)
            {
                if (optimizer.doesJointAffectLinkTransform(j, i))
                {
                    const int joint_index = optimizer.getPlanningJointIndex(j);

                    const Eigen::Affine3d& transform_joint = optimizer.getGoalLinkTransform(joint_index);
                    const Eigen::Vector3d& relative_link_position = transform_joint.inverse() * link_position;
                    const Eigen::Vector3d& relative_target_position = transform_joint.inverse() * target_position;

                    switch (robot_model.getJointType(joint_index))
                    {
                    case RobotModel::REVOLUTE:
                    {
                        const Eigen::Vector3d axis = robot_model.getJointAxis(joint_index).normalized();
                        // derivative = 2 (axis cross link) dot (link - target)
                        milestone_derivative(j, num_milestones - 1) += 2. * (axis.cross(relative_link_position)).dot(relative_link_position - relative_target_position) * position_weight * weight;

                        break;
                    }

                    default:
                        ROS_ERROR("Unsupported joint type [%s] for goal pose cost computation", robot_model.getJointType(joint_index));
                    }
                }
            }
        }

        if (orientation_weight != 0.)
        {
            const Eigen::Affine3d& goal_link_transform = optimizer.getGoalLinkTransform(i);

            const Eigen::Quaterniond link_orientation( goal_link_transform.linear() );
            const Eigen::Quaterniond& target_orientation( goal_link_pose.orientation );

            for (int j=0; j<num_joints; j++)
            {
                if (optimizer.doesJointAffectLinkTransform(j, i))
                {
                    const int joint_index = optimizer.getPlanningJointIndex(j);

                    const Eigen::Matrix3d& rotation_joint = optimizer.getGoalLinkTransform(joint_index).linear();
                    const Eigen::Quaterniond relative_link_orientation( rotation_joint.inverse() * link_orientation.toRotationMatrix() );
                    Eigen::Quaterniond relative_target_orientation( rotation_joint.inverse() * target_orientation.toRotationMatrix() );

                    if (relative_link_orientation.dot(relative_target_orientation) < 0)
                        relative_target_orientation = Eigen::Quaterniond(
                                    -relative_target_orientation.w(),
                                    -relative_target_orientation.x(),
                                    -relative_target_orientation.y(),
                                    -relative_target_orientation.z()
                                    );

                    switch (robot_model.getJointType(joint_index))
                    {
                    case RobotModel::REVOLUTE:
                    {
                        const Eigen::Vector3d axis = robot_model.getJointAxis(joint_index).normalized();
                        const Eigen::Quaterniond q_prime(0., 0.5 * axis(0), 0.5 * axis(1), 0.5 * axis(2));
                        // derivative = - ((0, 0.5 axis) times link) dot target
                        milestone_derivative(j, num_milestones - 1) += ratio_cosine_to_meter_ * (- (q_prime * relative_link_orientation).dot(relative_target_orientation)) * orientation_weight * weight;

                        break;
                    }

                    default:
                        ROS_ERROR("Unsupported joint type [%d] for goal pose cost computation", robot_model.getJointType(joint_index));
                    }
                }
            }
        }
    }

    // penalize velocities at last state
    for (int i=0; i<num_joints; i++)
    {
        const double v = milestones(num_joints + i, num_milestones - 1);
        milestone_derivative(num_joints + i, num_milestones - 1) += 2. * v * ratio_radian_per_sec_to_meter_ * weight;
    }
}

}
