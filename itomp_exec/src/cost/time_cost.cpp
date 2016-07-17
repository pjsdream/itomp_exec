#include <itomp_exec/cost/time_cost.h>
#include <itomp_exec/optimization/itomp_optimizer.h>


namespace itomp_exec
{

const double TimeCost::ratio_cosine_to_meter_ = 1.;
const double TimeCost::ratio_radian_per_sec_to_meter_ = 1.;

TimeCost::TimeCost(ITOMPOptimizer& optimizer, double weight)
    : Cost(optimizer, weight)
{
}

void TimeCost::addCost()
{
    ITOMPOptimizer& optimizer = getOptimizer();
    const double weight = getWeight();

    double& cost = optimizer.cost();

    const int num_interpolated_variables = optimizer.getNumInterpolatedConfigurations();
    const int num_robot_joints = optimizer.getNumRobotJoints();
    const int num_interpolation_samples = optimizer.getNumInterpolationSamples();
    const Eigen::MatrixXd& milestones = optimizer.getMilestones();
    const int num_joints = optimizer.getNumJoints();
    const int num_milestones = optimizer.getNumMilestones();

    for (int i=0; i<num_interpolated_variables; i++)
    {
        const double interpolated_time = optimizer.getInterpolatedTime(i);

        for (int j=0; j<num_robot_joints; j++)
        {
            const ITOMPOptimizer::GoalLinkPose& goal_link_pose = optimizer.getGoalLinkPose(j);

            const double& position_weight = goal_link_pose.position_weight;
            const double& orientation_weight = goal_link_pose.orientation_weight;

            if (position_weight != 0.)
            {
                const Eigen::Vector3d& link_position = optimizer.getInterpolatedLinkTransform(i, j).translation();
                const Eigen::Vector3d& target_position = goal_link_pose.position;

                cost += interpolated_time * (link_position - target_position).squaredNorm() * position_weight * weight / num_interpolation_samples;
            }

            if (orientation_weight != 0.)
            {
                const Eigen::Quaterniond link_orientation( optimizer.getInterpolatedLinkTransform(i, j).linear() );
                const Eigen::Quaterniond& target_orientation = goal_link_pose.orientation;

                cost += interpolated_time * (1 - std::abs(link_orientation.dot(target_orientation))) * ratio_cosine_to_meter_ * orientation_weight * weight / num_interpolation_samples;
            }
        }
    }

    // penalize velocities at last state
    for (int i=0; i<num_joints; i++)
    {
        const double v = milestones(num_joints + i, num_milestones - 1);
        cost += (v * v) * ratio_radian_per_sec_to_meter_ * weight;
    }
}

void TimeCost::addDerivative()
{
    ITOMPOptimizer& optimizer = getOptimizer();
    const double weight = getWeight();

    const Eigen::MatrixXd& milestones = optimizer.getMilestones();
    Eigen::MatrixXd& milestone_derivative = optimizer.milestoneDerivative();

    const int num_interpolated_variables = optimizer.getNumInterpolatedConfigurations();
    const int num_joints = optimizer.getNumJoints();
    const int num_robot_joints = optimizer.getNumRobotJoints();
    const int num_interpolation_samples = optimizer.getNumInterpolationSamples();
    const RobotModel& robot_model = optimizer.getRobotModel();
    const int num_milestones = optimizer.getNumMilestones();

    for (int i=0; i<num_interpolated_variables; i++)
    {
        const double interpolated_time = optimizer.getInterpolatedTime(i);

        for (int j=0; j<num_robot_joints; j++)
        {
            const ITOMPOptimizer::GoalLinkPose& goal_link_pose = optimizer.getGoalLinkPose(j);

            const double& position_weight = goal_link_pose.position_weight;
            const double& orientation_weight = goal_link_pose.orientation_weight;

            if (position_weight != 0.)
            {
                const Eigen::Vector3d& link_position = optimizer.getInterpolatedLinkTransform(i, j).translation();
                const Eigen::Vector3d& target_position = goal_link_pose.position;

                for (int k=0; k<num_joints; k++)
                {
                    if (optimizer.doesJointAffectLinkTransform(k, j))
                    {
                        const int joint_index = optimizer.getPlanningJointIndex(k);

                        const Eigen::Affine3d& transform_joint = optimizer.getInterpolatedLinkTransform(i, joint_index);
                        const Eigen::Vector3d& relative_link_position = transform_joint.inverse() * link_position;
                        const Eigen::Vector3d& relative_target_position = transform_joint.inverse() * target_position;

                        switch (robot_model.getJointType(joint_index))
                        {
                        case RobotModel::REVOLUTE:
                        {
                            const Eigen::Vector3d axis = robot_model.getJointAxis(joint_index).normalized();
                            // derivative = 2 (axis cross link) dot (link - target)
                            const double curve_derivative = 2. * (axis.cross(relative_link_position)).dot(relative_link_position - relative_target_position);

                            addDerivativeByInterpolationIndex(k, i, interpolated_time * curve_derivative * position_weight * weight / num_interpolation_samples);

                            break;
                        }

                        default:
                            ROS_ERROR("Unsupported joint type [%s] for time cost computation", robot_model.getJointType(joint_index));
                        }
                    }
                }
            }

            if (orientation_weight != 0.)
            {
                const Eigen::Quaterniond link_orientation( optimizer.getInterpolatedLinkTransform(i, j).linear() );
                const Eigen::Quaterniond& target_orientation = goal_link_pose.orientation;

                for (int k=0; k<num_joints; k++)
                {
                    if (optimizer.doesJointAffectLinkTransform(k, j))
                    {
                        const int joint_index = optimizer.getPlanningJointIndex(k);

                        const Eigen::Matrix3d& rotation_joint = optimizer.getInterpolatedLinkTransform(i, joint_index).linear();
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
                            const double curve_derivative = ratio_cosine_to_meter_ * (- (q_prime * relative_link_orientation).dot(relative_target_orientation));

                            addDerivativeByInterpolationIndex(k, i, interpolated_time * curve_derivative * orientation_weight * weight / num_interpolation_samples);

                            break;
                        }

                        default:
                            ROS_ERROR("Unsupported joint type [%d] for goal pose cost computation", robot_model.getJointType(joint_index));
                        }
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
