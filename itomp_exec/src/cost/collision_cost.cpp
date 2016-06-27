#include <itomp_exec/cost/collision_cost.h>
#include <itomp_exec/optimization/itomp_optimizer.h>


namespace itomp_exec
{

CollisionCost::CollisionCost(ITOMPOptimizer& optimizer, double weight)
    : Cost(optimizer, weight)
{
}

void CollisionCost::addCost()
{
    ITOMPOptimizer& optimizer = getOptimizer();
    const double weight = getWeight();

    double& cost = optimizer.cost();

    const int num_interpolated_variables = optimizer.getNumInterpolatedConfigurations();
    const int num_robot_joints = optimizer.getNumRobotJoints();
    const int num_interpolation_samples = optimizer.getNumInterpolationSamples();
    const double dynamic_obstacle_duration = optimizer.getDynamicObstacleDuration();
    const double planning_timestep = optimizer.getPlanningTimestep();
    const double trajectory_duration = optimizer.getTrajectoryDuration();
    const double dynamic_obstacle_max_speed = optimizer.getDynamicObstacleMaxSpeed();
    const Spheres& static_obstacle_spheres = optimizer.getStaticObstacleSpheres();
    const Spheres& dynamic_obstacle_spheres = optimizer.getDynamicObstacleSpheres();

    for (int i=0; i<num_interpolated_variables; i++)
    {
        const double interpolated_time = optimizer.getInterpolatedTime(i);

        for (int j=0; j<num_robot_joints; j++)
        {
            const Spheres& robot_spheres = optimizer.getInterpolatedLinkCollisionSpheres(i, j);

            for (int k=0; k<robot_spheres.size(); k++)
            {
                const Sphere& robot_sphere = robot_spheres[k];

                // static obstacle spheres
                for (int l=0; l<static_obstacle_spheres.size(); l++)
                {
                    const Sphere& obstacle_sphere = static_obstacle_spheres[l];

                    const double r = robot_sphere.radius + obstacle_sphere.radius;
                    const double d_squared = (robot_sphere.position - obstacle_sphere.position).squaredNorm();

                    if (d_squared < r*r)
                    {
                        cost += (r*r - d_squared) * trajectory_duration * weight / num_interpolation_samples;
                    }
                }

                // dynamic obstacle spheres
                if (planning_timestep + interpolated_time <= dynamic_obstacle_duration)
                {
                    for (int l=0; l<dynamic_obstacle_spheres.size(); l++)
                    {
                        const Sphere& obstacle_sphere = dynamic_obstacle_spheres[l];

                        const double r = robot_sphere.radius + (obstacle_sphere.radius + dynamic_obstacle_max_speed * (planning_timestep + interpolated_time));
                        const double d_squared = (robot_sphere.position - obstacle_sphere.position).squaredNorm();

                        if (d_squared < r*r)
                        {
                            cost += (r*r - d_squared) * trajectory_duration * weight / num_interpolation_samples;
                        }
                    }
                }
            }
        }
    }
}

void CollisionCost::addDerivative()
{
    ITOMPOptimizer& optimizer = getOptimizer();
    const double weight = getWeight();

    Eigen::MatrixXd& milestone_derivative = optimizer.milestoneDerivative();

    const int num_joints = optimizer.getNumJoints();
    const int num_interpolated_variables = optimizer.getNumInterpolatedConfigurations();
    const int num_robot_joints = optimizer.getNumRobotJoints();
    const int num_interpolation_samples = optimizer.getNumInterpolationSamples();
    const double dynamic_obstacle_duration = optimizer.getDynamicObstacleDuration();
    const double planning_timestep = optimizer.getPlanningTimestep();
    const double trajectory_duration = optimizer.getTrajectoryDuration();
    const double dynamic_obstacle_max_speed = optimizer.getDynamicObstacleMaxSpeed();
    const Spheres& static_obstacle_spheres = optimizer.getStaticObstacleSpheres();
    const Spheres& dynamic_obstacle_spheres = optimizer.getDynamicObstacleSpheres();
    const BoundingSphereRobotModel& robot_model = optimizer.getRobotModel();

    for (int i=0; i<num_interpolated_variables; i++)
    {
        const double interpolated_time = optimizer.getInterpolatedTime(i);

        for (int j=0; j<num_robot_joints; j++)
        {
            const Spheres& robot_spheres = optimizer.getInterpolatedLinkCollisionSpheres(i, j);

            for (int k=0; k<robot_spheres.size(); k++)
            {
                const Sphere& robot_sphere = robot_spheres[k];

                // static obstacle spheres
                for (int l=0; l<static_obstacle_spheres.size(); l++)
                {
                    const Sphere& obstacle_sphere = static_obstacle_spheres[l];

                    const double r = robot_sphere.radius + obstacle_sphere.radius;
                    const double d_squared = (robot_sphere.position - obstacle_sphere.position).squaredNorm();

                    if (d_squared < r*r)
                    {
                        for (int m=0; m<num_joints; m++)
                        {
                            if (optimizer.doesJointAffectLinkTransform(m, j))
                            {
                                const int joint_index = optimizer.getPlanningJointIndex(m);

                                const Eigen::Affine3d& transform_joint = optimizer.getInterpolatedLinkTransform(i, joint_index);
                                const Eigen::Vector3d& relative_robot_position = transform_joint.inverse() * robot_sphere.position;
                                const Eigen::Vector3d& relative_obstacle_position = transform_joint.inverse() * obstacle_sphere.position;

                                switch (robot_model.getJointType(joint_index))
                                {
                                case RobotModel::REVOLUTE:
                                {
                                    const Eigen::Vector3d axis = robot_model.getJointAxis(joint_index).normalized();
                                    const double curve_derivative = - 2. * (axis.cross(relative_robot_position)).dot(relative_robot_position - relative_obstacle_position);

                                    const std::pair<int, int> interpolation_index_position = optimizer.getInterpolationIndexPosition(i);
                                    const int milestone_index0 = interpolation_index_position.first - 1;
                                    const int milestone_index1 = milestone_index0 + 1;
                                    const int interpolation_index = interpolation_index_position.second;
                                    const Eigen::Vector4d variable_derivative = optimizer.getInterpolatedCurveBasis(interpolation_index) * curve_derivative * trajectory_duration * weight / num_interpolation_samples;

                                    if (milestone_index0 != -1)
                                    {
                                        milestone_derivative(m, milestone_index0) += variable_derivative(0);
                                        milestone_derivative(m, milestone_index0) += variable_derivative(1);
                                    }
                                    milestone_derivative(m, milestone_index1) += variable_derivative(2);
                                    milestone_derivative(m, milestone_index1) += variable_derivative(3);

                                    break;
                                }

                                default:
                                    ROS_ERROR("Unsupported joint type [%d] for goal pose cost computation", robot_model.getJointType(joint_index));
                                }
                            }
                        }
                    }
                }

                // dynamic obstacle spheres
                if (planning_timestep + interpolated_time <= dynamic_obstacle_duration)
                {
                    for (int l=0; l<dynamic_obstacle_spheres.size(); l++)
                    {
                        const Sphere& obstacle_sphere = dynamic_obstacle_spheres[l];

                        const double r = robot_sphere.radius + (obstacle_sphere.radius + dynamic_obstacle_max_speed * (planning_timestep + interpolated_time));
                        const double d_squared = (robot_sphere.position - obstacle_sphere.position).squaredNorm();

                        if (d_squared < r*r)
                        {
                            for (int m=0; m<num_joints; m++)
                            {
                                if (optimizer.doesJointAffectLinkTransform(m, j))
                                {
                                    const int joint_index = optimizer.getPlanningJointIndex(m);

                                    const Eigen::Affine3d& transform_joint = optimizer.getInterpolatedLinkTransform(i, joint_index);
                                    const Eigen::Vector3d& relative_robot_position = transform_joint.inverse() * robot_sphere.position;
                                    const Eigen::Vector3d& relative_obstacle_position = transform_joint.inverse() * obstacle_sphere.position;

                                    switch (robot_model.getJointType(joint_index))
                                    {
                                    case RobotModel::REVOLUTE:
                                    {
                                        const Eigen::Vector3d axis = robot_model.getJointAxis(joint_index).normalized();
                                        const double curve_derivative = - 2. * (axis.cross(relative_robot_position)).dot(relative_robot_position - relative_obstacle_position);

                                        const std::pair<int, int> interpolation_index_position = optimizer.getInterpolationIndexPosition(i);
                                        const int milestone_index0 = interpolation_index_position.first - 1;
                                        const int milestone_index1 = milestone_index0 + 1;
                                        const int interpolation_index = interpolation_index_position.second;
                                        const Eigen::Vector4d variable_derivative = optimizer.getInterpolatedCurveBasis(interpolation_index) * curve_derivative * trajectory_duration * weight / num_interpolation_samples;

                                        if (milestone_index0 != -1)
                                        {
                                            milestone_derivative(m, milestone_index0) += variable_derivative(0);
                                            milestone_derivative(m, milestone_index0) += variable_derivative(1);
                                        }
                                        milestone_derivative(m, milestone_index1) += variable_derivative(2);
                                        milestone_derivative(m, milestone_index1) += variable_derivative(3);

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
            }
        }
    }
}

}
