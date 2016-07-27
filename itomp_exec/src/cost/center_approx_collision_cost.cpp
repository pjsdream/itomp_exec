#include <itomp_exec/cost/center_approx_collision_cost.h>
#include <itomp_exec/optimization/itomp_optimizer.h>


namespace itomp_exec
{

CenterApproxCollisionCost::CenterApproxCollisionCost(ITOMPOptimizer& optimizer, double weight)
    : Cost(optimizer, weight)
{
}

void CenterApproxCollisionCost::addCost()
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

                // dynamic obstacle spheres
                if (planning_timestep + interpolated_time <= dynamic_obstacle_duration)
                {
                    for (int l=0; l<dynamic_obstacle_spheres.size(); l++)
                    {
                        const Sphere& obstacle_sphere = dynamic_obstacle_spheres[l];

                        const double r = robot_sphere.radius + obstacle_sphere.radius;
                        const double d_squared = (robot_sphere.position - obstacle_sphere.position).squaredNorm();

                        // symmetric covariant matrix
                        double sigma = std::pow(obstacle_sphere.radius, 1/.3);

                        const double volume = 4. / 3. * M_PI * r*r*r;
                        const double center_probability = 1. / std::sqrt(8 * M_PI * M_PI * M_PI * sigma.determinant()) * std::exp(- 0.5 * d_squared / sigma);

                        cost += center_probability * volume * trajectory_duration * weight / num_interpolation_samples;
                    }
                }
            }
        }
    }
}

}
