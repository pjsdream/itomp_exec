#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>
#include <class_loader/class_loader.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

namespace itomp_exec_planner
{

class ITOMPPlannerManager : public planning_interface::PlannerManager
{
public:

	ITOMPPlannerManager() :
			planning_interface::PlannerManager()
	{
	}

	virtual bool initialize(const robot_model::RobotModelConstPtr& model, const std::string &ns)
	{
        return true;
	}

	virtual bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
	{
		return true;
	}

	virtual std::string getDescription() const
	{
		return "ITOMP";
	}

	virtual void getPlanningAlgorithms(std::vector<std::string> &algs) const
	{
		algs.push_back("ITOMP");
        algs.push_back("ITOMP_replanning");
	}

	virtual void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pconfig)
	{
		PlannerManager::setPlannerConfigurations(pconfig);
	}

	virtual planning_interface::PlanningContextPtr getPlanningContext(
            const planning_scene::PlanningSceneConstPtr& planning_scene,
			const planning_interface::MotionPlanRequest &req, moveit_msgs::MoveItErrorCodes &error_code) const
	{
		context_->setPlanningScene(planning_scene);

		return context_;
	}

private:
	planning_interface::PlanningContextPtr context_;
};

}

CLASS_LOADER_REGISTER_CLASS(itomp_exec_planner::ITOMPPlannerManager, planning_interface::PlannerManager)
