#include <itomp_exec/planner/itomp_planner_node.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    
    ros::init(argc, argv, "move_itomp");
    
    ros::NodeHandle nh("~");
    itomp_exec::ITOMPPlannerNode planner(nh);
    
    planner.printParams();
    
    planning_interface::MotionPlanResponse res;
    planner.planAndExecute(res);
    
    return 0;
}
