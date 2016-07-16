#include <itomp_exec/planner/itomp_planner_node.h>
#include <itomp_exec/robot/bounding_sphere_robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/RobotState.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <control_msgs/GripperCommandGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <itomp_exec/util/joint_state_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <ros/ros.h>

#include <stdlib.h>
#include <time.h>


class ITOMPFetch
{
private:
    
    static const std::string planning_group_;
    static const std::string endeffector_name_;
    static const double endeffector_vertical_moving_distance_;
    static const double gripper_picking_distance_;
    
    struct Pose
    {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
    };

    struct AttachingSphere
    {
        double radius;
        Eigen::Vector3d position;
        std::string attaching_link;
        Eigen::Vector3d attaching_position;
    };
    
    struct Sphere
    {
        Sphere(double radius, const Eigen::Vector3d& position)
            : radius(radius), position(position)
        {
        }

        double radius;
        Eigen::Vector3d position;
    };
    
public:
    
    ITOMPFetch(const ros::NodeHandle& nh = ros::NodeHandle("~"));

    void moveGripper(double gap, bool wait_for_execution = true);
    void openGripper(bool wait_for_execution = true);
    void closeGripper(bool wait_for_execution = true);
    
    void moveTorso(double position, bool wait_for_execution = true);

    void attachSphere(int sphere_index);
    void detachSphere(int sphere_index);
    
    void runScenario();
    void runMovingArmScenario();
    void runMovingArmScenarioOMPL();
    
private:
    
    void loadTablePoses();
    
    void moveEndeffectorVertically(double distance); // using ITOMP, with short duration
    void moveEndeffectorVerticallyIK(double distance, bool wait_for_execution = true); // setFromIK result is very different from initial state
    void moveEndeffectorVerticallyTarget(double distance, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, bool wait_for_execution = true); // using ITOMP, to target distance
    
    void initializeCurrentState(robot_state::RobotState& state);
    void initializeCurrentState(moveit_msgs::RobotState& start_state);
    static void initializeTuckState(moveit_msgs::RobotState& start_state);
    static void initializeDefaultState(moveit_msgs::RobotState& start_state);
    
    ros::NodeHandle nh_;
    ros::Publisher start_state_publisher_;
    ros::Publisher goal_state_publisher_;
    ros::Publisher display_trajectory_publisher_;
    ros::Publisher target_positions_publisher_;
    itomp_exec::ITOMPPlannerNode planner_;
    
    // gripper actionlib client
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client_;
    
    // torso actionlib client
    // rostopic pub -1 /torso_controller/follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal '{header:{}, goal_id:{}, goal:{trajectory : {joint_names: [torso_lift_joint], points: [{positions : [0.35], time_from_start: [5.0, 0.0]}]}}}'
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_client_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_client_;
    
    std::vector<Pose> start_poses_;
    std::vector<Pose> target_poses_;

    // attaching spheres
    std::vector<AttachingSphere> attaching_spheres_;
    
    robot_model::RobotModelConstPtr moveit_robot_model_;
    itomp_exec::BoundingSphereRobotModelPtr robot_model_;

    // joint state listener
    itomp_exec::JointStateListener joint_state_listener_;
};



const std::string ITOMPFetch::planning_group_ = "arm";
const std::string ITOMPFetch::endeffector_name_ = "gripper_link";
const double ITOMPFetch::endeffector_vertical_moving_distance_ = 0.10;
const double ITOMPFetch::gripper_picking_distance_ = 0.060;



ITOMPFetch::ITOMPFetch(const ros::NodeHandle& nh)
    : nh_(nh)
    , planner_(nh)
    , gripper_client_("gripper_controller/gripper_action")
    , torso_client_("torso_controller/follow_joint_trajectory")
    , arm_client_("arm_controller/follow_joint_trajectory")
{
    ROS_INFO("waiting for arm controller server");
    arm_client_.waitForServer();

    // initialize planner
    planner_.printParams();
    planner_.printControllers();
    planner_.printCostWeights();
    
    // load robot model
    moveit_robot_model_ = planner_.getMoveitRobotModel();
    robot_model_ = planner_.getRobotModel();

    // publisher initialization
    start_state_publisher_ = nh_.advertise<moveit_msgs::DisplayRobotState>("start_state", 1);
    goal_state_publisher_ = nh_.advertise<moveit_msgs::DisplayRobotState>("goal_state", 1);
    display_trajectory_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("display_planned_path", 1);
    target_positions_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("target_positions", 1);
    ros::Duration(0.5).sleep();
    
    loadTablePoses();
}

void ITOMPFetch::moveGripper(double gap, bool wait_for_execution)
{
    control_msgs::GripperCommandGoal goal;
    goal.command.position = gap;
    goal.command.max_effort = 1000.;
    gripper_client_.sendGoal(goal);
    
    if (wait_for_execution)
        gripper_client_.waitForResult();
}

void ITOMPFetch::closeGripper(bool wait_for_execution)
{
    moveGripper(0.00, wait_for_execution);
}

void ITOMPFetch::openGripper(bool wait_for_execution)
{
    moveGripper(0.10, wait_for_execution);
}

void ITOMPFetch::moveTorso(double position, bool wait_for_execution)
{
    const double time = 1.0;
    
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(position);
    point.time_from_start = ros::Duration(time);
    
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("torso_lift_joint");
    goal.trajectory.points.push_back(point);
    
    torso_client_.sendGoal(goal);
    
    if (wait_for_execution)
    {
        if (!torso_client_.waitForResult())
            ROS_ERROR("Torso action client failed to finish trajectory execution");
    }
}

void ITOMPFetch::initializeTuckState(moveit_msgs::RobotState& start_state)
{
    start_state.joint_state.name.push_back("shoulder_pan_joint");
    start_state.joint_state.name.push_back("shoulder_lift_joint");
    start_state.joint_state.name.push_back("upperarm_roll_joint");
    start_state.joint_state.name.push_back("elbow_flex_joint");
    start_state.joint_state.name.push_back("forearm_roll_joint");
    start_state.joint_state.name.push_back("wrist_flex_joint");
    start_state.joint_state.name.push_back("wrist_roll_joint");
    
    start_state.joint_state.position.push_back(1.32);
    start_state.joint_state.position.push_back(1.40);
    start_state.joint_state.position.push_back(-0.2);
    start_state.joint_state.position.push_back(1.72);
    start_state.joint_state.position.push_back(0.0);
    start_state.joint_state.position.push_back(1.66);
    start_state.joint_state.position.push_back(0.0);
    
    start_state.is_diff = true;
}

void ITOMPFetch::initializeDefaultState(moveit_msgs::RobotState& start_state)
{
    start_state.joint_state.name.push_back("shoulder_pan_joint");
    start_state.joint_state.name.push_back("shoulder_lift_joint");
    start_state.joint_state.name.push_back("upperarm_roll_joint");
    start_state.joint_state.name.push_back("elbow_flex_joint");
    start_state.joint_state.name.push_back("forearm_roll_joint");
    start_state.joint_state.name.push_back("wrist_flex_joint");
    start_state.joint_state.name.push_back("wrist_roll_joint");
    
    start_state.joint_state.position.push_back(0.0);
    start_state.joint_state.position.push_back(0.0);
    start_state.joint_state.position.push_back(0.0);
    start_state.joint_state.position.push_back(0.0);
    start_state.joint_state.position.push_back(0.0);
    start_state.joint_state.position.push_back(0.0);
    start_state.joint_state.position.push_back(0.0);
    
    start_state.is_diff = true;
}

void ITOMPFetch::loadTablePoses()
{
    XmlRpc::XmlRpcValue poses;
    
    if (nh_.getParam("poses", poses))
    {
        for (int i=0; i<poses.size(); i++)
        {
            XmlRpc::XmlRpcValue xml_pose = poses[i];
            
            std::string type_string;
            Eigen::Vector3d position;
            Eigen::Quaterniond orientation;
            
            for (XmlRpc::XmlRpcValue::iterator it = xml_pose.begin(); it != xml_pose.end(); it++)
            {                
                const std::string key = static_cast<std::string>(it->first);
                
                if (key == "type")
                    type_string = static_cast<std::string>(it->second);
                
                else if (key == "position")
                {
                    XmlRpc::XmlRpcValue value = it->second;
                    for (int i=0; i<3; i++)
                        position(i) = static_cast<double>(value[i]);
                }
                
                else if (key == "orientation")
                {
                    XmlRpc::XmlRpcValue value = it->second;
                    orientation.w() = static_cast<double>(value[0]);
                    orientation.x() = static_cast<double>(value[1]);
                    orientation.y() = static_cast<double>(value[2]);
                    orientation.z() = static_cast<double>(value[3]);
                }
            }
            
            Pose pose;
            pose.position = position;
            pose.orientation = orientation;
            
            if (type_string == "start")
            {
                start_poses_.push_back(pose);
            }
            else if (type_string == "target")
            {
                target_poses_.push_back(pose);
            }
        }
    }

    XmlRpc::XmlRpcValue attaching_objects;

    if (nh_.getParam("attachable_objects", attaching_objects))
    {
        for (int i=0; i<attaching_objects.size(); i++)
        {
            XmlRpc::XmlRpcValue object = attaching_objects[i];

            std::string type;
            Eigen::Vector3d position;
            double radius;
            std::string attaching_link;
            Eigen::Vector3d attaching_position;

            type = static_cast<std::string>(object["type"]);

            if (type == "sphere")
            {
                XmlRpc::XmlRpcValue xml_position = object["position"];
                for (int j=0; j<3; j++)
                    position(j) = static_cast<double>(xml_position[j]);

                radius = static_cast<double>(object["radius"]);

                attaching_link = static_cast<std::string>(object["attaching_link"]);

                XmlRpc::XmlRpcValue xml_attaching_position = object["attaching_position"];
                for (int j=0; j<3; j++)
                    attaching_position(j) = static_cast<double>(xml_attaching_position[j]);

                AttachingSphere sphere;
                sphere.radius = radius;
                sphere.position = position;
                sphere.attaching_link = attaching_link;
                sphere.attaching_position = attaching_position;

                attaching_spheres_.push_back(sphere);
            }
        }
    }
    
    // visualize target poses
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    
    std_msgs::ColorRGBA red;
    red.r = 1.;
    red.g = 0.;
    red.b = 0.;
    red.a = 1.;
    
    std_msgs::ColorRGBA blue;
    blue.r = 0.;
    blue.g = 0.;
    blue.b = 1.;
    blue.a = 1.;
    
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.w = 1.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    
    marker.ns = "start_poses";
    for (int i=0; i<start_poses_.size(); i++)
    {
        marker.id = i;
        
        geometry_msgs::Point point;
        tf::pointEigenToMsg(start_poses_[i].position, point);
        marker.points.push_back(point);
        marker.colors.push_back(red);
        
        marker_array.markers.push_back(marker);
    }
    
    marker.ns = "target_poses";
    for (int i=0; i<target_poses_.size(); i++)
    {
        marker.id = i;
        
        geometry_msgs::Point point; 
        tf::pointEigenToMsg(target_poses_[i].position, point);
        marker.points.push_back(point);
        marker.colors.push_back(blue);
        
        marker_array.markers.push_back(marker);
    }
    
    target_positions_publisher_.publish(marker_array);
}

void ITOMPFetch::initializeCurrentState(robot_state::RobotState& state)
{
    // /joint_states topic is publishing one of controller group
    // one is a group of two fingers, and the other one has others
    sensor_msgs::JointState start_joint_state;
    std::set<std::string> joint_set;
    
    const std::vector<const robot_model::JointModel*>& active_joint_models = moveit_robot_model_->getActiveJointModels();
    for (int i=0; i<active_joint_models.size(); i++)
        joint_set.insert( active_joint_models[i]->getName() );
    
    while (!joint_set.empty())
    {
        sensor_msgs::JointState current_joint_state = joint_state_listener_.getJointStates();
        
        for (int i=0; i<current_joint_state.name.size(); i++)
        {
            const std::string joint_name = current_joint_state.name[i];
            if (joint_set.find(joint_name) != joint_set.end())
            {
                joint_set.erase(joint_name);
                
                start_joint_state.header = current_joint_state.header;
                start_joint_state.name.push_back(joint_name);
                
                if (!current_joint_state.position.empty())
                    start_joint_state.position.push_back( current_joint_state.position[i] );
                
                if (!current_joint_state.velocity.empty())
                    start_joint_state.velocity.push_back( current_joint_state.velocity[i] );
                
                if (!current_joint_state.effort.empty())
                    start_joint_state.effort.push_back( current_joint_state.effort[i] );
            }
        }
    }
    
    state.setVariableValues(start_joint_state);
    state.update();
}

void ITOMPFetch::initializeCurrentState(moveit_msgs::RobotState& start_state)
{
    robot_state::RobotState state( moveit_robot_model_ );
    initializeCurrentState(state);
    moveit::core::robotStateToRobotStateMsg(state, start_state, false);
}

void ITOMPFetch::runScenario()
{
    int goal_type = 0;
    int goal_index = 0;
    std::vector<char> goal_achieved(start_poses_.size(), false);
    
    while (true)
    {
        ROS_INFO("Goal index: [%d, %d]", goal_type, goal_index);
        planning_interface::MotionPlanRequest req;
        req.group_name = planning_group_;
        
        // initialize start state with current robot state
        initializeCurrentState(req.start_state);
        
        // initialize start state with current state
        //initializeDefaultState(req.start_state);

        // visualize start state
        moveit_msgs::DisplayRobotState start_state_display_msg;
        start_state_display_msg.state = req.start_state;
        start_state_publisher_.publish(start_state_display_msg);
        
        // goal pose setting
        /*
        moveit_msgs::PositionConstraint goal_position_constraint;
        goal_position_constraint.link_name = endeffector_name_;
        Eigen::Vector3d goal_ee_position(0.5, -0.5, 1.0);
        tf::vectorEigenToMsg(goal_ee_position, goal_position_constraint.target_point_offset);
        
        moveit_msgs::OrientationConstraint goal_orientation_constraint;
        goal_orientation_constraint.link_name = endeffector_name_;
        Eigen::Quaterniond goal_ee_orientation(0.707106781, 0.0, 0.707106781, 0.0);
        tf::quaternionEigenToMsg(goal_ee_orientation, goal_orientation_constraint.orientation);
        
        moveit_msgs::Constraints goal_constraints;
        goal_constraints.position_constraints.push_back(goal_position_constraint);
        goal_constraints.orientation_constraints.push_back(goal_orientation_constraint);
        req.goal_constraints.push_back(goal_constraints);
        
        planner_.setMotionPlanRequest(req);
        */
        moveit_msgs::PositionConstraint goal_position_constraint;
        goal_position_constraint.link_name = endeffector_name_;
        if (goal_type == 0)
            tf::vectorEigenToMsg(start_poses_[goal_index].position, goal_position_constraint.target_point_offset);
        else
            tf::vectorEigenToMsg(target_poses_[goal_index].position, goal_position_constraint.target_point_offset);
        
        moveit_msgs::OrientationConstraint goal_orientation_constraint;
        goal_orientation_constraint.link_name = endeffector_name_;
        if (goal_type == 0)
            tf::quaternionEigenToMsg(start_poses_[goal_index].orientation, goal_orientation_constraint.orientation);
        else
            tf::quaternionEigenToMsg(target_poses_[goal_index].orientation, goal_orientation_constraint.orientation);
        
        moveit_msgs::Constraints goal_constraints;
        goal_constraints.position_constraints.push_back(goal_position_constraint);
        goal_constraints.orientation_constraints.push_back(goal_orientation_constraint);
        req.goal_constraints.push_back(goal_constraints);
        
        planner_.setMotionPlanRequest(req);
        
        // trajectory duration
        double trajectory_duration;
        if (goal_type == 0 && goal_index == 0)
            trajectory_duration = 5.0;
        else
            trajectory_duration = 2.0 + goal_index;

        // plan and execute
        planning_interface::MotionPlanResponse res;
        planner_.setTrajectoryDuration(trajectory_duration);
        planner_.planAndExecute(res);

        // visualize robot trajectory
        moveit_msgs::MotionPlanResponse response_msg;
        res.getMessage(response_msg);

        moveit_msgs::DisplayTrajectory display_trajectory_msg;
        display_trajectory_msg.trajectory_start = response_msg.trajectory_start;
        display_trajectory_msg.trajectory.push_back( response_msg.trajectory );
        display_trajectory_msg.model_id = "model";
        display_trajectory_publisher_.publish(display_trajectory_msg);
        
        // move endeffector vertically using IK to pick or place
        if (goal_type == 0)
        {
            //moveEndeffectorVertically(-endeffector_vertical_moving_distance_);
            moveEndeffectorVerticallyTarget(-endeffector_vertical_moving_distance_, start_poses_[goal_index].position, start_poses_[goal_index].orientation);
            moveGripper(gripper_picking_distance_, false);
            attachSphere(goal_index);
            //moveEndeffectorVertically(endeffector_vertical_moving_distance_);
            moveEndeffectorVerticallyTarget(-endeffector_vertical_moving_distance_ + 0.04, start_poses_[goal_index].position, start_poses_[goal_index].orientation);
        }
        else
        {
            //moveEndeffectorVertically(-endeffector_vertical_moving_distance_);
            openGripper(false);
            detachSphere(goal_index);
            //moveEndeffectorVertically(endeffector_vertical_moving_distance_);
            moveEndeffectorVerticallyTarget(endeffector_vertical_moving_distance_, target_poses_[goal_index].position, target_poses_[goal_index].orientation);
        }
        
        // setup the next goal
        if (goal_type == 0)
        {
            goal_type = 1;
        }
        else
        {
            goal_type = 0;
            
            // goal index random assignment
            /*
            goal_achieved[goal_index] = true;
            
            std::vector<int> indices;
            for (int i=0; i<start_poses_.size(); i++)
            {
                if (!goal_achieved[i])
                    indices.push_back(i);
            }
            
            if (indices.empty())
                break;
            
            goal_index = indices[ rand() % indices.size() ];
            */
            
            // goal index assignment in order
            if (goal_index == start_poses_.size() - 1)
                break;
            goal_index++;
        }
        
        // once
        // break;
    }
}

void ITOMPFetch::runMovingArmScenario()
{
    // open, then close the gripper to grip an object
    openGripper();
    ROS_INFO("Waiting for 1.0 sec for grasping");
    ros::Duration(1.0).sleep();
    moveGripper(gripper_picking_distance_);

    int goal_index = 0;
    Pose start_pose;
    start_pose.position = Eigen::Vector3d(0.5, -0.7, 1.25);
    start_pose.orientation = Eigen::Quaterniond(1., 0., 0., 0.);
    Pose target_pose;
    target_pose.position = Eigen::Vector3d(0.5, 0.7, 1.25);
    target_pose.orientation = Eigen::Quaterniond(1., 0., 0., 0.);
    
    while (true)
    {
        ROS_INFO("Goal index: [%d]", goal_index);
        planning_interface::MotionPlanRequest req;
        req.group_name = planning_group_;
        
        // initialize start state with current robot state
        initializeCurrentState(req.start_state);
        
        // initialize start state with current state
        //initializeDefaultState(req.start_state);

        // visualize start state
        moveit_msgs::DisplayRobotState start_state_display_msg;
        start_state_display_msg.state = req.start_state;
        start_state_publisher_.publish(start_state_display_msg);
        
        // goal pose setting
        moveit_msgs::PositionConstraint goal_position_constraint;
        goal_position_constraint.link_name = endeffector_name_;
        if (goal_index == 0)
            tf::vectorEigenToMsg(start_pose.position, goal_position_constraint.target_point_offset);
        else
            tf::vectorEigenToMsg(target_pose.position, goal_position_constraint.target_point_offset);
        
        moveit_msgs::OrientationConstraint goal_orientation_constraint;
        goal_orientation_constraint.link_name = endeffector_name_;
        if (goal_index == 0)
            tf::quaternionEigenToMsg(start_pose.orientation, goal_orientation_constraint.orientation);
        else
            tf::quaternionEigenToMsg(target_pose.orientation, goal_orientation_constraint.orientation);
        
        moveit_msgs::Constraints goal_constraints;
        goal_constraints.position_constraints.push_back(goal_position_constraint);
        //goal_constraints.orientation_constraints.push_back(goal_orientation_constraint);
        req.goal_constraints.push_back(goal_constraints);
        
        planner_.setMotionPlanRequest(req);

        // plan and execute
        planning_interface::MotionPlanResponse res;
        planner_.planAndExecute(res);

        // visualize robot trajectory
        moveit_msgs::MotionPlanResponse response_msg;
        res.getMessage(response_msg);

        moveit_msgs::DisplayTrajectory display_trajectory_msg;
        display_trajectory_msg.trajectory_start = response_msg.trajectory_start;
        display_trajectory_msg.trajectory.push_back( response_msg.trajectory );
        display_trajectory_msg.model_id = "model";
        display_trajectory_publisher_.publish(display_trajectory_msg);
        
        // move endeffector vertically using IK to pick or place
        if (goal_index == 0)
        {
            goal_index = 1;
        }
        else
        {
            goal_index = 0;
        }
    }
}

void ITOMPFetch::runMovingArmScenarioOMPL()
{
    // prepare for OMPL
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(moveit_robot_model_));

    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    const std::string planner_plugin_name = "ompl_interface/OMPLPlanner";

    try
    {
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    }
    catch(pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
      planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
      if (!planner_instance->initialize(moveit_robot_model_, "move_group"))
        ROS_FATAL_STREAM("Could not initialize planner instance");
      ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch(pluginlib::PluginlibException& ex)
    {
      const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
      std::stringstream ss;
      for (std::size_t i = 0 ; i < classes.size() ; ++i)
        ss << classes[i] << " ";
      ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                       << "Available plugins: " << ss.str());
    }

    ROS_INFO("OMPL configurations:");
    const planning_interface::PlannerConfigurationMap& configurations = planner_instance->getPlannerConfigurations();
    for (auto it = configurations.begin(); it != configurations.end(); it++)
    {
        const std::string key = it->first;
        const planning_interface::PlannerConfigurationSettings& configuration = it->second;

        ROS_INFO(" Configuration key: %s", key.c_str());
        ROS_INFO("  group: %s", configuration.group.c_str());
        ROS_INFO("  name : %s", configuration.name.c_str());
        ROS_INFO("  config:");
        for (auto it2 = configuration.config.begin(); it2 != configuration.config.end(); it2++)
            ROS_INFO("   * %s: %s", it2->first.c_str(), it2->second.c_str());
    }


    trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager;
    // initialize trajectory_execution_manager with robot model
    // see how it is initialized with ros params at http://docs.ros.org/indigo/api/moveit_ros_planning/html/trajectory__execution__manager_8cpp_source.html#l00074
    std::string controller;
    if (nh_.getParam("moveit_controller_manager", controller))
    {
        trajectory_execution_manager.reset(new trajectory_execution_manager::TrajectoryExecutionManager(moveit_robot_model_));
    }
    else if (nh_.getParam("/move_group/moveit_controller_manager", controller))
    {
        // copy parameters from "/move_group" namespace to "~"
        bool moveit_manage_controllers = false;
        bool allowed_execution_duration_scaling = false;
        bool allowed_goal_duration_margin = false;
        bool controller_list_declared = false;
        XmlRpc::XmlRpcValue controller_list;
        double value;

        nh_.setParam("moveit_controller_manager", controller);

        if (nh_.getParam("/move_group/controller_list", controller_list))
        {
            controller_list_declared = true;
            nh_.setParam("controller_list", controller_list);
        }

        if (nh_.getParam("/move_group/moveit_manage_controllers", value))
        {
            moveit_manage_controllers = true;
            nh_.setParam("moveit_manage_controllers", value);
        }

        if (nh_.getParam("/move_group/allowed_execution_duration_scaling", value))
        {
            allowed_execution_duration_scaling = true;
            nh_.setParam("allowed_execution_duration_scaling", value);
        }

        if (nh_.getParam("/move_group/allowed_goal_duration_margin", value))
        {
            allowed_goal_duration_margin = true;
            nh_.setParam("allowed_goal_duration_margin", value);
        }

        trajectory_execution_manager.reset(new trajectory_execution_manager::TrajectoryExecutionManager(moveit_robot_model_));

        nh_.deleteParam("moveit_controller_manager");

        if (controller_list_declared)
            nh_.deleteParam("controller_list");

        if (moveit_manage_controllers)
            nh_.deleteParam("moveit_manage_controllers");

        if (allowed_execution_duration_scaling)
            nh_.deleteParam("allowed_execution_duration_scaling");

        if (allowed_goal_duration_margin)
            nh_.deleteParam("allowed_goal_duration_margin");
    }
    else
    {
        ROS_WARN("Controller is not defined. MoveItFakeControllerManager is used.");
        nh_.setParam("moveit_controller_manager", "moveit_fake_controller_manager/MoveItFakeControllerManager");
        trajectory_execution_manager.reset(new trajectory_execution_manager::TrajectoryExecutionManager(moveit_robot_model_, true));
        nh_.deleteParam("moveit_controller_manager");
    }


    // open, then close the gripper to grip an object
    openGripper();
    ROS_INFO("Waiting for 1.0 sec for grasping");
    ros::Duration(1.0).sleep();
    moveGripper(gripper_picking_distance_);

    int goal_index = 0;
    Pose start_pose;
    start_pose.position = Eigen::Vector3d(0.8, -0.7, 1.15);
    start_pose.orientation = Eigen::Quaterniond(1., 0., 0., 0.);
    Pose target_pose;
    target_pose.position = Eigen::Vector3d(0.8, 0.7, 1.15);
    target_pose.orientation = Eigen::Quaterniond(1., 0., 0., 0.);

    while (true)
    {
        ROS_INFO("Goal index: [%d]", goal_index);
        planning_interface::MotionPlanRequest req;
        req.group_name = planning_group_;

        // initialize start state with current robot state
        initializeCurrentState(req.start_state);

        // initialize start state with current state
        //initializeDefaultState(req.start_state);

        // visualize start state
        moveit_msgs::DisplayRobotState start_state_display_msg;
        start_state_display_msg.state = req.start_state;
        start_state_publisher_.publish(start_state_display_msg);

        // goal pose setting
        moveit_msgs::PositionConstraint goal_position_constraint;
        goal_position_constraint.header.frame_id = "base_link";
        goal_position_constraint.link_name = endeffector_name_;
        goal_position_constraint.weight = 1.;
        if (goal_index == 0)
            tf::vectorEigenToMsg(start_pose.position, goal_position_constraint.target_point_offset);
        else
            tf::vectorEigenToMsg(target_pose.position, goal_position_constraint.target_point_offset);

        shape_msgs::SolidPrimitive box;
        box.type = shape_msgs::SolidPrimitive::BOX;
        box.dimensions.push_back(0.1);
        box.dimensions.push_back(0.1);
        box.dimensions.push_back(0.1);

        geometry_msgs::Pose box_pose;
        box_pose.position.x = 0.;
        box_pose.position.y = 0.;
        box_pose.position.z = 0.;
        box_pose.orientation.w = 1.;
        box_pose.orientation.x = 0.;
        box_pose.orientation.y = 0.;
        box_pose.orientation.z = 0.;

        goal_position_constraint.constraint_region.primitives.push_back(box);
        goal_position_constraint.constraint_region.primitive_poses.push_back(box_pose);

        moveit_msgs::OrientationConstraint goal_orientation_constraint;
        goal_orientation_constraint.header.frame_id = "base_link";
        goal_orientation_constraint.link_name = endeffector_name_;
        goal_orientation_constraint.weight = 1.;
        goal_orientation_constraint.absolute_x_axis_tolerance = 0.1;
        goal_orientation_constraint.absolute_y_axis_tolerance = 0.1;
        goal_orientation_constraint.absolute_z_axis_tolerance = 0.1;
        if (goal_index == 0)
            tf::quaternionEigenToMsg(start_pose.orientation, goal_orientation_constraint.orientation);
        else
            tf::quaternionEigenToMsg(target_pose.orientation, goal_orientation_constraint.orientation);

        moveit_msgs::Constraints goal_constraints;
        goal_constraints.position_constraints.push_back(goal_position_constraint);
        goal_constraints.orientation_constraints.push_back(goal_orientation_constraint);
        req.goal_constraints.push_back(goal_constraints);

        // set algorithm to RRTk
        req.planner_id = "RRTkConfigDefault";
        req.allowed_planning_time = 5.0;
        req.num_planning_attempts = 1;
        req.workspace_parameters.header.frame_id = "map";
        req.workspace_parameters.min_corner.x = -10.;
        req.workspace_parameters.min_corner.y = -10.;
        req.workspace_parameters.min_corner.z = -10.;
        req.workspace_parameters.max_corner.x =  10.;
        req.workspace_parameters.max_corner.y =  10.;
        req.workspace_parameters.max_corner.z =  10.;

        // OMPL planner
        planning_interface::MotionPlanResponse res;
        planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
        context->solve(res);
        if(res.error_code_.val != res.error_code_.SUCCESS)
        {
          ROS_ERROR("Could not compute plan successfully (error code: %d)", res.error_code_.val);
          return;
        }

        // visualize robot trajectory
        moveit_msgs::MotionPlanResponse response_msg;
        res.getMessage(response_msg);

        moveit_msgs::DisplayTrajectory display_trajectory_msg;
        display_trajectory_msg.trajectory_start = response_msg.trajectory_start;
        display_trajectory_msg.trajectory.push_back( response_msg.trajectory );
        display_trajectory_msg.model_id = "model";
        display_trajectory_publisher_.publish(display_trajectory_msg);

        // execute robot trajectory
        trajectory_execution_manager->push(response_msg.trajectory);
        trajectory_execution_manager->execute();

        // move endeffector vertically using IK to pick or place
        if (goal_index == 0)
        {
            goal_index = 1;
        }
        else
        {
            goal_index = 0;
        }
    }
}

void ITOMPFetch::moveEndeffectorVertically(double distance)
{
    const double trajectory_duration = 1.0;
    const double planning_timestep = 0.1;
    
    planning_interface::MotionPlanRequest req;
    req.group_name = planning_group_;
    
    // ITOMP parameter change
    const double original_trajectory_duration = planner_.getTrajectoryDuration();
    const double original_planning_timestep = planner_.getPlanningTimestep();
    planner_.setTrajectoryDuration(trajectory_duration);
    planner_.setPlanningTimestep(planning_timestep);

    // compute endeffector pose
    robot_state::RobotState current_state( moveit_robot_model_ );
    initializeCurrentState(current_state);
    
    current_state.updateLinkTransforms();
    Eigen::Affine3d endeffector_pose = current_state.getGlobalLinkTransform(endeffector_name_);
    
    // transform vertically
    endeffector_pose.pretranslate(Eigen::Vector3d(0., 0., distance));
    
    // initialize start state with current robot state
    moveit::core::robotStateToRobotStateMsg(current_state, req.start_state, false);
    
    // visualize start state
    moveit_msgs::DisplayRobotState start_state_display_msg;
    start_state_display_msg.state = req.start_state;
    start_state_publisher_.publish(start_state_display_msg);
    
    // goal pose setting
    moveit_msgs::PositionConstraint goal_position_constraint;
    goal_position_constraint.link_name = endeffector_name_;
    tf::vectorEigenToMsg(endeffector_pose.translation(), goal_position_constraint.target_point_offset);
    
    moveit_msgs::OrientationConstraint goal_orientation_constraint;
    goal_orientation_constraint.link_name = endeffector_name_;
    tf::quaternionEigenToMsg(Eigen::Quaterniond(endeffector_pose.linear()), goal_orientation_constraint.orientation);
    
    moveit_msgs::Constraints goal_constraints;
    goal_constraints.position_constraints.push_back(goal_position_constraint);
    goal_constraints.orientation_constraints.push_back(goal_orientation_constraint);
    req.goal_constraints.push_back(goal_constraints);
    
    planner_.setMotionPlanRequest(req);
    
    // plan and execute
    planning_interface::MotionPlanResponse res;
    planner_.planAndExecute(res);
    
    // ITOMP parameter restore
    planner_.setTrajectoryDuration(original_trajectory_duration);
    planner_.setPlanningTimestep(original_planning_timestep);
    
    // visualize robot trajectory
    moveit_msgs::MotionPlanResponse response_msg;
    res.getMessage(response_msg);
}

void ITOMPFetch::moveEndeffectorVerticallyTarget(double distance, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, bool wait_for_execution)
{
    const double trajectory_duration = 1.0;
    const double planning_timestep = 0.1;
    
    planning_interface::MotionPlanRequest req;
    req.group_name = planning_group_;
    
    // ITOMP parameter change
    const double original_trajectory_duration = planner_.getTrajectoryDuration();
    const double original_planning_timestep = planner_.getPlanningTimestep();
    planner_.setTrajectoryDuration(trajectory_duration);
    planner_.setPlanningTimestep(planning_timestep);

    // compute endeffector pose
    robot_state::RobotState current_state( moveit_robot_model_ );
    initializeCurrentState(current_state);
    
    current_state.updateLinkTransforms();
    
    // initialize start state with current robot state
    moveit::core::robotStateToRobotStateMsg(current_state, req.start_state, false);
    
    // goal pose setting
    moveit_msgs::PositionConstraint goal_position_constraint;
    goal_position_constraint.link_name = endeffector_name_;
    tf::vectorEigenToMsg(position + Eigen::Vector3d(0., 0., distance), goal_position_constraint.target_point_offset);
    
    moveit_msgs::OrientationConstraint goal_orientation_constraint;
    goal_orientation_constraint.link_name = endeffector_name_;
    tf::quaternionEigenToMsg(orientation, goal_orientation_constraint.orientation);
    
    moveit_msgs::Constraints goal_constraints;
    goal_constraints.position_constraints.push_back(goal_position_constraint);
    goal_constraints.orientation_constraints.push_back(goal_orientation_constraint);
    req.goal_constraints.push_back(goal_constraints);
    
    planner_.setMotionPlanRequest(req);
    
    // plan and execute
    planning_interface::MotionPlanResponse res;
    planner_.planAndExecute(res);
    
    // ITOMP parameter restore
    planner_.setTrajectoryDuration(original_trajectory_duration);
    planner_.setPlanningTimestep(original_planning_timestep);
}

void ITOMPFetch::moveEndeffectorVerticallyIK(double distance, bool wait_for_execution)
{
    robot_state::RobotState current_state( moveit_robot_model_ );
    initializeCurrentState(current_state);
    
    current_state.updateLinkTransforms();
    Eigen::Affine3d endeffector_pose = current_state.getGlobalLinkTransform(endeffector_name_);
    
    // transform vertically
    endeffector_pose.pretranslate(Eigen::Vector3d(0., 0., distance));
    
    robot_state::RobotState goal_state = current_state;
    if (!goal_state.setFromIK( goal_state.getJointModelGroup(planning_group_), endeffector_pose, 100))
        ROS_ERROR("IK failed while moving endeffector vertically");
    
    // visualize goal state
    moveit_msgs::RobotState goal_state_msg;
    moveit::core::robotStateToRobotStateMsg(goal_state, goal_state_msg, false);
    moveit_msgs::DisplayRobotState goal_state_display_msg;
    goal_state_display_msg.state = goal_state_msg;
    goal_state_publisher_.publish(goal_state_display_msg);
    
    // execute trajectory
    const double time = 1.0;
    
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(time);
    for (int i=0; i<goal_state_msg.joint_state.name.size(); i++)
    {
        const std::string name = goal_state_msg.joint_state.name[i];
        const double position = goal_state_msg.joint_state.position[i];
        const double velocity = goal_state_msg.joint_state.velocity[i];
        
        goal.trajectory.joint_names.push_back(name);
        point.positions.push_back(position);
        point.velocities.push_back(velocity);
    }
    goal.trajectory.points.push_back(point);
    
    arm_client_.sendGoal(goal);
    
    if (wait_for_execution)
        arm_client_.waitForResult();
}

void ITOMPFetch::attachSphere(int sphere_index)
{
    const AttachingSphere& sphere = attaching_spheres_[sphere_index];
    const std::string attaching_link_name = sphere.attaching_link;
    const int attaching_link = robot_model_->getJointIndexByLinkName(attaching_link_name);

    robot_model_->attachSphere(attaching_link, sphere.attaching_position, sphere.radius);
}

void ITOMPFetch::detachSphere(int sphere_index)
{
    const AttachingSphere& sphere = attaching_spheres_[sphere_index];
    const std::string attaching_link_name = sphere.attaching_link;
    const int attaching_link = robot_model_->getJointIndexByLinkName(attaching_link_name);

    robot_model_->detachSpheres(attaching_link);
}

int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);
    
    srand(time(NULL));
    
    ros::init(argc, argv, "itomp_fetch");
    
    ITOMPFetch test_fetch;

    // initialize with highest torso position
    test_fetch.moveTorso(0.35);

    // open/close gripper at start
    /*
    test_fetch.moveGripper(0.01);
    test_fetch.openGripper();
    */


    if (argc == 1 || argv[1][0] == '1')
        test_fetch.runScenario();
    
    else
    {
        if (argc >= 3 && !strcmp(argv[2], "OMPL"))
            test_fetch.runMovingArmScenarioOMPL();
        else
            test_fetch.runMovingArmScenario();
    }
    
    return 0;
}
