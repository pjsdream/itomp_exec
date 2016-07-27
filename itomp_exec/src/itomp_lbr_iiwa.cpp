#include <itomp_exec/planner/itomp_planner_node.h>
#include <itomp_exec/robot/bounding_sphere_robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/RobotState.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <itomp_exec/util/joint_state_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <ros/ros.h>

#include <std_msgs/Bool.h>

// serialization
#include <resource_retriever/retriever.h>
#include <boost/serialization/nvp.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <stdlib.h>
#include <time.h>


/**
Serialization and deserilization of a 4 x 4 homogenous matrix.
*/
class HMatrix
{
public:
    double data_[16];
    double& operator[](std::size_t idx)       { return data_[idx]; }
    const double& operator[](std::size_t idx) const { return data_[idx]; }
    Eigen::Affine3d getEigen() const {
        Eigen::Matrix4d m;
        for (int i = 0; i < 16; ++i)
            m(i / 4, i % 4) = data_[i];
        Eigen::Affine3d am(m);
        return am;
    }
};

template<class Archive>
void serialize(Archive & ar, HMatrix& m, const unsigned int version)
{
        ar & boost::serialization::make_nvp("r11", m[0]);
        ar & boost::serialization::make_nvp("r12", m[1]);
        ar & boost::serialization::make_nvp("r13", m[2]);
        ar & boost::serialization::make_nvp("r14", m[3]);

        ar & boost::serialization::make_nvp("r21", m[4]);
        ar & boost::serialization::make_nvp("r22", m[5]);
        ar & boost::serialization::make_nvp("r23", m[6]);
        ar & boost::serialization::make_nvp("r24", m[7]);

        ar & boost::serialization::make_nvp("r31", m[8]);
        ar & boost::serialization::make_nvp("r32", m[9]);
        ar & boost::serialization::make_nvp("r33", m[10]);
        ar & boost::serialization::make_nvp("r34", m[11]);

        ar & boost::serialization::make_nvp("r41", m[12]);
        ar & boost::serialization::make_nvp("r42", m[13]);
        ar & boost::serialization::make_nvp("r43", m[14]);
        ar & boost::serialization::make_nvp("r44", m[15]);

}


class ITOMPLbriiwa
{
private:

    static const std::string planning_group_;
    static const std::string endeffector_name_;

    struct Pose
    {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
    };

public:

    ITOMPLbriiwa(const ros::NodeHandle& nh = ros::NodeHandle("~"));

    void loadVisualEnvironments();
    void loadFrames();

    void moveTorso(double position, bool wait_for_execution = true);

    void runScenario();
    void runMovingArmScenario();

private:

    void initializeCurrentState(robot_state::RobotState& state);
    void initializeCurrentState(moveit_msgs::RobotState& start_state);

    Eigen::Affine3d getGoalPoseRivetMagazine();
    Eigen::Affine3d getGoalPoseShelf(int row, int col, int dir);

    ros::NodeHandle nh_;
    itomp_exec::ITOMPPlannerNode planner_;

    robot_model::RobotModelConstPtr moveit_robot_model_;
    itomp_exec::BoundingSphereRobotModelPtr robot_model_;

    ros::Publisher display_trajectory_publisher_;
    ros::Publisher display_environments_;

    // joint state listener
    itomp_exec::JointStateListener joint_state_listener_;

    // visual meshes
    std::vector<std::pair<std::string, Eigen::Affine3d> > visual_meshes_;

    // frames
    Eigen::Affine3d mat_shelf_frame_;
    std::vector<std::vector<Eigen::Affine3d> > mat_task_frames_0_;
    std::vector<std::vector<Eigen::Affine3d> > mat_task_frames_78_;
    Eigen::Affine3d mat_rivet_magazine_;

    // demo
    ros::Publisher demo_request_publisher_;
};



const std::string ITOMPLbriiwa::planning_group_ = "whole_body";
const std::string ITOMPLbriiwa::endeffector_name_ = "revetting_tcp";



ITOMPLbriiwa::ITOMPLbriiwa(const ros::NodeHandle& nh)
    : nh_(nh)
    , planner_(nh)
    , mat_task_frames_0_(10, std::vector<Eigen::Affine3d>(65))
    , mat_task_frames_78_(10, std::vector<Eigen::Affine3d>(65))
{
    // initialize planner
    planner_.printParams();
    planner_.printControllers();
    planner_.printCostWeights();

    // load robot model
    moveit_robot_model_ = planner_.getMoveitRobotModel();
    robot_model_ = planner_.getRobotModel();

    // publisher initialization
    display_trajectory_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("display_planned_path", 1);
    display_environments_ = nh_.advertise<visualization_msgs::MarkerArray>("environments", 1);
    demo_request_publisher_ = nh_.advertise<std_msgs::Bool>("/demo/demo_request", 1);
    ros::Duration(0.5).sleep();

    loadFrames();
    loadVisualEnvironments();

    // visualize environment
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "environment";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;

    marker.color.r = 0.;
    marker.color.g = 1.;
    marker.color.b = 0.;
    marker.color.a = 1.;

    marker.mesh_use_embedded_materials = true;

    marker.scale.x = 1.;
    marker.scale.y = 1.;
    marker.scale.z = 1.;

    for (int i=0; i<visual_meshes_.size(); i++)
    {
        marker.id = i;

        marker.mesh_resource = visual_meshes_[i].first;
        tf::poseEigenToMsg(visual_meshes_[i].second, marker.pose);

        marker_array.markers.push_back(marker);
    }

    display_environments_.publish(marker_array);
}

void ITOMPLbriiwa::loadVisualEnvironments()
{
    // load static obstacles
    XmlRpc::XmlRpcValue environments;
    if (nh_.getParam("environment", environments))
    {
        for (int i=0; i<environments.size(); i++)
        {
            XmlRpc::XmlRpcValue& environment = environments[i];

            if (environment.hasMember("type"))
            {
                std::string type = static_cast<std::string>(environment["type"]);

                if (type == "mesh")
                {
                    if (environment.hasMember("file"))
                    {
                        std::string filename = static_cast<std::string>(environment["file"]);
                        Eigen::Affine3d transformation = Eigen::Affine3d::Identity();

                        if (environment.hasMember("position"))
                        {
                            Eigen::Vector3d p;
                            XmlRpc::XmlRpcValue& position = environment["position"];
                            for (int i=0; i<3; i++)
                                p(i) = static_cast<double>(position[i]);

                            transformation.translate(p);
                        }

                        if (environment.hasMember("orientation"))
                        {
                            // TODO
                        }

                        if (environment.hasMember("scale"))
                        {
                            // TODO
                        }

                        Eigen::Affine3d pre = mat_shelf_frame_;
                        pre.matrix().block(0, 3, 3, 1) /= 1000;

                        ROS_INFO("%s", filename.c_str());
                        visual_meshes_.push_back(std::make_pair(filename, transformation * pre));
                    }
                    else
                    {
                        ROS_ERROR("environment parameter does not have 'file' field.");
                        continue;
                    }
                }
                else
                {
                    ROS_ERROR("environment parameter does not support type [%s]", type.c_str());
                    continue;
                }
            }
            else
            {
                ROS_ERROR("environment parameter does not have 'type' field.");
                continue;
            }
        }
    }
}

void ITOMPLbriiwa::loadFrames()
{
    const std::string shelf_frame_file = "package://itomp_exec/env/shelf.frame.xml";
    const std::string task_frame_0_file = "package://itomp_exec/env/taskFrames_angle_0.0.xml";
    const std::string task_frame_78_file = "package://itomp_exec/env/taskFrames_angle_0.78.xml";

    {
        resource_retriever::Retriever r;
        resource_retriever::MemoryResource resource;
        try
        {
            resource = r.get(shelf_frame_file);
        }
        catch (resource_retriever::Exception& e)
        {
            ROS_ERROR("Failed to retrieve file: %s", e.what());
            return;
        }

        const char* buffer = (const char*)resource.data.get();
        std::stringstream iss(buffer);
        boost::archive::xml_iarchive ia(iss);

        Eigen::Affine3d m;
        HMatrix hm;
        ia >> BOOST_SERIALIZATION_NVP(hm);
        m = hm.getEigen();
        //std::cout << m << std::endl;
        mat_shelf_frame_ = m;
    }

    {
        resource_retriever::Retriever r;
        resource_retriever::MemoryResource resource;
        try
        {
            resource = r.get(task_frame_0_file);
        }
        catch (resource_retriever::Exception& e)
        {
            ROS_ERROR("Failed to retrieve file: %s", e.what());
            return;
        }

        const char* buffer = (const char*)resource.data.get();
        std::stringstream iss(buffer);
        boost::archive::xml_iarchive ia(iss);

        std::vector<HMatrix> hms;
        ia >> BOOST_SERIALIZATION_NVP(hms);
        for (int r = 0; r < 10; ++r)
        {
            for (int c = 0; c < 65; ++c)
            {
                mat_task_frames_0_[r][c] = hms[r * 65 + c].getEigen();
                //std::cout << mat_task_frames_0_[r][c] << std::endl;
            }
        }
    }

    {
        resource_retriever::Retriever r;
        resource_retriever::MemoryResource resource;
        try
        {
            resource = r.get(task_frame_78_file);
        }
        catch (resource_retriever::Exception& e)
        {
            ROS_ERROR("Failed to retrieve file: %s", e.what());
            return;
        }

        const char* buffer = (const char*)resource.data.get();
        std::stringstream iss(buffer);
        boost::archive::xml_iarchive ia(iss);

        std::vector<HMatrix> hms;
        ia >> BOOST_SERIALIZATION_NVP(hms);
        for (int r = 0; r < 10; ++r)
        {
            for (int c = 0; c < 65; ++c)
            {
                mat_task_frames_78_[r][c] = hms[r * 65 + c].getEigen();
                //std::cout << mat_task_frames_78_[r][c] << std::endl;
            }
        }
    }

    mat_rivet_magazine_(0, 3) = mat_shelf_frame_(0, 3) + 408.64828;
    mat_rivet_magazine_(1, 3) = mat_shelf_frame_(1, 3) - 848.06384;
    mat_rivet_magazine_(2, 3) = mat_shelf_frame_(2, 3) + 545.78644;
    mat_rivet_magazine_.linear() = Eigen::Matrix3d::Zero();
    mat_rivet_magazine_.linear()(1, 0) = 1.0;
    mat_rivet_magazine_.linear()(0, 1) = 1.0;
    mat_rivet_magazine_.linear()(2, 2) = -1.0;
}

void ITOMPLbriiwa::initializeCurrentState(robot_state::RobotState& state)
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

void ITOMPLbriiwa::initializeCurrentState(moveit_msgs::RobotState& start_state)
{
    robot_state::RobotState state( moveit_robot_model_ );
    initializeCurrentState(state);
    moveit::core::robotStateToRobotStateMsg(state, start_state, false);
}

Eigen::Affine3d ITOMPLbriiwa::getGoalPoseRivetMagazine()
{
    const double waypoint_offset = 45 * 0.001;

    Eigen::Affine3d magazine_frame = mat_rivet_magazine_;
    magazine_frame.translation() *= 0.001;
    Eigen::Affine3d magazine_frame_waypoint_frame = magazine_frame;
    magazine_frame_waypoint_frame.translation() += magazine_frame_waypoint_frame.linear() * Eigen::Vector3d(0, 0, -waypoint_offset);

    return magazine_frame_waypoint_frame;
}

Eigen::Affine3d ITOMPLbriiwa::getGoalPoseShelf(int row, int col, int dir)
{
    //const double waypoint_offset = 45 * 0.001;

    Eigen::Affine3d goal_frame = (dir == 0 ? mat_task_frames_0_ : mat_task_frames_78_)[row][col];
    goal_frame.translation() *= 0.001;
    //Eigen::Affine3d goal_final_frame = goal_frame;
    //goal_final_frame.translation() += goal_final_frame.linear() * Eigen::Vector3d(0, 0, waypoint_offset);

    return goal_frame;
}

void ITOMPLbriiwa::runScenario()
{
    int goal_type = 0;
    int goal_row = 0;
    int goal_col = 55;
    int goal_dir = 1;

    while (true)
    {
        ROS_INFO("Goal index: [%d]", goal_type);
        planning_interface::MotionPlanRequest req;
        req.group_name = planning_group_;

        // initialize start state with current robot state
        initializeCurrentState(req.start_state);

        // initialize start state with current state
        //initializeDefaultState(req.start_state);

        // visualize start state
        moveit_msgs::DisplayRobotState start_state_display_msg;
        start_state_display_msg.state = req.start_state;

        // goal pose setting
        Eigen::Affine3d goal_pose;
        if (goal_type == 0)
            goal_pose = getGoalPoseRivetMagazine();
        else
            goal_pose = getGoalPoseShelf(goal_row, goal_col, goal_dir);

        const Eigen::Vector3d goal_position = goal_pose.translation();
        const Eigen::Quaterniond goal_orientation(goal_pose.linear());

        moveit_msgs::PositionConstraint goal_position_constraint;
        goal_position_constraint.link_name = endeffector_name_;
        goal_position_constraint.weight = 1.;
        tf::vectorEigenToMsg(goal_position, goal_position_constraint.target_point_offset);

        moveit_msgs::OrientationConstraint goal_orientation_constraint;
        goal_orientation_constraint.link_name = endeffector_name_;
        goal_orientation_constraint.weight = 1.0;
        tf::quaternionEigenToMsg(goal_orientation, goal_orientation_constraint.orientation);

        moveit_msgs::Constraints goal_constraints;
        goal_constraints.position_constraints.push_back(goal_position_constraint);
        goal_constraints.orientation_constraints.push_back(goal_orientation_constraint);
        req.goal_constraints.push_back(goal_constraints);

        planner_.setMotionPlanRequest(req);

        // plan and execute
        moveit_msgs::RobotTrajectory res;
        planner_.planAndExecute(res);

        // visualize
        //planner_.visualizeTrajectory(endeffector_name_, 10);

        if (goal_type == 0)
        {
            planner_.measureCostComputationTime();
            goal_type = 1;

            std_msgs::Bool request;
            request.data = true;
            demo_request_publisher_.publish(request);
            //planner_.clearTrajectoryVisualization();
            //ros::Duration(1.5).sleep();
        }
        else
        {
            goal_type = 0;
            break;
        }

        // once
        //break;
    }
}

void ITOMPLbriiwa::runMovingArmScenario()
{
    int goal_index = 0;
    Pose start_pose;
    start_pose.position = Eigen::Vector3d(1.0, -0.5, 0.7);
    start_pose.orientation = Eigen::Quaterniond(0.707, 0., 0.707, 0.);
    Pose target_pose;
    target_pose.position = Eigen::Vector3d(1.0, 0.5, 0.7);
    target_pose.orientation = Eigen::Quaterniond(0.707, 0., 0.707, 0.);

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
        //start_state_publisher_.publish(start_state_display_msg);

        // goal pose setting
        moveit_msgs::PositionConstraint goal_position_constraint;
        goal_position_constraint.link_name = endeffector_name_;
        goal_position_constraint.weight = 1.;
        if (goal_index == 0)
            tf::vectorEigenToMsg(start_pose.position, goal_position_constraint.target_point_offset);
        else
            tf::vectorEigenToMsg(target_pose.position, goal_position_constraint.target_point_offset);

        moveit_msgs::OrientationConstraint goal_orientation_constraint;
        goal_orientation_constraint.link_name = endeffector_name_;
        goal_orientation_constraint.weight = 1.;
        if (goal_index == 0)
            tf::quaternionEigenToMsg(start_pose.orientation, goal_orientation_constraint.orientation);
        else
            tf::quaternionEigenToMsg(target_pose.orientation, goal_orientation_constraint.orientation);

        moveit_msgs::Constraints goal_constraints;
        goal_constraints.position_constraints.push_back(goal_position_constraint);
        goal_constraints.orientation_constraints.push_back(goal_orientation_constraint);
        req.goal_constraints.push_back(goal_constraints);

        planner_.setMotionPlanRequest(req);

        // plan and execute
        moveit_msgs::RobotTrajectory res;
        planner_.planAndExecute(res);

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

int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

    srand(time(NULL));

    ros::init(argc, argv, "itomp_lbr_iiwa");

    ITOMPLbriiwa itomp_lbr_iiwa;

    itomp_lbr_iiwa.runScenario();
    //itomp_lbr_iiwa.runMovingArmScenario();

    return 0;
}
