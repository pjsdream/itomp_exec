/*

License

ITOMP Optimization-based Planner
Copyright © and trademark ™ 2014 University of North Carolina at Chapel Hill.
All rights reserved.

Permission to use, copy, modify, and distribute this software and its documentation
for educational, research, and non-profit purposes, without fee, and without a
written agreement is hereby granted, provided that the above copyright notice,
this paragraph, and the following four paragraphs appear in all copies.

This software program and documentation are copyrighted by the University of North
Carolina at Chapel Hill. The software program and documentation are supplied "as is,"
without any accompanying services from the University of North Carolina at Chapel
Hill or the authors. The University of North Carolina at Chapel Hill and the
authors do not warrant that the operation of the program will be uninterrupted
or error-free. The end-user understands that the program was developed for research
purposes and is advised not to rely exclusively on the program for any reason.

IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE AUTHORS
BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY STATUTORY WARRANTY
OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND
THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS HAVE NO OBLIGATIONS
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

Any questions or comments should be sent to the author chpark@cs.unc.edu

*/

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <boost/variant/get.hpp>
#include <boost/lexical_cast.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include "move_kuka_iiwa.h"
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <sched.h>
#include <limits>
#include <resource_retriever/retriever.h>
#include <vector>

using namespace std;

namespace move_kuka
{

MoveKukaIIWA::MoveKukaIIWA(const ros::NodeHandle& node_handle) :
    node_handle_(node_handle), mat_task_frames_0_(10, std::vector<Eigen::Affine3d>(65)), mat_task_frames_78_(10, std::vector<Eigen::Affine3d>(65))
{

}

MoveKukaIIWA::~MoveKukaIIWA()
{
}

void MoveKukaIIWA::run(const std::string& group_name)
{
    // scene initialization
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    planning_scene_diff_publisher_ = node_handle_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    while (planning_scene_diff_publisher_.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
        ROS_INFO("Waiting planning_scene subscribers");
    }

    loadFrames();

    vis_marker_array_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100, true);

    loadStaticScene();

    drawFrames();

    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();
    ROS_INFO("Done");
}

void MoveKukaIIWA::loadStaticScene()
{
    moveit_msgs::PlanningScene planning_scene_msg;
    std::string environment_file;
    std::vector<double> environment_position;

    node_handle_.param<std::string>("/itomp_planner/environment_model", environment_file, "");

    geometry_msgs::Pose pose;
    geometry_msgs::Pose pose2;
    if (!environment_file.empty())
    {
        double scale;
        node_handle_.param("/itomp_planner/environment_model_scale", scale, 1.0);
        environment_position.resize(3, 0);
        if (node_handle_.hasParam("/itomp_planner/environment_model_position"))
        {
            XmlRpc::XmlRpcValue segment;
            node_handle_.getParam("/itomp_planner/environment_model_position", segment);
            if (segment.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                int size = segment.size();
                for (int i = 0; i < size; ++i)
                {
                    double value = segment[i];
                    environment_position[i] = value;
                }
            }
        }

        // Collision object
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = robot_model_->getModelFrame();
        collision_object.id = "environment";

        pose.position.x = environment_position[0];
        pose.position.y = environment_position[1];
        pose.position.z = environment_position[2];
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        Eigen::Affine3d& mat = mat_shelf_frame_;
        Eigen::Quaterniond q(mat.linear());
        //q = Eigen::Quaterniond();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        pose.position.x = mat.translation().x() * 0.001;
        pose.position.y = mat.translation().y() * 0.001;
        pose.position.z = mat.translation().z() * 0.001;

        shapes::Mesh* shape = shapes::createMeshFromResource(environment_file, Eigen::Vector3d(scale, scale, scale));
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(shape, mesh_msg);
        shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

        collision_object.meshes.push_back(mesh);
        collision_object.mesh_poses.push_back(pose);

        // shelf mesh
        shape = shapes::createMeshFromResource("package://move_kuka/env/SME_Aufnahmetisch_shelf_s.dae", Eigen::Vector3d(scale, scale, scale));
        shapes::constructMsgFromShape(shape, mesh_msg);
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
        const double distLeft2Middle = 11 * 30; // mm
        const double distMiddle2Right = 10 * 30; // mm
        pose2 = pose;
        pose2.position.y += (distMiddle2Right) * 0.001;
        collision_object.meshes.push_back(mesh);
        collision_object.mesh_poses.push_back(pose2);

        collision_object.operation = collision_object.ADD;
        //moveit_msgs::PlanningScene planning_scene_msg;
        planning_scene_msg.world.collision_objects.push_back(collision_object);
        planning_scene_msg.is_diff = true;
        planning_scene_->setPlanningSceneDiffMsg(planning_scene_msg);
    }

    planning_scene_diff_publisher_.publish(planning_scene_msg);

    // mesh marker
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker msg;
    msg.header.frame_id = robot_model_->getModelFrame();
    msg.header.stamp = ros::Time::now();
    msg.ns = "base_mesh";
    msg.type = visualization_msgs::Marker::MESH_RESOURCE;
    msg.mesh_resource = environment_file;
    msg.action = visualization_msgs::Marker::ADD;

    msg.scale.x = 1.0;
    msg.scale.y = 1.0;
    msg.scale.z = 1.0;

    msg.id = 0;
    msg.color.a = 0.5;
    msg.color.r = 0.0;
    msg.color.g = 1.0;
    msg.color.b = 0.0;

    msg.pose = pose;

    ma.markers.push_back(msg);

    // shelf
    msg.ns = "shelf_mesh";
    msg.pose = pose2;
    msg.mesh_resource = "package://move_kuka/env/SME_Aufnahmetisch_shelf_s.dae";
    ma.markers.push_back(msg);

    vis_marker_array_publisher_.publish(ma);
}

void MoveKukaIIWA::loadFrames()
{
    const std::string shelf_frame_file = "package://move_kuka/env/shelf.frame.xml";
    const std::string task_frame_0_file = "package://move_kuka/env/taskFrames_angle_0.0.xml";
    const std::string task_frame_78_file = "package://move_kuka/env/taskFrames_angle_0.78.xml";

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

void MoveKukaIIWA::drawFrames()
{
    const double scale = 0.002;

    int id = 0;

    visualization_msgs::Marker::_color_type RED, GREEN, BLUE;
    RED.a = 1.0;
    RED.r = 1.0;
    RED.g = 0.0;
    RED.b = 0.0;
    GREEN.a = 1.0;
    GREEN.r = 0.5;
    GREEN.g = 1.0;
    GREEN.b = 0.0;
    BLUE.a = 1.0;
    BLUE.r = 0.0;
    BLUE.g = 0.0;
    BLUE.b = 1.0;

    visualization_msgs::MarkerArray ma;

    visualization_msgs::Marker msg;
    msg.header.frame_id = robot_model_->getModelFrame();
    msg.header.stamp = ros::Time::now();
    msg.type = visualization_msgs::Marker::LINE_LIST;
    msg.action = visualization_msgs::Marker::ADD;

    msg.scale.x = scale;
    msg.scale.y = scale;
    msg.scale.z = scale;

    msg.ns = "shelf";

    geometry_msgs::Point point_origin;
    point_origin.x = mat_shelf_frame_(0, 3) * 0.001;
    point_origin.y = mat_shelf_frame_(1, 3) * 0.001;
    point_origin.z = mat_shelf_frame_(2, 3) * 0.001;

    geometry_msgs::Point point_dir;
    const double scale_dir = 0.05;

    msg.id = ++id;
    msg.color = RED;
    msg.points.resize(0);
    point_dir.x = point_origin.x + mat_shelf_frame_(0, 0) * scale_dir;
    point_dir.y = point_origin.y + mat_shelf_frame_(1, 0) * scale_dir;
    point_dir.z = point_origin.z + mat_shelf_frame_(2, 0) * scale_dir;
    msg.points.push_back(point_origin);
    msg.points.push_back(point_dir);
    ma.markers.push_back(msg);

    msg.id = ++id;
    msg.color = GREEN;
    msg.points.resize(0);
    point_dir.x = point_origin.x + mat_shelf_frame_(0, 1) * scale_dir;
    point_dir.y = point_origin.y + mat_shelf_frame_(1, 1) * scale_dir;
    point_dir.z = point_origin.z + mat_shelf_frame_(2, 1) * scale_dir;
    msg.points.push_back(point_origin);
    msg.points.push_back(point_dir);
    ma.markers.push_back(msg);

    msg.id = ++id;
    msg.color = BLUE;
    msg.points.resize(0);
    point_dir.x = point_origin.x + mat_shelf_frame_(0, 2) * scale_dir;
    point_dir.y = point_origin.y + mat_shelf_frame_(1, 2) * scale_dir;
    point_dir.z = point_origin.z + mat_shelf_frame_(2, 2) * scale_dir;
    msg.points.push_back(point_origin);
    msg.points.push_back(point_dir);
    ma.markers.push_back(msg);

    msg.ns = "task_0";
    for (int i = 0; i < mat_task_frames_0_.size(); ++i)
    {

        for (int j = 0; j < mat_task_frames_0_[i].size(); ++j)
        {
            //if (j != 0 && i != 0)
            //  continue;
            const Eigen::Affine3d& mat = mat_task_frames_0_[i][j];
            geometry_msgs::Point point_origin;
            point_origin.x = mat.translation()(0) * 0.001;
            point_origin.y = mat.translation()(1) * 0.001;
            point_origin.z = mat.translation()(2) * 0.001;

            msg.id = ++id;
            msg.color = RED;
            msg.points.resize(0);
            point_dir.x = point_origin.x + mat(0, 0) * scale_dir;
            point_dir.y = point_origin.y + mat(1, 0) * scale_dir;
            point_dir.z = point_origin.z + mat(2, 0) * scale_dir;
            msg.points.push_back(point_origin);
            msg.points.push_back(point_dir);
            ma.markers.push_back(msg);

            msg.id = ++id;
            msg.color = GREEN;
            msg.points.resize(0);
            point_dir.x = point_origin.x + mat(0, 1) * scale_dir;
            point_dir.y = point_origin.y + mat(1, 1) * scale_dir;
            point_dir.z = point_origin.z + mat(2, 1) * scale_dir;
            msg.points.push_back(point_origin);
            msg.points.push_back(point_dir);
            ma.markers.push_back(msg);

            msg.id = ++id;
            msg.color = BLUE;
            msg.points.resize(0);
            point_dir.x = point_origin.x + mat(0, 2) * scale_dir;
            point_dir.y = point_origin.y + mat(1, 2) * scale_dir;
            point_dir.z = point_origin.z + mat(2, 2) * scale_dir;
            msg.points.push_back(point_origin);
            msg.points.push_back(point_dir);
            ma.markers.push_back(msg);
        }
    }

    msg.ns = "task_78";
    for (int i = 0; i < mat_task_frames_78_.size(); ++i)
    {
        for (int j = 0; j < mat_task_frames_78_[i].size(); ++j)
        {
            const Eigen::Affine3d& mat = mat_task_frames_78_[i][j];
            geometry_msgs::Point point_origin;
            point_origin.x = mat.translation()(0) * 0.001;
            point_origin.y = mat.translation()(1) * 0.001;
            point_origin.z = mat.translation()(2) * 0.001;

            msg.id = ++id;
            msg.color = RED;
            msg.points.resize(0);
            point_dir.x = point_origin.x + mat(0, 0) * scale_dir;
            point_dir.y = point_origin.y + mat(1, 0) * scale_dir;
            point_dir.z = point_origin.z + mat(2, 0) * scale_dir;
            msg.points.push_back(point_origin);
            msg.points.push_back(point_dir);
            ma.markers.push_back(msg);

            msg.id = ++id;
            msg.color = GREEN;
            msg.points.resize(0);
            point_dir.x = point_origin.x + mat(0, 1) * scale_dir;
            point_dir.y = point_origin.y + mat(1, 1) * scale_dir;
            point_dir.z = point_origin.z + mat(2, 1) * scale_dir;
            msg.points.push_back(point_origin);
            msg.points.push_back(point_dir);
            ma.markers.push_back(msg);

            msg.id = ++id;
            msg.color = BLUE;
            msg.points.resize(0);
            point_dir.x = point_origin.x + mat(0, 2) * scale_dir;
            point_dir.y = point_origin.y + mat(1, 2) * scale_dir;
            point_dir.z = point_origin.z + mat(2, 2) * scale_dir;
            msg.points.push_back(point_origin);
            msg.points.push_back(point_dir);
            ma.markers.push_back(msg);
        }
    }

    msg.ns = "rivet_magazine";
    const Eigen::Affine3d& mat = mat_rivet_magazine_;
    point_origin.x = mat.translation()(0) * 0.001;
    point_origin.y = mat.translation()(1) * 0.001;
    point_origin.z = mat.translation()(2) * 0.001;

    msg.id = ++id;
    msg.color = RED;
    msg.points.resize(0);
    point_dir.x = point_origin.x + mat(0, 0) * scale_dir;
    point_dir.y = point_origin.y + mat(1, 0) * scale_dir;
    point_dir.z = point_origin.z + mat(2, 0) * scale_dir;
    msg.points.push_back(point_origin);
    msg.points.push_back(point_dir);
    ma.markers.push_back(msg);

    msg.id = ++id;
    msg.color = GREEN;
    msg.points.resize(0);
    point_dir.x = point_origin.x + mat(0, 1) * scale_dir;
    point_dir.y = point_origin.y + mat(1, 1) * scale_dir;
    point_dir.z = point_origin.z + mat(2, 1) * scale_dir;
    msg.points.push_back(point_origin);
    msg.points.push_back(point_dir);
    ma.markers.push_back(msg);

    msg.id = ++id;
    msg.color = BLUE;
    msg.points.resize(0);
    point_dir.x = point_origin.x + mat(0, 2) * scale_dir;
    point_dir.y = point_origin.y + mat(1, 2) * scale_dir;
    point_dir.z = point_origin.z + mat(2, 2) * scale_dir;
    msg.points.push_back(point_origin);
    msg.points.push_back(point_dir);
    ma.markers.push_back(msg);

    vis_marker_array_publisher_.publish(ma);

    ros::WallDuration sleep_time(0.01);
    sleep_time.sleep();
}

}

int main(int argc, char **argv)
{
    // for debug
    setbuf(stdout, NULL);

    ros::init(argc, argv, "move_itomp");
    ros::NodeHandle node_handle("~");

    move_kuka::MoveKukaIIWA* move_kuka = new move_kuka::MoveKukaIIWA(node_handle);
    move_kuka->run("whole_body");
    delete move_kuka;

    return 0;
}
