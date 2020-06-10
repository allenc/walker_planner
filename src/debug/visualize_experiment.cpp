// standard includes 
#include <stdlib.h>
#include <iosfwd>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <kdl_conversions/kdl_msg.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <ros/ros.h>
#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <smpl/angles.h>
#include <smpl/debug/visualizer_ros.h>
#include <smpl/debug/marker_conversions.h> 
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/planning_params.h>
#include <urdf_parser/urdf_parser.h>

#include "critical_roadmap/critical_prm.h"
#include "critical_roadmap/critical_prm_constructor.h"
#include "config/planner_config.h"
#include "config/collision_space_scene.h"
#include "config/get_collision_objects.h"

static
auto MakePathVisualization(
    smpl::CollisionChecker* cc, 
    const std::vector<smpl::RobotState>& path, 
    const std::vector<float>& rgba, 
    const std::string& ns)
    -> std::vector<smpl::visual::Marker>
{
    std::vector<smpl::visual::Marker> ma;

    if (path.empty()) {
        return ma;
    }

    auto cinc = 1.0f / float(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        auto markers = cc->getCollisionModelVisualization(path[i]);

        for (auto& marker : markers) {
            auto r = rgba[0]; 
            auto g = rgba[1]; 
            auto b = rgba[2]; 
            auto a = rgba[3]; 
            marker.color = smpl::visual::Color{ r, g, b, a };
        }

        for (auto& m : markers) {
            ma.push_back(std::move(m));
        }
    }

    for (size_t i = 0; i < ma.size(); ++i) {
        auto& marker = ma[i];
        marker.ns = ns; 
        marker.id = i;
    }

    return ma;
}

static
visualization_msgs::Marker GetGoalPoseVisualization(
    std::vector<double>& goal_pose, 
    std::string& ref_frame, 
    std::string& ns) 
{

    Eigen::Quaterniond q; 
    smpl::from_euler_zyx<double>(goal_pose[5], goal_pose[4], goal_pose[3], q); 

    // Make goal pose marker 
    visualization_msgs::Marker m; 
    m.pose.position.x = goal_pose[0]; 
    m.pose.position.y = goal_pose[1]; 
    m.pose.position.z = goal_pose[2];       

    m.pose.orientation.x = q.x(); 
    m.pose.orientation.y = q.y(); 
    m.pose.orientation.z = q.z(); 
    m.pose.orientation.w = q.w();   

    m.type = visualization_msgs::Marker::ARROW; 
    m.action = visualization_msgs::Marker::ADD; 
    m.header.frame_id = ref_frame; 
    m.ns = ns; 
    
    m.scale.x = 0.1; 
    m.scale.y = 0.02; 
    m.scale.z = 0.02; 

    // m.scale.x = 0.5; 
    // m.scale.y = 0.5; 
    // m.scale.z = 0.5;     

    m.color.a = 1.0; 
    m.color.r = 0.0; 
    m.color.g = 0.0;    
    m.color.b = 1.0; 

    return m; 
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "test_ompl_planner");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    ROS_INFO("Initialize visualizer");
    smpl::VisualizerROS visualizer(nh, 100);
    smpl::viz::set_visualizer(&visualizer);

    // Let publishers set up
    ros::Duration(1.0).sleep();

    /////////////////
    // Robot Model //
    /////////////////

    ROS_INFO("Load common parameters");

    // Robot_description required to initialize collision checker, state space,
    // and forward kinematics...
    auto robot_description_key = "robot_description";
    std::string robot_description_param;
    if (!nh.searchParam(robot_description_key, robot_description_param)) {
        ROS_ERROR("Failed to find 'robot_description' key on the param server");
        return 1;
    }

    std::string robot_description;
    if (!nh.getParam(robot_description_param, robot_description)) {
        ROS_ERROR("Failed to retrieve param 'robot_description' from the param server");
        return 1;
    }

    // Planning group required to initialize collision checker and state
    // space...
    RobotModelConfig robot_config;
    if (!ReadRobotModelConfig(ros::NodeHandle("~robot_model"), robot_config)) {
        ROS_ERROR("Failed to read robot model config from param server");
        return 1;
    }

    // Everyone needs to know the name of the planning frame for reasons...
    // ...frame_id for the occupancy grid (for visualization)
    // ...frame_id for collision objects (must be the same as the grid, other than that, useless)
    std::string planning_frame;
    if (!ph.getParam("planning_frame", planning_frame)) {
        ROS_ERROR("Failed to retrieve param 'planning_frame' from the param server");
        return 1;
    }

    ////////////////////
    // Occupancy Grid //
    ////////////////////

    ROS_INFO("Initialize Occupancy Grid");

    auto df_size_x = 20.0;
    auto df_size_y = 15.0;
    auto df_size_z = 1.5;
    // auto df_res = 0.05;
    auto df_res = 0.1;    
    auto df_origin_x = 0.0;
    auto df_origin_y = 0.0;
    auto df_origin_z = 0.0;
    auto max_distance = 1.8;

    using DistanceMapType = smpl::EuclidDistanceMap;

    auto df = std::make_shared<DistanceMapType>(
            df_origin_x, df_origin_y, df_origin_z,
            df_size_x, df_size_y, df_size_z,
            df_res,
            max_distance);

    auto ref_counted = false;
    smpl::OccupancyGrid grid(df, ref_counted);

    grid.setReferenceFrame(planning_frame);
    SV_SHOW_INFO(grid.getBoundingBoxVisualization());

    //////////////////////////////////
    // Initialize Collision Checker //
    //////////////////////////////////

    ROS_INFO("Initialize collision checker");

    // This whole manage storage for all the scene objects and must outlive
    // its associated CollisionSpace instance.
    CollisionSpaceScene scene;

    smpl::collision::CollisionModelConfig cc_conf;
    if (!smpl::collision::CollisionModelConfig::Load(ph, cc_conf)) {
        ROS_ERROR("Failed to load Collision Model Config");
        return 1;
    }

    smpl::collision::CollisionSpace cc;
    if (!cc.init(
            &grid,
            robot_description,
            cc_conf,
            robot_config.group_name,
            robot_config.planning_joints))
    {
        ROS_ERROR("Failed to initialize Collision Space");
        return 1;
    }

    /////////////////
    // Setup Scene //
    /////////////////

    ROS_INFO("Initialize scene");

    scene.SetCollisionSpace(&cc);
    auto map_config = getMultiRoomMapConfig(ph);
    std::vector<moveit_msgs::CollisionObject> tmp;
    auto objects = GetMultiRoomMapCollisionCubes(grid.getReferenceFrame(), map_config, tmp);
    for (auto& object : objects) {
        scene.ProcessCollisionObjectMsg(object);
    }

    SV_SHOW_INFO(cc.getCollisionRobotVisualization());
    SV_SHOW_INFO(cc.getCollisionWorldVisualization());
    SV_SHOW_INFO(grid.getOccupiedVoxelsVisualization());

    ros::Duration(1.0).sleep();

    auto rm = SetupRobotModel(robot_description, robot_config);
    if (!rm) {
        ROS_ERROR("Failed to set up Robot Model");
        return 1;
    }

    // Read in start state from file and update the scene...
    // Start state is also required by the planner...
    moveit_msgs::RobotState start_state;
    if (!ReadInitialConfiguration(ph, start_state)) {
        ROS_ERROR("Failed to get initial configuration.");
    }

    // Read in goal joint state if available 
    moveit_msgs::RobotState goal_state;    
    if (!ReadJointStateGoalConfiguration(ph, goal_state)) {
        ROS_ERROR("Failed to get goal configuration"); 
    }

    // Read in goal pose 
    std::vector<double> goal(6, 0); 
    if (!ReadPoseGoal(ph, goal)) {
        ROS_ERROR("Failed to get goal pose.");
    }        

    // Read in critical states if available
    std::vector<smpl::RobotState> critical_states; 
    ROS_INFO("Getting critical states");     
    if (!ReadCriticalStateConfigurations(ph, critical_states)) { 
        ROS_ERROR("Failed to get critical configurations"); 
    }    

    // Set reference state in the robot planning model...
    smpl::urdf::RobotState reference_state;
    InitRobotState(&reference_state, &rm->m_robot_model);
    for (auto i = 0; i < start_state.joint_state.name.size(); ++i) {
        auto* var = GetVariable(&rm->m_robot_model, &start_state.joint_state.name[i]);
        if (var == NULL) {
            ROS_WARN("Failed to do the thing");
            continue;
        }
        ROS_INFO("Set joint %s to %f", start_state.joint_state.name[i].c_str(), start_state.joint_state.position[i]);
        SetVariablePosition(&reference_state, var, start_state.joint_state.position[i]);
    }

    // Set reference state in the collision model...
    SetReferenceState(rm.get(), GetVariablePositions(&reference_state));
    if (!scene.SetRobotState(start_state)) {
        ROS_ERROR("Failed to set start state on Collision Space Scene");
        return 1;
    }

    cc.setPadding(0.02); 
    cc.setWorldToModelTransform(Eigen::Affine3d::Identity());

    /////////////////////
    // Visualizations  //
    /////////////////////

    // Get start markers 
    std::vector<double> start_joint_state(start_state.joint_state.position.begin(), 
            start_state.joint_state.position.end());   
    auto start_state_markers = cc.getCollisionRobotVisualization(start_joint_state);

    for (auto& m : start_state_markers.markers) {
        m.ns = "start_config"; 
    }    

    // Get goal joint state markers 
    std::vector<double> goal_joint_state(goal_state.joint_state.position.begin(), 
            goal_state.joint_state.position.end()); 
    auto goal_state_markers = cc.getCollisionRobotVisualization(goal_joint_state);

    for (auto& m : goal_state_markers.markers) {
        m.ns = "goal_config"; 
    }        

    // Get goal pose markers
    std::string ns_goal = "goal_pose";
    auto goal_pose_marker = GetGoalPoseVisualization(goal, planning_frame, ns_goal);     

    // Get critical markers
    std::string ns_crit = "critical_config";
    std::vector<float> rgba = { 1.0, 0.0, 0.0, 1.0 };         
    auto crit_markers = MakePathVisualization(&cc, critical_states, rgba, ns_crit); 

    ROS_INFO("Visualizing experiment"); 
    while (ros::ok()) {

        // Vis scene
        SV_SHOW_INFO(cc.getCollisionRobotVisualization());
        SV_SHOW_INFO(cc.getCollisionWorldVisualization());
        SV_SHOW_INFO(grid.getOccupiedVoxelsVisualization());        
        SV_SHOW_INFO(grid.getBoundingBoxVisualization());                        

        // Vis problem configs/pose
        SV_SHOW_INFO(start_state_markers);
        SV_SHOW_INFO(goal_pose_marker); 
        SV_SHOW_INFO(crit_markers);                 
        SV_SHOW_INFO(goal_state_markers);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));        
    }
    
    return 0;
}
