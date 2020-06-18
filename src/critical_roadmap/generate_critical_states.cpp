// standard includes 
#include <stdlib.h>
#include <iosfwd>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

// system includes
#include <ros/ros.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h> 
#include <ompl/geometric/SimpleSetup.h>
#include <sbpl_collision_checking/collision_space.h>

#include <smpl/debug/visualizer_ros.h>
#include <smpl/distance_map/euclid_distance_map.h>

// project includes
#include "critical_roadmap/critical_prm.h"
#include "critical_roadmap/critical_common.h"
#include "critical_roadmap/critical_prm_constructor.h"
#include "config/planner_config.h"
#include "config/collision_space_scene.h"
#include "config/get_collision_objects.h"

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
    auto df_res = 0.05;
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
        return 1;
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

    cc.setWorldToModelTransform(Eigen::Affine3d::Identity());

    ///////////////////
    // Planner Setup //
    ///////////////////

    ROS_INFO("Initialize the planner");

    // Construct state space from the urdf + planning group...
    auto urdf = urdf::parseURDF(robot_description);
    if (!urdf) {
        ROS_ERROR("Failed to parse URDF");
        return 1;
    }

    auto state_space = ConstructWalkerStateSpaceWithDubinsBase(*urdf, 
        robot_config.planning_joints);

    ompl::geometric::SimpleSetup ss(state_space);

    // Use CollisionSpace as the collision checker...
    ss.setStateValidityChecker([&](const ompl::base::State* state)
    {
        std::vector<double> values;// = state->reals();
        state_space->copyToReals(values, state);
        return cc.isStateValid(values);
    });

    // Set up a projection evaluator to provide forward kinematics...
    auto* fk_projection = new ProjectionEvaluatorFK(state_space);
    fk_projection->model = rm.get();
    state_space->registerProjection(
            "fk", ompl::base::ProjectionEvaluatorPtr(fk_projection));

    //////////////
    // Planning //
    //////////////

    // To Do: change this so it passes pdef, space, state space directly and not ss 
    ROS_INFO("Setup the query");

    // Initialize random start/goal, only for ss setup not used in crit prm construction
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(state_space);
    start.random(); 
    ss.setStartState(start); 

    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(state_space);
    goal.random(); 
    ss.setGoalState(goal); 

    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    ss.setup();     

    // Read Critical Roadmap Construction Parameters
    smpl::ConstructionConfig cfg; 
    if (!ReadCriticalConstructionConfig(ros::NodeHandle("~critical_config"), cfg)) {
        ROS_ERROR("Failed to read construction parameters"); 
    }

    // Construct Critical PRM 
    auto crit_ctor = smpl::CriticalPRMConstructor(ss, cfg); 
    crit_ctor.constructCriticalPRM(); 

    // Test loading prm 
    auto pdata_storage = new ompl::base::PlannerDataStorage(); 
    std::string pdata_filename = cfg.roadmap_dir + "/pdata_noncritical"; 
    ompl::base::PlannerData pdata(ss.getSpaceInformation());     
    pdata_storage->load(pdata_filename.c_str(), pdata);

    auto prm = new ompl::geometric::CriticalPRM(pdata); 
    ROS_ERROR("Initialized prm with %d vertices, %d edges", 
        prm->milestoneCount(), prm->edgeCount());     

    return 0; 


    // below is old 

    // // Initialize critical roadmap construction parameters 
    // smpl::ConstructionConfig cfg; // To Do: Move these to rosparams
    // cfg.using_star_strategy = false; 

    // // test params
    // // cfg.grow_time = 10.0; 
    // // cfg.expand_time = 0.0;    
    // // cfg.expand_iters = 1e2; 
    // // cfg.num_sampled_starts = 1; 
    // // cfg.num_sampled_goals_per_start = 3;

    // cfg.grow_time =300.0; 
    // cfg.expand_iters = 1e6;
    // // cfg.expand_time = 120.0;
    // cfg.num_sampled_starts = 50; 
    // cfg.num_sampled_goals_per_start = 200; 
    // cfg.save_roadmap = true; 
    // std::string crit_road_dir = "/home/allen/catkin_ws/src/walker_planner/critical_roadmaps/"; 
    // std::string crit_road_label = "grow300_expand300_s50_g2000_dubins0_17"; 
    // cfg.save_dir = crit_road_dir + crit_road_label; 


    // ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    // ss.setup(); 

    // // Construct Critical PRM 
    // auto crit_ctor = smpl::CriticalPRMConstructor(ss, cfg);
    // crit_ctor.constructCriticalPRM(); 


    // // Test loading prm 
    // // std::string f_grown_milestones = cfg.save_dir + "/load/grown_milestones.csv"; 
    // // auto prm = new ompl::geometric::CriticalPRM(ss.getSpaceInformation()); 
    // // prm->setProblemDefinition(ss.getProblemDefinition()); 
    // // prm->growRoadmapFromFile(f_grown_milestones); 


    // // prm->seed(7); 
    // // prm->expandRoadmap(cfg.expand_iters); 
    // //     ROS_ERROR("Initialized prm with %d vertices, %d edges", 
    // //         prm->milestoneCount(), prm->edgeCount());     

    // // return 0; 


    // // Vis 
    // ROS_INFO("Visualizing scene"); 
    // while (ros::ok()) {
    //     SV_SHOW_INFO(cc.getCollisionRobotVisualization());
    //     SV_SHOW_INFO(cc.getCollisionWorldVisualization());
    //     SV_SHOW_INFO(grid.getOccupiedVoxelsVisualization());        
    //     SV_SHOW_INFO(grid.getBoundingBoxVisualization());                        
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));        
    // }
    
    // return 0;
}
