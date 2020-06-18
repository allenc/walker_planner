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

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h> 
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>

#include <smpl/debug/visualizer_ros.h>
#include <smpl/utils/debugging_utils.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl_ompl_interface/ompl_interface.h>

// project includes
#include "critical_roadmap/critical_common.h"
#include "critical_roadmap/critical_prm.h"
#include "critical_roadmap/critical_prm_constructor.h"
#include "config/planner_config.h"
#include "config/collision_space_scene.h"
#include "config/get_collision_objects.h"

// struct ExperimentConfig 
// {
//     // plan params
//     std::string ompl_planner;     
//     std::string roadmap_filename;
//     std::string crit_states_filename;
//     double allowed_planning_time;     
//     std::string exp_id;     

//     // plan res params
//     bool use_post_processing;     
//     bool use_visualization; 
//     bool use_logging;    
//     std::string log_filename;         
// }; 

// static 
// void ReadExperimentConfig(const ros::NodeHandle& nh, ExperimentConfig& cfg)
// {
//     nh.param<std::string>("ompl_planner", cfg.ompl_planner, "prm"); 
//     nh.param<std::string>("roadmap_filename", cfg.roadmap_filename, "");
//     nh.param<std::string>("crit_states_filename", cfg.crit_states_filename, "");
//     nh.param<double>("allowed_planning_time", cfg.allowed_planning_time, 60.0);
//     nh.param<std::string>("exp_id", cfg.exp_id, "0"); 
//     nh.param<bool>("use_post_processing", cfg.use_post_processing, true); 
//     nh.param<bool>("use_visualization", cfg.use_visualization, false);     
//     nh.param<bool>("use_logging", cfg.use_logging, false);     

//     if (cfg.use_logging) {
//         nh.param<std::string>("log_filename", cfg.log_filename, ""); 
//     }
// }

struct OMPLExperimentConfig 
{
    std::string planner_id;
    std::string roadmap_dir; 
    double planning_time; 

    std::string exp_id; 

    bool use_visualization; 
    bool use_logging; 
    bool use_post_processing;
    std::string log_filename; 
}; 

bool ReadOMPLExperimentConfig(
    const ros::NodeHandle& nh,
    OMPLExperimentConfig& config) 
{
    if (!nh.getParam("planner_id", config.planner_id)) {
        ROS_ERROR("Failed to read 'planner_id' from the param server");
        return false;
    } else {
        ROS_INFO("Planner id: %s", config.planner_id.c_str()); 
    }    

    std::string critical_roadmap_dirs; 
    if (!nh.getParam("roadmap_dir", critical_roadmap_dirs)) {
        ROS_ERROR("Failed to read param 'roadmap_dir' from the param server");
        return false;
    }

    std::string roadmap_label; 
    if (!nh.getParam("roadmap_label", roadmap_label)) {
        ROS_ERROR("Failed to read param 'roadmap_label' from the param server");
        return false;
    }    

    config.roadmap_dir = critical_roadmap_dirs + roadmap_label; 
    ROS_INFO("Roadmap dir: %s", config.roadmap_dir.c_str());    

    if (!nh.getParam("planning_time", config.planning_time)) {
        ROS_ERROR("Failed to read 'planning_time' from the param server");
        return false;
    } else {
        ROS_INFO("Planning time: %f", config.planning_time); 
    }    

    if (!nh.getParam("exp_id", config.exp_id)) {
        ROS_ERROR("Failed to read 'exp_id' from the param server");
        return false;
    } else {
        ROS_INFO("Exp id: %s", config.exp_id.c_str()); 
    }


    if (!nh.getParam("animate", config.use_visualization)) {
        ROS_ERROR("Failed to read param 'animate' from the param server");
        return false;
    } else {
        ROS_INFO("Use visualization: %d", config.use_visualization); 
    }    

    if (!nh.getParam("logging", config.use_logging)) {
        ROS_ERROR("Failed to read param 'logging' from the param server");
        return false;
    } else {
        ROS_INFO("Use logging: %d", config.use_logging); 
    }    

    if (!nh.getParam("post_processing", config.use_post_processing)) {
        ROS_ERROR("Failed to read param 'post_processing' from the param server");
        return false;
    } else {
        ROS_INFO("Use post processing: %d", config.use_post_processing); 
    }        

    if (config.use_logging) {
        if (!nh.getParam("log_filename", config.log_filename)) {
            ROS_ERROR("Failed to read param 'log_filename' from the param server");
            return false;
        } else {
            ROS_INFO("Log filename: %s", config.log_filename.c_str()); 
        }            
    }

    return true; 
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
    auto df_res = 0.1;    
    // auto df_res = 0.05;
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

    // Use Collision space as motion validator 
    // ss.getSpaceInformation()->setMotionValidator(std::make_shared<WalkerMotionValidator>(state_space, cc)); 

    // Set up a projection evaluator to provide forward kinematics...
    auto* fk_projection = new ProjectionEvaluatorFK(state_space);
    fk_projection->model = rm.get();
    state_space->registerProjection(
            "fk", ompl::base::ProjectionEvaluatorPtr(fk_projection));

    // Initialize optimization objective
    auto opt_obj = std::make_shared<ompl::base::PathLengthOptimizationObjective>(
        ss.getSpaceInformation());    
    opt_obj->setCostThreshold(opt_obj->infiniteCost()); // Terminate on first solution    
    ss.setOptimizationObjective(opt_obj);

    // Initialize start state
    std::vector<double> start_joints(start_state.joint_state.position);
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(state_space);
    for (int i = 0; i < start_joints.size(); ++i) {
        start[i] = start_joints[i];
    }
    ss.setStartState(start); 

    // Initialize goal state
    // Note: For goal poses, we need to add a ik projection for ompl goal sampling    
    moveit_msgs::RobotState goal_state;
    if (!ReadJointStateGoalConfiguration(ph, goal_state)) {
        ROS_ERROR("Failed to get initial configuration.");
        return 1;
    }    

    std::vector<double> goal_joints(goal_state.joint_state.position);
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(state_space);
    for (int i = 0; i < goal_joints.size(); ++i) {
        goal[i] = goal_joints[i];
    }    

    ss.setGoalState(goal);
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);

    // Initialize OMPL Planner 

    OMPLExperimentConfig cfg; 
    ReadOMPLExperimentConfig(ros::NodeHandle("~planning"), cfg); 

    auto search_mode = ompl::geometric::CriticalPRM::QUERY; 
    
    int expand_iters = 1e6; 

    if (cfg.planner_id == "prm" || cfg.planner_id == "critical_prm") 
    {
        // Load PlannerData
        auto pdata_storage = new ompl::base::PlannerDataStorage();         
        ompl::base::PlannerData pdata(ss.getSpaceInformation());     
        std::string pdata_filename = cfg.roadmap_dir + "/pdata_noncritical";         
        pdata_storage->load(pdata_filename.c_str(), pdata);

        auto prm = new ompl::geometric::CriticalPRM(pdata); 

        if (cfg.planner_id == "critical_prm") {
            // To Do: Add critical connections        
            // crit_prm->addCriticalConnections(cfg.crit_states_filename); 
            ROS_INFO("Not implemented yet"); 
        }

        ROS_INFO("Initialized prm with %d vertices, %d edges", 
            prm->milestoneCount(), prm->edgeCount());         

        prm->setProblemDefinition(ss.getProblemDefinition()); 
        prm->setMode(search_mode); 
        ss.setPlanner(ompl::base::PlannerPtr(prm)); 

    } else if (cfg.planner_id == "rrt_connect") {
        auto* rrtc = new ompl::geometric::RRTConnect(ss.getSpaceInformation());        
        ss.setPlanner(ompl::base::PlannerPtr(rrtc));         
    } else {
        ROS_ERROR("Unrecognized OMPL planner %s", cfg.planner_id.c_str()); 
        return 0; 
    }

    //////////////
    // Planning //
    //////////////

    auto solved = ss.solve(cfg.planning_time);

    ///////////////////////////////////
    // Visualizations and Statistics //
    ///////////////////////////////////

    auto found_path = ss.getSolutionPath(); 
    found_path.interpolate(100);         
    found_path.printAsMatrix(std::cout); 

    if (solved && cfg.use_visualization) {
        size_t pidx = 0;
        while (ros::ok()) { 
            auto* state = found_path.getState(pidx);           
            auto point = smpl::MakeStateSMPL(state_space.get(), state);
            auto markers = cc.getCollisionRobotVisualization(point);
            for (auto& m : markers.markers) {
                m.ns = "path_animation";
            }
            // Vis solution
            SV_SHOW_INFO(markers);

            // Vis scene 
            SV_SHOW_INFO(cc.getCollisionRobotVisualization());
            SV_SHOW_INFO(cc.getCollisionWorldVisualization());
            SV_SHOW_INFO(cc.getOccupiedVoxelsVisualization());            

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            pidx++;
            pidx %= found_path.getStateCount();            
        }
    }

    if (cfg.use_logging) {
        smpl::Writer w (cfg.log_filename); 

        std::string entry = "-1, -1, -1, -1, -1, -1, -1\n"; // fail case
        if (solved) {
            auto time = std::to_string(ss.getLastPlanComputationTime()); 
            auto expansions = "-1"; // No expansions
            auto cost_planner = "-1"; // Planning cost is same as planner interface cost (path length)
            auto cost_pi = std::to_string(ss.getSolutionPath().cost(opt_obj).value()); 

            // Post processing 
            if (cfg.use_post_processing) {
                ss.simplifySolution();
            }            
            auto time_processed = cfg.use_post_processing ? \
                std::to_string(ss.getLastSimplificationTime()) : "-1"; 
            auto cost_processed = cfg.use_post_processing ? \
                std::to_string(ss.getSolutionPath().cost(opt_obj).value()) : "-1"; 
            entry = time + ", " + \
            expansions + ", " + \
            cost_planner + ", " + \
            cost_pi + ", " + \
            time_processed + ", " + \
            cost_processed + ", " + \
            cfg.exp_id + "\n"; 
        }

        w.Write(entry); 
    }

    return !solved;
}
