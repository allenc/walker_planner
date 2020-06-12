#include <tf2/LinearMath/Quaternion.h>
#include <sstream>
#include <memory>
#include <fstream>
#include <algorithm>

#include <ros/ros.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manip_lattice_walker.h>
#include <smpl/graph/motion_primitive.h>

#include <smpl/heuristic/bfs_2d_heuristic.h>
#include <smpl/heuristic/bfs_3d_heuristic.h>
#include <smpl/heuristic/bfs_3d_base_heuristic.h>
#include <smpl/heuristic/bfs_fullbody_heuristic.h>
#include <smpl/heuristic/bfs_base_rot_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/euclid_diffdrive_heuristic.h>
#include <smpl/heuristic/arm_retract_heuristic.h>
#include <smpl/heuristic/mother_heuristic.h>

#include <smpl/distance_map/euclid_distance_map.h>

#include <smpl/ros/critical_planner_interface.h>
// #include <smpl/ros/planner_interface.h>

#include <smpl/utils/debugging_utils.h>
#include <smpl/utils/logging_utils.h>

#include <smpl/debug/visualizer_ros.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>

#include "config/planner_config.h"
#include "config/collision_space_scene.h"
#include "config/get_collision_objects.h"

// #include "motion_planner.h"
// #include "motion_planner_ros.h"
// #include "utils/utils.h"
// #include "walker_planner/GraspPose.h"
// #include "walker_planner/Path1.h"

using Path = std::vector< std::array<double, 3> >;
using Point = std::pair<double, double>;

using namespace std;

struct PlanFeatures {
    std::vector<double> x_rel_door;
    std::vector<double> y_rel_door;
    std::vector<bool> base_path_through_door;
};

// bool constructHeuristics(
//         std::vector<std::unique_ptr<smpl::RobotHeuristic>>& heurs,
//         smpl::ManipLattice* pspace,
//         smpl::OccupancyGrid* grid,
//         smpl::KDLRobotModel* rm,
//         PlannerConfig& params ){

//     SMPL_INFO("Initialize Heuristics");
//     const int DefaultCostMultiplier = 1000;

//     struct AnchorHeuristic : public BfsHeuristic {
//         int GetGoalHeuristic(int state_id) override {
//             return std::max(bfs_3d_base->GetGoalHeuristic(state_id), bfs_3d->GetGoalHeuristic(state_id));
//         }

//         double getMetricGoalDistance(double x, double y, double z) override {
//             return bfs_3d->getMetricGoalDistance(x, y, z);
//         }

//         double getMetricStartDistance(double x, double y, double z) override {
//             return bfs_3d->getMetricStartDistance(x, y, z);
//         }
//     };

//     /*
//     struct EndEffHeuristic : public BfsHeuristic {
//         bool init(std::shared_ptr<smpl::Bfs3DBaseHeuristic> _bfs_3d_base,
//                 std::shared_ptr<smpl::Bfs3DHeuristic> _bfs_3d){
//             if(!BfsHeuristic::init(_bfs_3d_base, _bfs_3d))
//                 return false;
//             pose_ext = bfs_3d->planningSpace()->getExtension<smpl::PoseProjectionExtension>();
//             return true;
//         }

//         int GetGoalHeuristic(int state_id){
//             if (state_id == bfs_3d->planningSpace()->getGoalStateID()) {
//                 return 0;
//             }
//             if(pose_ext == nullptr)
//                 return 0;
//             smpl::Affine3 p;
//             if(!pose_ext->projectToPose(state_id, p))
//                 return 0;

//             auto goal_pose = bfs_3d->planningSpace()->goal().pose;

//             smpl::Quaternion qa(p.rotation());
//             smpl::Quaternion qb(goal_pose.rotation());
//             double dot = qa.dot(qb);
//             if (dot < 0.0) {
//                 qb = smpl::Quaternion(-qb.w(), -qb.x(), -qb.y(), -qb.z());
//                 dot = qa.dot(qb);
//             }
//             int rot_dist = DefaultCostMultiplier*smpl::angles::normalize_angle(2.0 * std::acos(dot));

//             int base_dist = bfs_3d_base->GetGoalHeuristic(state_id);
//             int arm_dist = bfs_3d->GetGoalHeuristic(state_id);

//             int heuristic = base_coeff*base_dist + arm_coeff*arm_dist + rot_coeff*rot_dist;
//             ROS_ERROR("%d + %d + %d = %d", int(base_coeff*base_dist), int(arm_coeff*arm_dist), int(rot_coeff*rot_dist), heuristic);
//             return heuristic;
//         }

//         double base_coeff=0.02;
//         double arm_coeff=1;
//         double rot_coeff=1;
//         smpl::PoseProjectionExtension* pose_ext = nullptr;
//     };
//     */

//     struct RetractArmHeuristic : public BfsHeuristic {
//         bool init(std::shared_ptr<smpl::Bfs3DBaseHeuristic> _bfs_3d_base,
//                 std::shared_ptr<smpl::Bfs3DHeuristic> _bfs_3d){
//             if(!BfsHeuristic::init(_bfs_3d_base, _bfs_3d))
//                 return false;
//             return true;
//         }

//         int GetGoalHeuristic(int state_id){
//             if(state_id == 0)
//                 return 0;

//             smpl::Vector3 p;
//             if(!bfs_3d->m_pp->projectToPoint(state_id, p)){
//                 SMPL_ERROR("RetractArmHeuristic Could not project");
//                 return 0;
//             }
//             auto retracted_robot_state = bfs_3d->planningSpace()->getExtension<smpl::ExtractRobotStateExtension>()->extractState(state_id);

//             /* End-effector distance
//             for(int i=3; i<retracted_robot_state.size(); i++)
//                 retracted_robot_state[i] = 0;
//             smpl::Vector3 retracted_p;
//             if(!bfs_3d->m_pp->projectToPoint(retracted_robot_state, retracted_p)){
//                 SMPL_ERROR("RetractArmHeuristic Could not project");
//                 return 0;
//             }
//             int retract_heuristic = euclidDist(p.data(), retracted_p.data(), 3) * DefaultCostMultiplier;
//             */

//             // Norm of first 5 joints.
//             double norm = 0.0;
//             for(int i=3; i<8; i++)
//                 norm += (retracted_robot_state[i]*retracted_robot_state[i]);
//             norm = sqrt(norm);
//             int retract_heuristic = DefaultCostMultiplier * norm;

//             int base_dist = bfs_3d_base->GetGoalHeuristic(state_id);

//             int heuristic = base_coeff*base_dist + retract_arm_coeff*retract_heuristic;
//             //ROS_ERROR("%d + %d = %d", base_dist, int(retract_arm_coeff*retract_heuristic), heuristic);

//             return heuristic;
//         }

//         double base_coeff = 0.0;
//         double retract_arm_coeff = 10.0;

//     };

//     struct ImprovedEndEffHeuristic : public BfsHeuristic {
//         bool init(std::shared_ptr<smpl::Bfs3DBaseHeuristic> _bfs_3d_base,
//                 std::shared_ptr<smpl::Bfs3DHeuristic> _bfs_3d,
//                 std::shared_ptr<RetractArmHeuristic> _retract_arm){
//             if(!BfsHeuristic::init(_bfs_3d_base, _bfs_3d))
//                 return false;
//             m_retract_arm_heur = _retract_arm;
//             pose_ext = bfs_3d->planningSpace()->getExtension<smpl::PoseProjectionExtension>();
//             return true;
//         }

//         int GetGoalHeuristic(int state_id){
//             if (state_id == bfs_3d->planningSpace()->getGoalStateID()) {
//                 return 0;
//             }
//             if(pose_ext == nullptr)
//                 return 0;
//             smpl::Affine3 p;
//             if(!pose_ext->projectToPose(state_id, p))
//                 return 0;

//             auto goal_pose = bfs_3d->planningSpace()->goal().pose;

//             smpl::Quaternion qa(p.rotation());
//             smpl::Quaternion qb(goal_pose.rotation());
//             double dot = qa.dot(qb);
//             if (dot < 0.0) {
//                 qb = smpl::Quaternion(-qb.w(), -qb.x(), -qb.y(), -qb.z());
//                 dot = qa.dot(qb);
//             }
//             int rot_dist = DefaultCostMultiplier*smpl::angles::normalize_angle(2.0 * std::acos(dot));

//             int base_dist = bfs_3d_base->GetGoalHeuristic(state_id);
//             int arm_dist  = 0;
//             //if(arm_dist > 10000){
//                 //arm_dist = m_retract_arm_heur->GetGoalHeuristic(state_id);
//                 //rot_dist = 0.0;
//             //} else {
//                 arm_dist = bfs_3d->GetGoalHeuristic(state_id);
//             //}

//             //int heuristic = base_coeff*base_dist + arm_coeff*arm_dist + rot_coeff*rot_dist;
//             int heuristic = arm_coeff*arm_dist + rot_coeff*rot_dist;
//             //ROS_ERROR("%d + %d + %d = %d", int(base_coeff*base_dist), int(arm_coeff*arm_dist), int(rot_coeff*rot_dist), heuristic);
//             return heuristic;
//         }

//         double base_coeff=0.05;
//         double arm_coeff=1;
//         double rot_coeff=2;

//         std::shared_ptr<RetractArmHeuristic> m_retract_arm_heur;
//         smpl::PoseProjectionExtension* pose_ext = nullptr;
//     };

//     struct BaseRotHeuristic : public BfsHeuristic {
//         bool init(std::shared_ptr<smpl::Bfs3DBaseHeuristic> _bfs_3d_base,
//                 std::shared_ptr<smpl::Bfs3DHeuristic> _bfs_3d,
//                 std::shared_ptr<RetractArmHeuristic> _retract_arm,
//                 double _orientation){
//             if(!BfsHeuristic::init(_bfs_3d_base, _bfs_3d))
//                 return false;
//             m_retract_arm_heur = _retract_arm;
//             orientation = _orientation;
//             return true;
//         }

//         int GetGoalHeuristic(int state_id){
//             if(state_id == 0)
//                 return 0;
// 	    ROS_ERROR("Dynamic casting");
//             auto robot_state = (dynamic_cast<smpl::ManipLattice*>(
//                         bfs_3d->planningSpace()))->extractState(state_id);
//             int yaw_dist = DefaultCostMultiplier*smpl::angles::shortest_angle_dist(robot_state[2], orientation);

//             int base_dist = bfs_3d_base->GetGoalHeuristic(state_id);
//             double arm_fold_heur = 0.0;
//             if(base_dist > 10000)
//                 arm_fold_heur = m_retract_arm_heur->GetGoalHeuristic(state_id);

//             int heuristic = base_dist + orientation_coeff*yaw_dist + arm_fold_coeff*arm_fold_heur;
//             //ROS_ERROR("%d + %d + %d = %d", int(base_dist), int(orientation_coeff*yaw_dist), int(arm_fold_coeff*arm_fold_heur), heuristic);
//             return heuristic;
//         }

//         std::shared_ptr<RetractArmHeuristic> m_retract_arm_heur;
//         double orientation = 0.0;
//         double orientation_coeff = 5.0;
//         double arm_fold_coeff = 1.0;
//     };


// /*
//     auto bfs_2d = std::make_unique<smpl::Bfs2DHeuristic>();
//     bfs_2d->setCostPerCell(params.cost_per_cell);
//     bfs_2d->setInflationRadius(params.inflation_radius_2d);
//     if (!bfs_2d->init(pspace, grid)) {
//         ROS_ERROR("Could not initialize Bfs2Dheuristic.");
//         return false;
//     }
// */

//     auto bfs_3d = std::make_shared<smpl::Bfs3DHeuristic>();
//     bfs_3d->setCostPerCell(params.cost_per_cell);
//     bfs_3d->setInflationRadius(params.inflation_radius_3d);
//     if (!bfs_3d->init(pspace, grid)) {
//         ROS_ERROR("Could not initialize Bfs3Dheuristic.");
//         return false;
//     }

//     auto bfs_3d_base = std::make_shared<smpl::Bfs3DBaseHeuristic>();
//     bfs_3d_base->setCostPerCell(params.cost_per_cell);
//     bfs_3d_base->setInflationRadius(params.inflation_radius_2d);
//     if (!bfs_3d_base->init(pspace, grid, 4)) {
//         ROS_ERROR("Could not initialize Bfs3DBaseHeuristic");
//         return false;
//     }

//     auto retract_arm = std::make_shared<RetractArmHeuristic>();
//     if(!retract_arm->init(bfs_3d_base, bfs_3d)){
//         ROS_ERROR("Could not initialize RetractArmHeuristic initialize");
//         return false;
//     }


//     //Compute a feasible base location.
//     std::vector<int> base_x, base_y;
//     heurs.clear();

//     {
//         auto anchor = std::make_unique<AnchorHeuristic>();
//         anchor->init( bfs_3d_base, bfs_3d );
//         heurs.push_back(std::move(anchor));
//     }
//     /*
//     {
//         auto inad = std::make_unique<EndEffHeuristic>();
//         inad->init( bfs_3d_base, bfs_3d );
//         heurs.push_back(std::move(inad));
//     }*/
//     {
//         auto inad = std::make_unique<ImprovedEndEffHeuristic>();
//         if(!inad->init( bfs_3d_base, bfs_3d, retract_arm )){
//             ROS_ERROR("Could not initialize ImprovedEndEffHeuristic.");
//             return false;
//         }
//         heurs.push_back(std::move(inad));
//     }
//     /*
//     {
//         auto h = std::make_unique<smpl::Bfs3DHeuristic>();
//         h->setCostPerCell(params.cost_per_cell);
//         h->setInflationRadius(params.inflation_radius);
//         if (!h->init(pspace, grid)) {
//             ROS_ERROR("Could not initialize heuristic.");
//             return false;
//         }
//         heurs.push_back(std::move(h));
//     }
//     */

//     int num_rot_heurs = 0;
//     for(int i=0; i<num_rot_heurs; i++){
//         auto h = std::make_unique<BaseRotHeuristic>();
//         if (!h->init(bfs_3d_base, bfs_3d, retract_arm, 6.28/num_rot_heurs*i)) {
//             ROS_ERROR("Could not initialize BaseRotheuristic.");
//             return false;
//         }
//         heurs.push_back(std::move(h));
//     }
//     {
//         auto h = std::make_unique<smpl::EuclidDiffHeuristic>();
//         if (!h->init(pspace)) {
//             ROS_ERROR("Could not initialize heuristic.");
//             return false;
//         }
//         //heurs.push_back(std::move(h));
//     }
//     /*
//     {
//         auto h = std::make_unique<smpl::BfsFullbodyHeuristic>();
//         h->setCostPerCell(params.cost_per_cell);
//         h->setInflationRadius(params.inflation_radius);
//         if (!h->init(pspace, grid)) {
//             ROS_ERROR("Could not initialize heuristic.");
//             return false;
//         }
//         //SV_SHOW_INFO(h->get2DMapVisualization());
//         //heurs.push_back(std::move(h));
//     }
//     */

//     for (auto& entry : heurs) {
//         pspace->insertHeuristic(entry.get());
//     }
//     return true;
// }

struct ExperimentConfig 
{
    // plan params
    std::string planner_id;     
    double epsilon; 
    double allowed_planning_time;         
    std::string scenario;
    std::string exp_id;

    // plan res params
    bool use_post_processing;     
    bool use_visualization; 
    bool use_logging;    
    std::string log_filename;         
}; 

static 
void ReadExperimentConfig(const ros::NodeHandle& nh, ExperimentConfig& cfg)
{
    nh.param<std::string>("planner_id", cfg.planner_id, "arastar.bfs.manip"); 
    nh.param<double>("epsilon", cfg.epsilon, 25.0);    
    nh.param<double>("allowed_planning_time", cfg.allowed_planning_time, 60.0);
    nh.param<std::string>("scenario", cfg.scenario, "multi_room");
    nh.param<std::string>("exp_id", cfg.exp_id, "0"); 

    nh.param<bool>("use_post_processing", cfg.use_post_processing, false); 
    nh.param<bool>("use_visualization", cfg.use_visualization, false);     
    nh.param<bool>("use_logging", cfg.use_logging, false);     

    if (cfg.use_logging) {
        nh.param<std::string>("log_filename", cfg.log_filename, ""); 
    }
}

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

int main(int argc, char** argv){
    ros::init(argc, argv, "walker_planner");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    ROS_INFO("Initialize visualizer");
    smpl::VisualizerROS visualizer(nh, 100);
    smpl::viz::set_visualizer(&visualizer);    

    // Let publishers set up
    ros::Duration(1.0).sleep();    

    //////////////////////
    //  Read ROS Params // 
    //////////////////////

    ExperimentConfig exp_config;
    ReadExperimentConfig(ph, exp_config);     

    /////////////////
    // Robot Model //
    /////////////////

    ROS_INFO("Load common parameters");
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

    // Reads planning_joints, frames.
    RobotModelConfig robot_config;
    if (!ReadRobotModelConfig(ros::NodeHandle("~robot_model"), robot_config)) {
        ROS_ERROR("Failed to read robot model config from param server");
        return 1;
    }

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

    ROS_INFO("collision model loaded");
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

    // Read in collision objects
    // auto map_config = getMultiRoomMapConfig(ph);
    // std::vector<moveit_msgs::CollisionObject> tmp;
    // auto objects = GetMultiRoomMapCollisionCubes(grid.getReferenceFrame(), map_config, tmp);
    // for (auto& object : objects) {
    //     scene.ProcessCollisionObjectMsg(object);
    // }    

    scene.SetCollisionSpace(&cc);

    auto rm = SetupRobotModel(robot_description, robot_config);
    if (!rm) {
        ROS_ERROR("Failed to set up Robot Model");
        return 1;
    }

    rm->printRobotModelInformation();

    // Read in start/goal/critical state from file and update the scene
    moveit_msgs::RobotState start_state; 
    if (!ReadInitialConfiguration(ph, start_state)) {
        ROS_ERROR("Failed to get initial configuration");
        return 1;
    }

    // MIAStar isn't bidirectional, but need to read in goal state for JD calcs
    moveit_msgs::RobotState goal_state;    
    if (!ReadJointStateGoalConfiguration(ph, goal_state)) {
        ROS_ERROR("Failed to get initial configuration");
        return 1;       
    }    

    std::vector<double> goal_state_smpl(goal_state.joint_state.position);    

    std::vector<smpl::RobotState> critical_states; 
    if (!ReadCriticalStateConfigurations(ph, critical_states)) {
        ROS_ERROR("Failed to get critical_states");
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
    SetReferenceState(rm.get(), GetVariablePositions(&reference_state));    

    // Set reference state in the collision model...
    if (!scene.SetRobotState(start_state)) {
        ROS_ERROR("Failed to set start state on Collision Space Scene");
        return 1;
    }    

    // cc.setPadding(0.02);
    cc.setWorldToModelTransform(Eigen::Affine3d::Identity());

    // Vis scene
    SV_SHOW_INFO(cc.getCollisionRobotVisualization());
    SV_SHOW_INFO(cc.getCollisionWorldVisualization());
    SV_SHOW_INFO(grid.getOccupiedVoxelsVisualization());        
    SV_SHOW_INFO(grid.getBoundingBoxVisualization());                            

    // Vis problem configs
    std::vector<double> start_joint_state(start_state.joint_state.position.begin(), 
            start_state.joint_state.position.end());       
    auto start_state_markers = cc.getCollisionRobotVisualization(start_joint_state);
    for (auto& m : start_state_markers.markers) {
        m.ns = "start_config"; 
    }        

    std::vector<double> goal_joint_state(goal_state.joint_state.position.begin(), 
            goal_state.joint_state.position.end()); 
    auto goal_state_markers = cc.getCollisionRobotVisualization(goal_joint_state);

    for (auto& m : goal_state_markers.markers) {
        m.ns = "goal_config"; 
    }            

    std::string ns_crit = "critical_config";
    std::vector<float> rgba = { 1.0, 0.0, 0.0, 1.0 };         
    auto crit_markers = MakePathVisualization(&cc, critical_states, rgba, ns_crit);     
    
    SV_SHOW_INFO(start_state_markers);    
    SV_SHOW_INFO(goal_state_markers);    
    SV_SHOW_INFO(crit_markers);                     

    ///////////////////
    // Planner Setup //
    ///////////////////    

    PlannerConfig planning_config;
    if (!ReadPlannerConfig(ros::NodeHandle("~planning"), planning_config)){
        ROS_ERROR("Failed to read planner config");
        return 1;
    }

    // smpl::PlannerInterface planner(rm.get(), &cc, &grid); 
    smpl::CriticalPlannerInterface planner(rm.get(), &cc, &grid);     
    planner.setCriticalStates(critical_states); 

    // smpl::PlannerInterface planner(rm.get(), &cc, &grid); 
    smpl::PlanningParams params;
    params.addParam("discretization", planning_config.discretization);
    params.addParam("mprim_filename", planning_config.mprim_filename);
    params.addParam("use_xyz_snap_mprim", planning_config.use_xyz_snap_mprim);
    params.addParam("use_rpy_snap_mprim", planning_config.use_rpy_snap_mprim);
    params.addParam("use_xyzrpy_snap_mprim", planning_config.use_xyzrpy_snap_mprim);
    params.addParam("use_short_dist_mprims", planning_config.use_short_dist_mprims);
    params.addParam("xyz_snap_dist_thresh", planning_config.xyz_snap_dist_thresh);
    params.addParam("rpy_snap_dist_thresh", planning_config.rpy_snap_dist_thresh);
    params.addParam("xyzrpy_snap_dist_thresh", planning_config.xyzrpy_snap_dist_thresh);
    params.addParam("short_dist_mprims_thresh", planning_config.short_dist_mprims_thresh);
    params.addParam("search_mode", false);
    params.addParam("allow_partial_solutions", false);
    params.addParam("target_epsilon", 1.0);
    params.addParam("delta_epsilon", 1.0);
    params.addParam("improve_solution", false);
    params.addParam("bound_expansions", true);
    params.addParam("repair_time", 1.0);
    params.addParam("bfs_inflation_radius", 0.02);
    params.addParam("bfs_cost_per_cell", 100);
    params.addParam("epsilon", exp_config.epsilon);     
    // params.addParam("epsilon_anchor", exp_config.epsilon_anchor); 
    // params.addParam("epsilon_heur", exp_config.epsilon_heur);         

    // Post processing params
    if (exp_config.use_post_processing) {
        params.shortcut_path = true; 
        params.shortcut_type = smpl::ShortcutType::JOINT_SPACE; 
        params.interpolate_path = true;
    }    

    if (!planner.init(params)) {
        ROS_ERROR("Failed to initialize Planner Interface");
        return 1;
    }    

    //////////////
    // Planning //
    //////////////    

    std::vector<double> goal(6, 0);
    ph.param("goal/x", goal[0], 0.0);
    ph.param("goal/y", goal[1], 0.0);
    ph.param("goal/z", goal[2], 0.0);
    ph.param("goal/roll", goal[3], 0.0);
    ph.param("goal/pitch", goal[4], 0.0);
    ph.param("goal/yaw", goal[5], 0.0);

    moveit_msgs::MotionPlanRequest req;
    moveit_msgs::MotionPlanResponse res;

    req.allowed_planning_time = exp_config.allowed_planning_time; 

    req.goal_constraints.resize(1); 
    FillGoalConstraint(goal, planning_frame, req.goal_constraints[0]); 

    req.group_name = robot_config.group_name;
    req.max_acceleration_scaling_factor = 1.0;
    req.max_velocity_scaling_factor = 1.0;
    req.num_planning_attempts = 1;

    req.planner_id = exp_config.planner_id; 

    req.start_state = start_state;

    // Execute planner
    ROS_INFO("Calling solve...");
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.robot_state = start_state;

    bool plan_success = planner.solve(planning_scene, req, res, goal_state_smpl); 
    // bool plan_success = planner.solve(planning_scene, req, res);     

    if (!plan_success) { 
        ROS_ERROR("Failed to plan.");
    }    

    ///////////////////////////////////
    // Visualizations and Statistics //
    ///////////////////////////////////

    auto planning_stats = planner.getPlannerStats();

    ROS_INFO("Planning statistics");
    for (auto& entry : planning_stats) {
        ROS_INFO("    %s: %0.3f", entry.first.c_str(), entry.second);
    }

    ROS_INFO("animate: %d, logging: %d", 
        exp_config.use_visualization, exp_config.use_logging); 

    if (exp_config.use_visualization) {
        ROS_INFO("Animate path");
        size_t pidx = 0;
        while (ros::ok()) {
            auto& point = res.trajectory.joint_trajectory.points[pidx];
            auto markers = cc.getCollisionRobotVisualization(point.positions);
            for (auto& m : markers.markers) {
                m.ns = "path_animation";
            }
            SV_SHOW_INFO(markers);
            SV_SHOW_INFO(cc.getCollisionRobotVisualization());        
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
            pidx++;
            pidx %= res.trajectory.joint_trajectory.points.size();
        }
    }


    ROS_ERROR("Use logging: %d, Write results to: %s", 
        exp_config.use_logging, exp_config.log_filename.c_str());    
    if (exp_config.use_logging) {    


        smpl::Writer w(exp_config.log_filename); 

        std::string entry = "-1, -1, -1, -1, -1, -1, -1\n"; // fail case
        if (plan_success) {
            auto planning_stats = planner.getPlannerStats();            
            auto time = std::to_string(planning_stats["initial solution planning time"]); 
            auto expansions = std::to_string(planning_stats["expansions"]); 
            auto cost_planner = std::to_string(planning_stats["planner solution cost"]); 
            auto cost_pi = std::to_string(planning_stats["solution cost"]); 
            auto time_processed = std::to_string(planning_stats["processed time"]); 
            auto cost_processed = std::to_string(planning_stats["processed solution cost"]); 

            entry = time + ", " + \
            expansions + ", " + \
            cost_planner + ", " + \
            cost_pi + ", " + \
            time_processed + ", " + \
            cost_processed + ", " + \
            exp_config.exp_id + "\n"; 
        }
        w.Write(entry); 
    }        

    return !plan_success;    

    /////////////// Shivams stuff 

 //    auto resolutions = getResolutions( rm.get(), planning_config );
 //    auto action_space = std::make_unique<smpl::ManipLatticeActionSpace>();
 //    auto space = std::make_unique<smpl::ManipLattice>();

 //    if (!space->init( rm.get(), &cc, resolutions, action_space.get() )) {
 //        SMPL_ERROR("Failed to initialize Manip Lattice");
 //        return 1;
 //    }

 //    if (!action_space->init(space.get())) {
 //        SMPL_ERROR("Failed to initialize Manip Lattice Action Space");
 //        return 1;
 //    }

 //    if(!action_space->load(planning_config.mprim_filename))
 //        return 1;

 //    space->setVisualizationFrameId(grid_ptr->getReferenceFrame());

 //    using MotionPrimitive = smpl::MotionPrimitive;
 //    action_space->useMultipleIkSolutions(planning_config.use_multiple_ik_solutions);
 //    action_space->useAmp(MotionPrimitive::SNAP_TO_XYZ, planning_config.use_xyz_snap_mprim);
 //    action_space->useAmp(MotionPrimitive::SNAP_TO_RPY, planning_config.use_rpy_snap_mprim);
 //    action_space->useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.use_xyzrpy_snap_mprim);
 //    action_space->useAmp(MotionPrimitive::SHORT_DISTANCE, planning_config.use_short_dist_mprims);
 //    action_space->ampThresh(MotionPrimitive::SNAP_TO_XYZ, planning_config.xyz_snap_dist_thresh);
 //    action_space->ampThresh(MotionPrimitive::SNAP_TO_RPY, planning_config.rpy_snap_dist_thresh);
 //    action_space->ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, planning_config.xyzrpy_snap_dist_thresh);
 //    action_space->ampThresh(MotionPrimitive::SHORT_DISTANCE, planning_config.short_dist_mprims_thresh);

 //    /////////////
 //    //Planning///
 //    /////////////


 //    std::vector<std::unique_ptr<smpl::RobotHeuristic>> robot_heurs;

 //    if(!constructHeuristics( robot_heurs, space.get(), grid_ptr.get(), rm.get(), planning_config )){
 //        ROS_ERROR("Could not construct heuristics.");
 //        return 0;
 //    }

 //    ROS_ERROR("Num heurs: %d", robot_heurs.size());
 //    assert(robot_heurs[0] != nullptr);

 //    std::vector<Heuristic*> heurs;

 //    for(int i=0; i < robot_heurs.size(); i++)
 //        heurs.push_back(robot_heurs[i].get());

 //    Heuristic* anchor_heur = heurs[0];
 //    std::vector<Heuristic*> inad_heurs( heurs.begin() + 1, heurs.end() );

 //    using MotionPlanner = MPlanner::MotionPlanner<MHAPlanner, smpl::ManipLattice>;
 //    auto search_ptr = std::make_unique<MHAPlanner>(
 //            space.get(), anchor_heur, inad_heurs.data(), inad_heurs.size());

 //    ROS_ERROR("%f", planning_config.cost_per_cell);
 //    ROS_ERROR("%f", planning_config.inflation_radius_2d);
 //    ROS_ERROR("%f", planning_config.inflation_radius_3d);
 //    ROS_ERROR("%f", planning_config.eps);
 //    ROS_ERROR("%f", planning_config.eps_mha);

 //    const int max_planning_time = planning_config.planning_time;
 //    const double eps = planning_config.eps;
 //    const double eps_mha = planning_config.eps_mha;
 //    MPlanner::PlannerParams planner_params = { max_planning_time, eps, eps_mha, false };

 //    auto mplanner = std::make_unique<MotionPlanner>();
 //    mplanner->init(search_ptr.get(), space.get(), heurs, planner_params);

 //    MotionPlannerROS< Callbacks<>, ReadExperimentFromParamServer, MotionPlanner >
 //            mplanner_ros(ph, rm.get(), scene_ptr.get(), mplanner.get(), grid_ptr.get());

 //    ExecutionStatus status = ExecutionStatus::WAITING;
 //    //while(status == ExecutionStatus::WAITING) {
 //    std::string file_prefix = "paths/solution_path";
 //    std::ofstream stats_file;
 //    PlanningEpisode ep = planning_config.start_planning_episode;
 //    auto path_pub = nh.advertise<walker_planner::Path1>("Robot_path", 1000);

 //    std::vector<smpl::RobotState> plan;
 //    while(ep <= planning_config.end_planning_episode ){
 //        loop_rate.sleep();
 //        std::string file_suffix = std::to_string(ep) + ".txt";
 //        space->clearStats();

	// int done;
 //        ros::param::get("/walker_planner_done", done);
 //        if(done){
 //            publish_path(plan, path_pub);
	//     continue;
	// }
 //        status = mplanner_ros.execute(ep);
 //    //}
 //        if(status == ExecutionStatus::SUCCESS){
 //            ROS_INFO("----------------");
 //            ROS_INFO("Planning Time: %f", mplanner_ros.getPlan(ep).planning_time);
 //            ROS_INFO("----------------");
 //            plan = mplanner_ros.getPlan(ep).robot_states;

 //            // Write to file.
 //            stats_file.open("planning_stats.txt", std::ios::app);
 //            auto plan_stats = mplanner_ros.getPlan(ep);
 //            stats_file<<std::to_string(ep)<<" "<<max_planning_time<<" ";
 //            stats_file<<plan_stats.planning_time << " " << plan_stats.num_expansions << " " << plan_stats.cost<<" ";
 //            stats_file<<plan_stats.ik_computations<<" "<<plan_stats.ik_evaluations<<" "<<plan_stats.ik_valid<<" ";
 //            stats_file<<std::to_string(eps)<<" "<<std::to_string(eps_mha)<<"\n";
 //            stats_file.close();

 //            std::string file_name = file_prefix + file_suffix;
 //            std::string header = "Solution Path";
 //            SMPL_ERROR("%s", file_name.c_str());
 //            writePath(file_name, header , plan);
 //            ///////////////////
 //            //Compute Features
 //            //////////////////
 //            /*
 //            std::vector<moveit_msgs::CollisionObject> landmarks;
 //            auto map_config = getMultiRoomMapConfig(ph);
 //            auto objects = GetMultiRoomMapCollisionCubes(grid_ptr->getReferenceFrame(), map_config, landmarks );
 //            ROS_ERROR("Landmarks: %d", landmarks.size());
 //            auto features = computePlanFeatures(plan_stats,
 //                    dynamic_cast<BfsHeuristic*>(anchor_heur)->bfs_3d_base.get(),
 //                    landmarks[0]);
 //            ROS_ERROR("Features size: %d", features.x_rel_door.size());
 //            for(int i=0; i<features.x_rel_door.size(); i++){
 //                ROS_ERROR("%f", features.x_rel_door[i]);
 //                ROS_ERROR("%f", features.y_rel_door[i]);
 //                ROS_ERROR("%d", features.base_path_through_door[i]);
 //            }
 //            */


 //            visualization_msgs::MarkerArray whole_path;
 //            std::vector<visualization_msgs::Marker> m_all;

 //            ///*
 //            int idx = 0;
 //            for( int pidx=0; pidx<plan.size(); pidx++ ){
 //                auto& state = plan[pidx];
 //                auto markers = cc.getCollisionRobotVisualization(state);
 //                for (auto& m : markers.markers) {
 //                    m.ns = "path_animation";
 //                    m.id = idx;
 //                    idx++;
 //                    whole_path.markers.push_back(m);
 //                }
 //                visualizer.visualize(smpl::visual::Level::Info, markers);
 //                std::this_thread::sleep_for(std::chrono::milliseconds(20));
 //            }
 //            //*/
	//     publish_path(plan, path_pub);
 //            ros::param::set("/walker_planner_done", 1);
 //        }

 //        if(status != ExecutionStatus::WAITING){
 //            status = ExecutionStatus::WAITING;
 //            ep++;
 //        }
 //        ros::spinOnce();
 //    }

}
