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
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ros/ros.h>
#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <smpl/angles.h>
#include <smpl/debug/visualizer_ros.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/planning_params.h>
#include <smpl_ompl_interface/ompl_interface.h>
#include <urdf_parser/urdf_parser.h>

#include "critical_roadmap/critical_prm.h"
#include "critical_roadmap/critical_prm_constructor.h"
#include "config/planner_config.h"
#include "config/collision_space_scene.h"
#include "config/get_collision_objects.h"

static
bool ReadPR2RobotStatesFromFile(
    const std::string& traj_filename, 
    std::vector<smpl::RobotState>& traj)
{
    traj.clear(); 
    std::ifstream f(traj_filename); 
    std::string line; 
    while (std::getline(f, line)) {

        std::istringstream iss(line);
        std::string line_stream; 

        smpl::RobotState row; 
        while (std::getline(iss, line_stream, ',')) {
            row.push_back(std::stod(line_stream)); 
        }

        traj.push_back(row); 
    }
    return true; 
}

static
bool ReadCriticalScoresFromFile(
    const std::string& filename, 
    std::vector<int>& v_int)
{
    v_int.clear(); 
    std::ifstream f(filename); 
    std::string line; 
    while (std::getline(f, line)) {

        std::istringstream iss(line);
        std::string line_stream; 

        std::getline(iss, line_stream); 

        v_int.push_back(std::stoi(line_stream)); 
    }
    
    return true;     
}

static
auto MakeTopKCriticalPathVisualization(
    smpl::CollisionChecker* cc, 
    int& top_k, 
    const std::vector<smpl::RobotState>& path,
    const std::vector<float>& rgb_crit,
    const std::vector<float>& rgb_noncrit,     
    const std::vector<double>& opacity,     
    const std::string& ns_crit, 
    const std::string& ns_noncrit)
    -> std::pair< std::vector<smpl::visual::Marker>, std::vector<smpl::visual::Marker> >
{
    std::vector<smpl::visual::Marker> ma_crit;
    std::vector<smpl::visual::Marker> ma_noncrit;    

    auto cinc = 1.0f / float(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        auto markers = cc->getCollisionModelVisualization(path[i]);

        for (auto& marker : markers) {
            if (i < top_k) {
                marker.color = smpl::visual::Color{ rgb_crit[0], rgb_crit[1], 
                    rgb_crit[2], 1.0 };
            } else {
                marker.color = smpl::visual::Color{ rgb_noncrit[0], rgb_noncrit[1],
                    rgb_noncrit[2], (float)opacity[i] }; 
            }
        }

        for (auto& m : markers) {
            if (i < top_k) {
                ma_crit.push_back(std::move(m));
            } else {
                ma_noncrit.push_back(std::move(m)); 
            }
        }
    }

    for (size_t i = 0; i < ma_crit.size(); ++i) {
        auto& marker = ma_crit[i];
        marker.ns = ns_crit; 
        marker.id = i;
    }

    for (size_t i = 0; i < ma_noncrit.size(); ++i) {
        auto& marker = ma_noncrit[i];
        marker.ns = ns_noncrit; 
        marker.id = i;
    }    

    return std::make_pair<
        std::vector<smpl::visual::Marker>,
        std::vector<smpl::visual::Marker> >(std::move(ma_crit), std::move(ma_noncrit));   
}

auto ConstructStateSpace(
    const urdf::ModelInterface& urdf,
    const std::vector<std::string>& planning_joints)
    -> ompl::base::StateSpacePtr
{
    auto* concrete_space = new ompl::base::CompoundStateSpace;

    for (auto& joint_name : planning_joints) {
        auto joint = urdf.getJoint(joint_name);
        switch (joint->type) {
        case urdf::Joint::UNKNOWN:
            return NULL;
        case urdf::Joint::FIXED:
            break;
        case urdf::Joint::PRISMATIC:
        case urdf::Joint::REVOLUTE:
        {
            auto* subspace = new ompl::base::RealVectorStateSpace(1);
            if (joint->safety) {
                subspace->setBounds(joint->safety->soft_lower_limit, joint->safety->soft_upper_limit);
            } else if (joint->limits) {
                if(joint_name == "x" )
                    subspace->setBounds(0, 20);
                else if(joint_name == "y")
                    subspace->setBounds(0, 15);
                else
                subspace->setBounds(joint->limits->lower, std::min(joint->limits->upper, 20.0));
            } else {
                subspace->setBounds(-1.0, 1.0);
            }

            ompl::base::StateSpacePtr subspace_ptr(subspace);
            concrete_space->addSubspace(subspace_ptr, 1.0);
            break;
        }
        case urdf::Joint::CONTINUOUS:
        {
            concrete_space->addSubspace(
                    ompl::base::StateSpacePtr(new ompl::base::SO2StateSpace),
                    1.0);
            break;
        }
        case urdf::Joint::PLANAR:
        {
            auto* subspace = new ompl::base::SE2StateSpace;
            ompl::base::RealVectorBounds bounds(2);
            bounds.setLow(-1.0);
            bounds.setHigh(1.0);
            subspace->setBounds(bounds);
            concrete_space->addSubspace(ompl::base::StateSpacePtr(subspace), 1.0);
            break;
        }
        case urdf::Joint::FLOATING:
        {
            auto* subspace = new ompl::base::SE3StateSpace();
            ompl::base::RealVectorBounds bounds(3);
            bounds.setLow(-1.0);
            bounds.setHigh(1.0);
            subspace->setBounds(bounds);
            concrete_space->addSubspace(ompl::base::StateSpacePtr(subspace), 1.0);
            break;
        }
        default:
            ROS_WARN("Skip unrecognized joint type");
            break;
        }
    }

    return ompl::base::StateSpacePtr(concrete_space);
}

struct ProjectionEvaluatorFK : public ompl::base::ProjectionEvaluator
{
    smpl::KDLRobotModel* model = NULL;
    const ompl::base::StateSpace* state_space = NULL;

    using Base = ompl::base::ProjectionEvaluator;

    ProjectionEvaluatorFK(const ompl::base::StateSpace* space);
    ProjectionEvaluatorFK(const ompl::base::StateSpacePtr& space);

    auto getDimension() const -> unsigned int override;
    void project(
            const ompl::base::State* state,
            ompl::base::EuclideanProjection& projection) const override;
    void setCellSizes(const std::vector<double>& cell_sizes) override;
    void defaultCellSizes() override;
    void setup() override;
    void printSettings(std::ostream& out = std::cout) const override;
    void printProjection(
            const ompl::base::EuclideanProjection& projection,
            std::ostream& out = std::cout) const override;
};

ProjectionEvaluatorFK::ProjectionEvaluatorFK(
    const ompl::base::StateSpace* space)
:
    Base(space)
{
    state_space = space;
}

ProjectionEvaluatorFK::ProjectionEvaluatorFK(
    const ompl::base::StateSpacePtr& space)
:
    Base(space)
{
    state_space = space.get();
}

auto ProjectionEvaluatorFK::getDimension() const -> unsigned int
{
    return 6;
}

void ProjectionEvaluatorFK::project(
        const ompl::base::State* state,
        ompl::base::EuclideanProjection& projection) const
{
    auto values = smpl::MakeStateSMPL(this->state_space, state);
    auto pose = model->computeFK(values);
    projection.resize(getDimension(), 0.0);
    projection[0] = pose.translation().x();
    projection[1] = pose.translation().y();
    projection[2] = pose.translation().z();
    smpl::angles::get_euler_zyx(
            pose.rotation(), projection[3], projection[4], projection[5]);
}

void ProjectionEvaluatorFK::setCellSizes(const std::vector<double>& cell_sizes)
{
    this->Base::setCellSizes(cell_sizes);
}

void ProjectionEvaluatorFK::defaultCellSizes()
{
    this->Base::defaultCellSizes();
}

void ProjectionEvaluatorFK::setup()
{
    this->Base::setup();
}

void ProjectionEvaluatorFK::printSettings(std::ostream& out) const
{
    this->Base::printSettings(out);
}

void ProjectionEvaluatorFK::printProjection(
    const ompl::base::EuclideanProjection& projection, std::ostream& out) const
{
    this->Base::printProjection(projection, out);
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

    /////////////////////
    // Visualizations  //
    /////////////////////

    // To Do: Move these to rosparams
    std::string f_vertices = "/home/allen/catkin_ws/src/walker_planner/critical_roadmaps/test/crit_roadmap_vertices.csv"; 
    std::string f_scores = "/home/allen/catkin_ws/src/walker_planner/critical_roadmaps/test/crit_roadmap_scores.csv"; 

    std::vector<smpl::RobotState> crit_states; 
    ReadPR2RobotStatesFromFile(f_vertices, crit_states);     

    std::vector<int> crit_scores; 
    ReadCriticalScoresFromFile(f_scores, crit_scores);    

    // Compute normalized scores 
    std::vector<double> crit_scores_norm(crit_scores.begin(), crit_scores.end()); 
    double max_score = *(std::max_element(crit_scores_norm.begin(), crit_scores_norm.end()));     

    std::transform(crit_scores_norm.begin(), crit_scores_norm.end(), 
        crit_scores_norm.begin(), [max_score](double& n){return n/max_score; });     

    // VIsualize critical states with relative opacity 
    std::string ns_crit = "critical_config"; 
    std::string ns_noncrit = "noncritical_config";         
    std::vector<float> rgb_crit = { 1.0, 0.0, 0.0};     
    std::vector<float> rgb_noncrit = { 0.0, 1.0, 0.0};           

    int top_k;    
    ph.param("top_k", top_k, 10); 

    ROS_INFO("Visualizing top %d critical states", top_k); 

    auto crit_markers = MakeTopKCriticalPathVisualization(&cc, top_k, 
        crit_states, rgb_crit, rgb_noncrit, crit_scores_norm, ns_crit, ns_noncrit);  

    while (ros::ok()) {

        // Vis scene
        SV_SHOW_INFO(cc.getCollisionRobotVisualization());
        SV_SHOW_INFO(cc.getCollisionWorldVisualization());
        SV_SHOW_INFO(grid.getOccupiedVoxelsVisualization());        
        SV_SHOW_INFO(grid.getBoundingBoxVisualization());                        

        // Vis crit roadmap 
        SV_SHOW_INFO(crit_markers.first);
        SV_SHOW_INFO(crit_markers.second);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));        
    }
    
    return 0;
}
