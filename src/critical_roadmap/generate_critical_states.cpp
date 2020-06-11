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

    //SV_SHOW_INFO(grid.getDistanceFieldVisualization(0.2));

    //SV_SHOW_INFO(cc.getCollisionRobotVisualization());
    //SV_SHOW_INFO(cc.getCollisionWorldVisualization());
    //SV_SHOW_INFO(cc.getOccupiedVoxelsVisualization());

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

    auto state_space = ConstructStateSpace(*urdf, robot_config.planning_joints);

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

    ROS_INFO("Setup the query");

    // Initialize random start/goal, only for ss setup not used in crit prm construction
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(state_space);
    start.random(); 
    ss.setStartState(start); 

    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(state_space);
    goal.random(); 
    ss.setGoalState(goal); 

    // Initialize critical roadmap construction parameters 
    smpl::ConstructionConfig cfg; // To Do: Move these to rosparams
    cfg.using_star_strategy = false; 
    cfg.grow_time = 60.0; 
    cfg.expand_time = 1.0;     
    cfg.num_sampled_starts = 50; 
    cfg.num_sampled_goals_per_start = 200; 
    cfg.save_roadmap = true; 
    cfg.save_dir = "/home/allen/catkin_ws/src/walker_planner/critical_roadmaps/test"; 

    ss.setup(); 

    // Construct Critical PRM 
    // auto crit_ctor = smpl::CriticalPRMConstructor(ss, cfg);
    // crit_ctor.constructCriticalPRM(); 


    // Test loading prm 
    std::string f_grown_milestones = "/home/allen/catkin_ws/src/walker_planner/critical_roadmaps/test/grown_milestones.csv"; 
    auto prm = new ompl::geometric::CriticalPRM(ss.getSpaceInformation()); 
    prm->setProblemDefinition(ss.getProblemDefinition()); 
    prm->growRoadmapFromFile(f_grown_milestones); 
        ROS_ERROR("Initialized prm with %d vertices, %d edges", 
            prm->milestoneCount(), prm->edgeCount());     

    return 0; 


    // Vis 
    ROS_INFO("Visualizing scene"); 
    while (ros::ok()) {
        SV_SHOW_INFO(cc.getCollisionRobotVisualization());
        SV_SHOW_INFO(cc.getCollisionWorldVisualization());
        SV_SHOW_INFO(grid.getOccupiedVoxelsVisualization());        
        SV_SHOW_INFO(grid.getBoundingBoxVisualization());                        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));        
    }
    
    return 0;
}
