#include "config/planner_config.h"

void FillGoalConstraint(
    const std::vector<double>& pose,
    std::string frame_id,
    moveit_msgs::Constraints& goals,
    double xyz_tol,
    double rpy_tol)
{
    if (pose.size() < 6) {
        return;
    }

    goals.position_constraints.resize(1);
    goals.orientation_constraints.resize(1);
    goals.position_constraints[0].header.frame_id = frame_id;

    goals.position_constraints[0].constraint_region.primitives.resize(1);
    goals.position_constraints[0].constraint_region.primitive_poses.resize(1);
    goals.position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.x = pose[0];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.y = pose[1];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.z = pose[2];

//    goals.position_constraints[0].position.x = pose[0];
//    goals.position_constraints[0].position.y = pose[1];
//    goals.position_constraints[0].position.z = pose[2];

    Eigen::Quaterniond q;
    smpl::angles::from_euler_zyx(pose[5], pose[4], pose[3], q);
    tf::quaternionEigenToMsg(q, goals.orientation_constraints[0].orientation);

    geometry_msgs::Pose p;
    p.position = goals.position_constraints[0].constraint_region.primitive_poses[0].position;
    p.orientation = goals.orientation_constraints[0].orientation;
    leatherman::printPoseMsg(p, "Goal");

    /// set tolerances
    goals.position_constraints[0].constraint_region.primitives[0].dimensions.resize(3, xyz_tol);
    goals.orientation_constraints[0].absolute_x_axis_tolerance = rpy_tol;
    goals.orientation_constraints[0].absolute_y_axis_tolerance = rpy_tol;
    goals.orientation_constraints[0].absolute_z_axis_tolerance = rpy_tol;

    ROS_INFO("Done packing the goal constraints message.");
}

bool ReadInitialConfiguration(
    ros::NodeHandle& nh,
    moveit_msgs::RobotState& state){
    XmlRpc::XmlRpcValue xlist;

    // joint_state
    if (nh.hasParam("initial_configuration/joint_state")) {
        nh.getParam("initial_configuration/joint_state", xlist);

        if (xlist.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("initial_configuration/joint_state is not an array.");
        }

        if (xlist.size() > 0) {
            for (int i = 0; i < xlist.size(); ++i) {
                state.joint_state.name.push_back(std::string(xlist[i]["name"]));

                if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                    state.joint_state.position.push_back(double(xlist[i]["position"]));
                }
                else {
                    ROS_DEBUG("Doubles in the yaml file have to contain decimal points. (Convert '0' to '0.0')");
                    if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                        int pos = xlist[i]["position"];
                        state.joint_state.position.push_back(double(pos));
                    }
                }
            }
        }
    }
    else {
        ROS_WARN("initial_configuration/joint_state is not on the param server.");
    }

    // multi_dof_joint_state
    if (nh.hasParam("initial_configuration/multi_dof_joint_state")) {
        nh.getParam("initial_configuration/multi_dof_joint_state", xlist);

        if (xlist.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            if (xlist.size() != 0) {
                auto &multi_dof_joint_state = state.multi_dof_joint_state;
                multi_dof_joint_state.joint_names.resize(xlist.size());
                multi_dof_joint_state.transforms.resize(xlist.size());
                for (int i = 0; i < xlist.size(); ++i) {
                    multi_dof_joint_state.joint_names[i] = std::string(xlist[i]["joint_name"]);

                    Eigen::Quaterniond q;
                    smpl::angles::from_euler_zyx(
                            (double)xlist[i]["yaw"], (double)xlist[i]["pitch"], (double)xlist[i]["roll"], q);

                    geometry_msgs::Quaternion orientation;
                    tf::quaternionEigenToMsg(q, orientation);

                    multi_dof_joint_state.transforms[i].translation.x = xlist[i]["x"];
                    multi_dof_joint_state.transforms[i].translation.y = xlist[i]["y"];
                    multi_dof_joint_state.transforms[i].translation.z = xlist[i]["z"];
                    multi_dof_joint_state.transforms[i].rotation.w = orientation.w;
                    multi_dof_joint_state.transforms[i].rotation.x = orientation.x;
                    multi_dof_joint_state.transforms[i].rotation.y = orientation.y;
                    multi_dof_joint_state.transforms[i].rotation.z = orientation.z;
                }
            } else {
                ROS_WARN("initial_configuration/multi_dof_joint_state array is empty");
            }
        } else {
            ROS_WARN("initial_configuration/multi_dof_joint_state is not an array.");
        }
    }

    ROS_INFO("Read initial state containing %zu joints and %zu multi-dof joints", state.joint_state.name.size(), state.multi_dof_joint_state.joint_names.size());
    return true;
}

bool ReadJointStateGoalConfiguration(
    ros::NodeHandle& nh, 
    moveit_msgs::RobotState& state) 
{
    XmlRpc::XmlRpcValue xlist; 

    if (nh.hasParam("goal_configuration/joint_state")) {
        nh.getParam("goal_configuration/joint_state", xlist); 

        if (xlist.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("goal_configuration/joint_state is not an array.");            
        }

        if (xlist.size() > 0) {
            for (int i = 0; i < xlist.size(); ++i) {
                state.joint_state.name.push_back(std::string(xlist[i]["name"])); 
                if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeDouble) { 
                    state.joint_state.position.push_back(double(xlist[i]["position"]));                    
                } else {
                    ROS_DEBUG("Doubles in the yaml file have to contain decimal points"); 
                    if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                        int pos = xlist[i]["position"]; 
                        state.joint_state.position.push_back(double(pos)); 
                    }
                }
            }

        }
    } else {
        ROS_WARN("goal_configuration/joint_state is not on the param server."); 
        return false; 
    }
    return true; 
}

bool ReadPoseGoal(ros::NodeHandle& nh, std::vector<double>& goal)
{
    nh.param("goal/x", goal[0], 0.0);
    nh.param("goal/y", goal[1], 0.0);
    nh.param("goal/z", goal[2], 0.0);
    nh.param("goal/roll", goal[3], 0.0);
    nh.param("goal/pitch", goal[4], 0.0);
    nh.param("goal/yaw", goal[5], 0.0);
    return true; 
}

bool ReadCriticalStateConfigurations(
    ros::NodeHandle& nh, 
    std::vector<smpl::RobotState>& critical_states)
{
    critical_states.clear(); 
    XmlRpc::XmlRpcValue xlist; 
    if (nh.hasParam("critical_configurations")) {
        nh.getParam("critical_configurations", xlist); 
        if (xlist.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("additional_roots is not an array.");
            return false; 
        } else {
            ROS_ERROR("Got %zu additional_roots", xlist.size());
        }
        for (int i = 0; i < xlist.size(); ++i) {
            smpl::RobotState rs; 
            for (int j = 0; j < xlist[i].size(); ++j) {
                rs.push_back(double(xlist[i][j])); 
            }
            critical_states.push_back(rs); 
        }
    } else {
        ROS_ERROR("additional_roots is not on the param server"); 
        return false; 
    }
    return true;     
}

bool ReadRobotModelConfig(const ros::NodeHandle &nh, RobotModelConfig &config)
{
    if (!nh.getParam("group_name", config.group_name)) {
        ROS_ERROR("Failed to read 'group_name' from the param server");
        return false;
    }

    std::string planning_joint_list;
    if (!nh.getParam("planning_joints", planning_joint_list)) {
        ROS_ERROR("Failed to read 'planning_joints' from the param server");
        return false;
    }

    std::stringstream joint_name_stream(planning_joint_list);
    while (joint_name_stream.good() && !joint_name_stream.eof()) {
        std::string jname;
        joint_name_stream >> jname;
        if (jname.empty()) {
            continue;
        }
        config.planning_joints.push_back(jname);
    }

    // only required for generic kdl robot model?
    nh.getParam("kinematics_frame", config.kinematics_frame);
    nh.getParam("chain_tip_link", config.chain_tip_link);
    return true;
}

bool ReadPlannerConfig(const ros::NodeHandle &nh, PlannerConfig &config)
{
    if (!nh.getParam("discretization", config.discretization)) {
        ROS_ERROR("Failed to read 'discretization' from the param server");
        return false;
    }

    if (!nh.getParam("mprim_filename", config.mprim_filename)) {
        ROS_ERROR("Failed to read param 'mprim_filename' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyz_snap_mprim", config.use_xyz_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_rpy_snap_mprim", config.use_rpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_rpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyzrpy_snap_mprim", config.use_xyzrpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyzrpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_short_dist_mprims", config.use_short_dist_mprims)) {
        ROS_ERROR("Failed to read param 'use_short_dist_mprims' from the param server");
        return false;
    }

    if (!nh.getParam("xyz_snap_dist_thresh", config.xyz_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyz_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("rpy_snap_dist_thresh", config.rpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'rpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("xyzrpy_snap_dist_thresh", config.xyzrpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyzrpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("short_dist_mprims_thresh", config.short_dist_mprims_thresh)) {
        ROS_ERROR("Failed to read param 'short_dist_mprims_thresh' from the param server");
        return false;
    }
    if (!nh.getParam("cost_per_cell", config.cost_per_cell)) {
        ROS_ERROR("Failed to read param 'cost_per_cell' from the param server");
        return false;
    }

    if(!nh.getParam("inflation_radius_2d", config.inflation_radius_2d)){
        ROS_ERROR("Failed to read param 'inflation_radius_2d' from the param server");
        return false;
    }

    if(!nh.getParam("inflation_radius_3d", config.inflation_radius_3d)){
        ROS_ERROR("Failed to read param 'inflation_radius_3d' from the param server");
        return false;
    }

    if (!nh.getParam("planner_id", config.planner_id)) {
        ROS_ERROR("Failed to read param 'planner_id' from the param server");
        return false;
    }                

    if(!nh.getParam("epsilon_anchor", config.epsilon_anchor)){
        ROS_ERROR("Failed to read param 'eps' from the param server");
        return false;
    }

    if(!nh.getParam("epsilon_mha", config.epsilon_mha)){
        ROS_ERROR("Failed to read param 'eps_mha' from the param server");
        return false;
    }
    if(!nh.getParam("planning_time", config.planning_time)){
        ROS_ERROR("Failed to read param 'planning_time' from the param server");
        return false;
    }

    if (!nh.getParam("animate", config.use_visualization)) {
        ROS_ERROR("Failed to read param 'animate' from the param server");
        return false;
    }    

    if (!nh.getParam("logging", config.use_logging)) {
        ROS_ERROR("Failed to read param 'logging' from the param server");
        return false;
    }        

    if (config.use_logging) {
        if (!nh.getParam("log_filename", config.log_filename)) {
            ROS_ERROR("Failed to read param 'log_filename' from the param server");
            return false;
        }                
    }

    if(!nh.getParam("post_processing", config.use_post_processing)){
        ROS_ERROR("Failed to read param 'post_processing' from the param server");
        return false;
    }

    return true;
}


MultiRoomMapConfig getMultiRoomMapConfig(ros::NodeHandle nh){
    MultiRoomMapConfig config;
    nh.param("map/seed", config.seed, 1000);
    nh.param("map/x_max", config.x_max, 0.0);
    nh.param("map/y_max", config.y_max, 0.0);
    nh.param("map/h_max", config.h_max, 0.0);
    nh.param("map/door_width", config.door_width, 0.0);
    nh.param("map/alley_width", config.alley_width, 0.0);
    nh.param("map/n_tables", config.n_tables, 0);
    nh.param("map/min_table_len", config.min_table_len, 0.0);
    nh.param("map/max_table_len", config.max_table_len, 0.0);
    nh.param("map/table_height", config.table_height, 0.0);
    nh.param("map/n_objects", config.n_objects_per_table, 0);
 
    return config;
}

