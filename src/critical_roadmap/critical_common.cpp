#include "critical_roadmap/critical_common.h"

// standard includes
// #include <string>

// system includes
// #include <ros/ros.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>

#include <smpl/angles.h>
#include <smpl_ompl_interface/ompl_interface.h>

bool ReadCriticalConstructionConfig(
    const ros::NodeHandle &nh,
    smpl::ConstructionConfig &config)
{
    if (!nh.getParam("grow_time", config.grow_time)) {
        ROS_ERROR("Failed to read 'grow_time' from the param server");
        return false;
    } else {
        ROS_INFO("Grow time: %f", config.grow_time); 
    }

    if (!nh.getParam("expand_time", config.expand_time)) {
        ROS_ERROR("Failed to read param 'expand_time' from the param server");
        return false;
    } else {
        ROS_INFO("Expand time: %f", config.expand_time); 
    }        

    if (!nh.getParam("num_sampled_starts", config.num_sampled_starts)) {
        ROS_ERROR("Failed to read param 'num_sampled_starts' from the param server");
        return false;
    } else {
        ROS_INFO("Sampled starts: %d", config.num_sampled_starts); 
    }        

    if (!nh.getParam("num_sampled_goals_per_start", config.num_sampled_goals_per_start)) {
        ROS_ERROR("Failed to read param 'num_sampled_goals_per_start' from the param server");
        return false;
    } else {
        ROS_INFO("Goals per start: %d", config.num_sampled_goals_per_start); 
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

    return true;
}

auto ConstructWalkerStateSpaceWithDubinsBase(
    const urdf::ModelInterface& urdf,
    const std::vector<std::string>& planning_joints)
    -> ompl::base::StateSpacePtr
{
    auto* concrete_space = new ompl::base::CompoundStateSpace;

    // Note: We make dubins distance symmetrical for undirected graph
    auto dubins_subspace = new ompl::base::DubinsStateSpace(0.17, true); 
    auto subspace = ompl::base::StateSpacePtr(dubins_subspace); 

     ompl::base::RealVectorBounds bounds(2);
     bounds.setLow(0);
     bounds.high[0] = 20; 
     bounds.high[1] = 15;   

    subspace->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

    ROS_ERROR("Base as dubins state space"); 
    subspace->printSettings(std::cout); 
    // dubins_subspace->as<ompl::base::CompoundStateSpace>()->printSettings(); 

    concrete_space->addSubspace(subspace, 1.0); 

    for (auto& joint_name : planning_joints) {
        if (joint_name == "x" || joint_name == "y" || joint_name == "theta")
            continue; 

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
                else {
                    ROS_INFO("Arm joint: %s bounds", joint_name.c_str());
                    subspace->setBounds(joint->limits->lower, std::min(joint->limits->upper, 20.0));
                    subspace->printSettings(std::cout);                       
                }
            } else {
                subspace->setBounds(-M_PI, M_PI);
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