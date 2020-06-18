// standard includes
#include <vector>
#include <string>
#include <ostream> 

// system includes
#include <ros/ros.h>

// project includes
#include <ros/ros.h>
#include <urdf_parser/urdf_parser.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>

// project includes
#include "critical_roadmap/critical_prm_constructor.h"

bool ReadCriticalConstructionConfig(const ros::NodeHandle &nh,
    smpl::ConstructionConfig &config); 


auto ConstructWalkerStateSpaceWithDubinsBase(const urdf::ModelInterface& urdf,
    const std::vector<std::string>& planning_joints) -> ompl::base::StateSpacePtr;


auto ConstructStateSpace(const urdf::ModelInterface& urdf, 
	const std::vector<std::string>& planning_joints) -> ompl::base::StateSpacePtr; 


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


