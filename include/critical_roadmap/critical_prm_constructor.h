#ifndef CRITICAL_PRM_CONSTRUCTOR_H
#define CRITICAL_PRM_CONSTRUCTOR_H

// standard includes
#include <memory>
#include <vector>
#include <utility>
#include <unordered_map> 

// system includes
#include <smpl/types.h> 

#include <Eigen/Dense>
#include <boost/functional/hash.hpp> 

#include <ompl/base/Planner.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>

// project includes
#include "critical_roadmap/critical_prm.h"
#include "utils/utils.h"

namespace smpl {

struct CriticalScoreComparator {
	bool operator()(const std::pair<RobotState, int>& p1, const std::pair<RobotState, int>& p2) const 
	{
		return p1.second > p2.second; 
	}
}; 

struct ConstructionConfig
{
	bool using_star_strategy;
	double grow_time; 
	double expand_time; 

	int num_sampled_starts; 
	int num_sampled_goals_per_start; 	

	bool save_roadmap;
	std::string save_dir; 
}; 

class CriticalPRMConstructor
{
public:
	CriticalPRMConstructor(const ompl::geometric::SimpleSetup& ss,
		ConstructionConfig& cfg); 

	auto constructPRM() -> const ompl::geometric::PRM::Graph; 
	void constructCriticalPRM(); // 
	bool runProblemInstance(ompl::base::State* start, ompl::base::State* goal);

private: // PRM member attributes	
	ConstructionConfig m_cfg; 

	ompl::base::ProblemDefinitionPtr m_pdef; 
	ompl::base::SpaceInformationPtr m_space; 
	ompl::base::StateSpacePtr m_state_space; 

	std::shared_ptr<ompl::geometric::CriticalPRM> m_prm_planner; 

	using CriticalScoreMap = std::unordered_map<RobotState, int, VectorHash<double> >; 
	CriticalScoreMap m_critical_score_map; 
}; 
} // namespace smpl 
#endif 




