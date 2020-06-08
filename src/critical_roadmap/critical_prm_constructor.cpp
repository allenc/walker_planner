#include "critical_roadmap/critical_prm_constructor.h"

// standard incldues
#include <random>
#include <memory>
#include <fstream> 
#include <algorithm>
#include <exception>

// system includes
#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/tools/config/MagicConstants.h> 
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <smpl/console/console.h> 
#include <smpl/utils/debugging_utils.h> 
#include <smpl/utils/logging_utils.h> 
#include <smpl_ompl_interface/ompl_interface.h>

#include <boost/thread.hpp>

// project includes
#include "critical_roadmap/critical_prm.h"
#include "utils/utils.h"

namespace smpl { 

static 
int GetRandomIntInRange(int min, int max) {
	std::random_device rd; 
	std::mt19937 mt(rd()); 
	std::uniform_int_distribution<>dist(min, max); 
	return dist(mt); 
}

static 
void WriteRoadmapToFile(
	std::string& prm_filename,	
	std::vector<smpl::RobotState>& prm_vertices)
{
	smpl::Writer w(prm_filename); 
	w.Clear(); 
	w.WriteRobotPath(prm_vertices);	
}

static
void WriteCriticalRoadmapToFile(
	std::string& critical_prm_save_dir,
	std::vector<smpl::RobotState>& critical_prm_vertices, 
	std::vector<int>& critical_prm_scores)
{
	std::string f_vertices; // Filename to store Critical PRM vertices 
	std::string f_critical_scores; // Filename to store Critical PRM scores

	f_vertices = critical_prm_save_dir + "/crit_roadmap_vertices.csv"; 
	f_critical_scores = critical_prm_save_dir + "/crit_roadmap_scores.csv"; 	

	WriteRoadmapToFile(f_vertices, critical_prm_vertices); 

	smpl::Writer w_s(f_critical_scores); 
	w_s.Clear(); 	
	w_s.WriteVector<int>(critical_prm_scores); 
}

static 
void PrintOMPLState(ompl::base::StateSpace* state_space, ompl::base::State* state) 
{
	auto rs_state = MakeStateSMPL(state_space, state); 
	PrintSMPLState(rs_state); 
}

CriticalPRMConstructor::CriticalPRMConstructor(
	const ompl::geometric::SimpleSetup& ss, 
	ConstructionConfig& cfg)
:
	m_cfg(cfg), 
	m_pdef(ss.getProblemDefinition()), 
	m_space(ss.getSpaceInformation()),
	m_state_space(ss.getStateSpace()), 
	m_prm_planner(std::make_shared<ompl::geometric::CriticalPRM>(
		ss.getSpaceInformation(), m_cfg.using_star_strategy)), 
	m_critical_score_map() 
{
	// Make sure that optimization objective is configured to stop at first solution
	if (!m_pdef->hasOptimizationObjective()) {
		auto opt_obj = std::make_shared<ompl::base::PathLengthOptimizationObjective>(m_space);
		if (!m_cfg.using_star_strategy) {
			opt_obj->setCostThreshold(opt_obj->infiniteCost());
		}
		m_pdef->setOptimizationObjective(opt_obj); 
	}		

	// Initialize critical prm planner 
	m_prm_planner->setProblemDefinition(m_pdef); 	

	auto mode = ompl::geometric::CriticalPRM::CONSTRUCT;  
	m_prm_planner->setMode(mode); 

	std::string milestones_filename = m_cfg.save_dir + "/grown_milestones.csv"; 
	m_prm_planner->setLoggingParameters(m_cfg.save_roadmap, milestones_filename);  
}

void CriticalPRMConstructor::constructCriticalPRM() 
{	
	// Construct initial uniform prm 
	auto prm_roadmap = constructPRM(); 

	// Sample start and goal from this prm 
	int num_roadmap_vertices = boost::num_vertices(prm_roadmap); 
	auto v_state_map = boost::get(ompl::geometric::PRM::vertex_state_t(), prm_roadmap); 	
	std::vector<bool> v_starts(num_roadmap_vertices, false); // tracks which starts have been used

	// Solve queries
	int sampled_starts = 0; 	
	while (sampled_starts < m_cfg.num_sampled_starts) 		
	{		
		auto v_start = GetRandomIntInRange(0, num_roadmap_vertices); 

		if (v_starts[v_start]) {
			continue; 
		}

		// Run shortest path query to all other roadmap vertices 
		int sampled_goals_per_start = 0; 
		while (sampled_goals_per_start < m_cfg.num_sampled_goals_per_start) 
		{
			SMPL_INFO("Sampled_starts %d/%d, sampled goals %d/%d", 
				sampled_starts, m_cfg.num_sampled_starts, 
				sampled_goals_per_start, m_cfg.num_sampled_goals_per_start); 

			auto v_goal = GetRandomIntInRange(0, num_roadmap_vertices); 

			if (v_goal == v_start || !v_state_map[v_start] || !v_state_map[v_goal]) {
				continue;
			}

			if (runProblemInstance(v_state_map[v_start], v_state_map[v_goal])) {
				++sampled_goals_per_start; 
			}
		}

		v_starts[v_start] = true; 
		++sampled_starts; 
	}

	// Write crit states to file sorted by betweeness centrality / crit score 
	if (m_cfg.save_roadmap) {
		std::vector< std::pair<RobotState, int> > sorted_critical_scores(
			m_critical_score_map.begin(), m_critical_score_map.end()); 

		std::sort(sorted_critical_scores.begin(), sorted_critical_scores.end(), 
			CriticalScoreComparator());

		std::vector<RobotState> critical_vertices; 
		std::vector<int> critical_scores; 

		for (auto p : sorted_critical_scores) {
			critical_vertices.push_back(p.first); 
			critical_scores.push_back(p.second); 			
		}

		WriteCriticalRoadmapToFile(m_cfg.save_dir, critical_vertices, 
			critical_scores); 
	}
}

auto CriticalPRMConstructor::constructPRM() -> const ompl::geometric::PRM::Graph
{
	SMPL_INFO("Growing PRM for %fs then further expanding for %fs", 
		m_cfg.grow_time, m_cfg.expand_time);

	m_prm_planner->growRoadmap(m_cfg.grow_time); 
	// To Do: Include expand step? 

	SMPL_INFO("Finished growing prm, got %d vertices and %d edges", 
		m_prm_planner->milestoneCount(), m_prm_planner->edgeCount()); 

	auto roadmap = m_prm_planner->getRoadmap(); 

	if (m_cfg.save_roadmap) {
   	    
   	    // Get PRM vertices as RobotState		
		auto v_state_map = boost::get(ompl::geometric::PRM::vertex_state_t(), 
			roadmap); 

		std::vector<smpl::RobotState> roadmap_vertices; 

		for (auto v : boost::make_iterator_range(boost::vertices(roadmap))) {
			auto rs_vertex = smpl::MakeStateSMPL(m_state_space.get(), v_state_map[v]); 
			roadmap_vertices.push_back(rs_vertex); 
		}

		std::string roadmap_filename = m_cfg.save_dir + "/all_roadmap_vertices.csv"; 
		WriteRoadmapToFile(roadmap_filename, roadmap_vertices); 
	}
	return roadmap; 
}

ompl::base::OptimizationObjectivePtr allocateOptimizationObjective(
	ompl::base::SpaceInformationPtr& space, std::string& obj_type)
{
	if (obj_type == "path_length") {
		return std::make_shared<ompl::base::PathLengthOptimizationObjective>(space); 
	} else {
		SMPL_ERROR("Unrecognized optimization objective: %s", obj_type.c_str());
		return ompl::base::OptimizationObjectivePtr(); 
	}
}

bool CriticalPRMConstructor::runProblemInstance(
	ompl::base::State* start,
	ompl::base::State* goal) 
{
	bool debugging = false; 
	std::string obj_type = "path_length"; // Do I need others?


	if (debugging) {
		PrintOMPLState(m_state_space.get(), start); 
		PrintOMPLState(m_state_space.get(), goal); 		
	}

	// Reset planning problem 
	m_prm_planner->clearQuery(); 

	// Reset problem definition
	m_pdef->clearStartStates(); 
	m_pdef->addStartState(start); 

	m_pdef->clearGoal(); 
	m_pdef->setGoalState(goal); 

	m_prm_planner->setProblemDefinition(m_pdef); 

	auto time_termination_condition = ompl::base::timedPlannerTerminationCondition(10.0); 
	auto planning_res = m_prm_planner->solve(time_termination_condition); 

	if (debugging && planning_res) {
		SMPL_ERROR("RAW SOLUTION:");
		auto found_path = m_pdef->getSolutionPath().get(); 
		dynamic_cast<const ompl::geometric::PathGeometric&>(*found_path).printAsMatrix(std::cout); 
	}


	if (planning_res) {
		auto found_path = m_pdef->getSolutionPath().get();  
		auto found_path_geo = dynamic_cast<ompl::geometric::PathGeometric&>(*found_path); 

		// Simplify found path to remove non-critical states
		auto path_len_obj = allocateOptimizationObjective(m_space, obj_type); 

		auto path_simplifier = ompl::geometric::PathSimplifier(m_space, 
			m_pdef->getGoal()); 

		if (path_simplifier.reduceVertices(found_path_geo)) {
			if (debugging) {
				SMPL_ERROR("SIMPLIFIED SOLUTION:"); 
				found_path_geo.printAsMatrix(std::cout); 
			}
		}

		// Update critical scores
		for (auto i = 0; i < found_path_geo.getStateCount(); ++i) {
			if (i == 0 || i == found_path_geo.getStateCount()-1) continue; 
			auto crit_state = MakeStateSMPL(m_state_space.get(), found_path_geo.getState(i)); 
			++m_critical_score_map[crit_state]; 
			if (debugging) {
				SMPL_ERROR("Incrementing frequency counter for following state %d", i); 
				PrintSMPLState(crit_state); 
			}			
		}
		// SMPL_ERROR("reduced vertices size has %d verts", found_path_geo.getStateCount()); 
		// auto non_terminal_state_count = found_path_geo.getStateCount() -2; // omit start/goal 
		// for (int i = 1; i < non_terminal_state_count; ++i) {
		// 	auto critical_state = MakeStateSMPL(m_state_space.get(), found_path_geo.getState(i)); 
		// 	++m_critical_score_map[critical_state]; 

		// 	if (debugging) {
		// 		SMPL_ERROR("Incrementing frequency counter for following state %d", i); 
		// 		PrintSMPLState(critical_state); 
		// 	}
		// }
	} 
	return planning_res ? true : false; 
}
} // namespace smpl 