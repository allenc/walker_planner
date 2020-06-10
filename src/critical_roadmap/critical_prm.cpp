#include "critical_roadmap/critical_prm.h"

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

namespace ompl { 
	namespace geometric { 

		static
		bool ReadMilestonesFromFile(
			const ompl::base::StateSpace* space, 
			const std::string& f_milestones, 
			std::vector<base::State*>& milestones)
		{
			// Read as smpl::RobotState and convert to ompl::State*			
			std::vector<smpl::RobotState> v_rs; 

			std::ifstream f(f_milestones); 
			std::string line; 
			while (std::getline(f, line)) {
				std::istringstream iss(line); 
				std::string line_stream; 

				smpl::RobotState row; 
				while (std::getline(iss, line_stream, ',')) {
					row.push_back(std::stod(line_stream)); 
				}

				v_rs.push_back(row); 
			}

			milestones.clear(); 
			for (auto& rs : v_rs) {
				auto milestone = smpl::MakeStateOMPL(space, rs); 
				milestones.push_back(milestone); 
			}

			SMPL_INFO("Read %zu milestones from file", milestones.size()); 

			return true; 
		}

		void CriticalPRM::setLoggingParameters(
			bool& save,
			const std::string& f_roadmap)
		{
			m_save = save; 

			if (m_save) {
				m_writer = new smpl::Writer(f_roadmap);				
				m_writer->Clear(); 
			}
		}

		void CriticalPRM::growRoadmap(double growTime)
		{
			this->growRoadmap(base::timedPlannerTerminationCondition(growTime));
		}

		void CriticalPRM::growRoadmap(const base::PlannerTerminationCondition &ptc)
		{
			if (!isSetup())
				setup();
			if (!sampler_)
				sampler_ = si_->allocValidStateSampler();

			base::State *workState = si_->allocState();
			this->growRoadmap(ptc, workState);
			si_->freeState(workState);

			// Build state -> vertex id mapping
			auto v_state_map = boost::get(PRM::vertex_state_t(), g_); 
			for (auto v : boost::make_iterator_range(boost::vertices(g_))) {
				auto s = toSMPLState(v_state_map[v]); 
				if (m_vertex_state_id_map.find(s) != m_vertex_state_id_map.end()) {
					OMPL_ERROR("Duplicate vertex found"); 
				}
				m_vertex_state_id_map[s] = v; 
			}			
		}		

		void CriticalPRM::growRoadmap(
			const base::PlannerTerminationCondition &ptc,
			base::State *workState)
		{
			std::vector<smpl::RobotState> added_milestones; 			

            /* grow roadmap in the regular fashion -- sample valid states, add them to the roadmap, add valid connections */
			while (ptc == false)
			{
				iterations_++;

                // search for a valid state
				bool found = false;
				while (!found && ptc == false)
				{
					unsigned int attempts = 0;
					do
					{
						found = sampler_->sample(workState);
						attempts++;
					} while (attempts < magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found);
				}

                // add it as a milestone
				if (found) {

					if (m_save) {
						added_milestones.push_back(toSMPLState(workState)); 
					}

					addMilestone(si_->cloneState(workState));					
				}
			}

			if (m_save) {
				m_writer->WriteRobotPath(added_milestones); 
			}
		}

		void CriticalPRM::growRoadmapFromFile(const std::string& f_roadmap)
		{
			if (!isSetup())
				setup();			

			// Read and load ompl milestones from file 
			std::vector<base::State*> roadmap_milestones;  
			if (!ReadMilestonesFromFile(si_->getStateSpace().get(), 
				f_roadmap, roadmap_milestones)) 
			{
				OMPL_ERROR("Failed to read milestones from file %s", 
					f_roadmap.c_str()); 
			} 

			base::State* workState = si_->allocState();

			for (auto m : roadmap_milestones) {
				workState = m;
				auto v_added = PRM::addMilestone(si_->cloneState(workState)); 
				m_vertices.push_back(v_added); 
			}

			si_->freeState(workState);

			// Build state -> vertex id mapping
			auto v_state_map = boost::get(PRM::vertex_state_t(), g_); 
			for (auto v : boost::make_iterator_range(boost::vertices(g_))) {
				auto s = toSMPLState(v_state_map[v]); 
				if (m_vertex_state_id_map.find(s) != m_vertex_state_id_map.end()) {
					OMPL_ERROR("Duplicate vertex found"); 
				}
				m_vertex_state_id_map[s] = v; 
			}
		}

		std::vector<double> CriticalPRM::toSMPLState(const base::State* state) 
		{
			return smpl::MakeStateSMPL(si_->getStateSpace().get(), state); 
		}

		void CriticalPRM::addCriticalConnections(const std::string& f_critical_states)
		{
			// Read and load critical ompl milestones from file
			std::vector<base::State*> critical_roadmap_milestones; 
			if (!ReadMilestonesFromFile(si_->getStateSpace().get(), 
				f_critical_states, critical_roadmap_milestones)) 
			{
				OMPL_ERROR("Failed to read critical milestones from file %s", 
					f_critical_states.c_str()); 
			}

			// Convert ompl state to graph ids 
			std::vector<unsigned long int> critical_vertex_ids; 
			for (auto m : critical_roadmap_milestones) {
				auto s = toSMPLState(m); 
				if (m_vertex_state_id_map.find(s) == m_vertex_state_id_map.end()) {
					OMPL_ERROR("Failed to find OMPL state in vertex map"); 
				}
				critical_vertex_ids.push_back(m_vertex_state_id_map[s]); 
			}

			// Connect critical states to all other vertices 
			for (const auto& m : critical_vertex_ids) {

				const std::vector<Vertex>& connect_targets = m_vertices; 		

				for (const auto& t : connect_targets) {
					if (connectionFilter_(t, m)) // default set to always return true
					{
						totalConnectionAttemptsProperty_[m]++; 
						totalConnectionAttemptsProperty_[t]++; 						
						if (si_->checkMotion(stateProperty_[m], stateProperty_[t]))
						{
							successfulConnectionAttemptsProperty_[m]++; 
							successfulConnectionAttemptsProperty_[t]++; 							
							const base::Cost weight = opt_->motionCost(stateProperty_[t], stateProperty_[m]);
							const Graph::edge_property_type properties(weight);
							boost::add_edge(t, m, properties, g_);
							uniteComponents(t, m);
						}
					}
				}
			}
		}


		base::PlannerStatus CriticalPRM::solve(const base::PlannerTerminationCondition& ptc) 
		{
			checkValidity();
			base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

			if (!goal)
			{
				OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
				return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
			}

            // Add the valid start states as milestones   
			while (const base::State *st = pis_.nextStart()) {
				switch (m_mode) {
					case Mode::CONSTRUCT:
					    startM_.push_back(m_vertex_state_id_map[toSMPLState(st)]);
					    break; 
					case Mode::QUERY:
					    startM_.push_back(addMilestone(si_->cloneState(st))); 
					    break; 
					default:
					    OMPL_ERROR("Mode not set"); 
					    return base::PlannerStatus::ABORT; 
				}
			}

			if (startM_.size() == 0)
			{
				OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
				return base::PlannerStatus::INVALID_START;
			}

			if (!goal->couldSample())
			{
				OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
				return base::PlannerStatus::INVALID_GOAL;
			}

            // Ensure there is at least one valid goal state
			if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
			{
				const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();

				if (st) {
				switch (m_mode) {
					case Mode::CONSTRUCT:
	                    goalM_.push_back(m_vertex_state_id_map[toSMPLState(st)]);
					    break; 
					case Mode::QUERY:
   					    goalM_.push_back(addMilestone(si_->cloneState(st))); 
					    break; 
					default:
					    OMPL_ERROR("Mode not set"); 
					    return base::PlannerStatus::ABORT; 
					}					
				}

				if (goalM_.empty())
				{
					OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
					return base::PlannerStatus::INVALID_GOAL;
				}
			}

			unsigned long int nrStartStates = boost::num_vertices(g_);
			OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

            // Reset addedNewSolution_ member and create solution checking thread
			addedNewSolution_ = false;
			base::PathPtr sol;
			boost::thread slnThread(boost::bind(&CriticalPRM::checkForSolution, this, ptc, boost::ref(sol)));

            // construct new planner termination condition that fires when the given ptc is true, or a solution is found
			base::PlannerTerminationCondition ptcOrSolutionFound =
			base::plannerOrTerminationCondition(ptc, base::PlannerTerminationCondition(boost::bind(&CriticalPRM::addedNewSolution, this)));

			// if (m_mode == Mode::QUERY)
  		// 	    constructRoadmap(ptcOrSolutionFound); 

            // Ensure slnThread is ceased before exiting solve
			slnThread.join();

			OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

			if (sol)
			{
				base::PlannerSolution psol(sol);
				psol.setPlannerName(getName());
                // if the solution was optimized, we mark it as such
				psol.setOptimized(opt_, bestCost_, addedNewSolution());
				pdef_->addSolutionPath(psol);
			}

			return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
		}
	} // namespace geometric 
} // namespace ompl 