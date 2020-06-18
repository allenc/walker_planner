#include "critical_roadmap/critical_prm.h"

// standard incldues
#include <random>
#include <memory>
#include <fstream> 
#include <algorithm>
#include <exception>
#include <thread>

// system includes
#include <ompl/base/Planner.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/tools/config/MagicConstants.h> 
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <smpl/console/console.h> 
#include <smpl/utils/debugging_utils.h> 
#include <smpl/utils/logging_utils.h> 
#include <smpl_ompl_interface/ompl_interface.h>


#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/graph/astar_search.hpp>

#include "GoalVisitor.hpp"

 #define foreach BOOST_FOREACH

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

		std::vector<double> CriticalPRM::toSMPLState(const base::State* state) 
		{
			return smpl::MakeStateSMPL(si_->getStateSpace().get(), state); 
		}

		// From newer version of ompl
		CriticalPRM::CriticalPRM(
			const base::PlannerData& data, 
			bool starStrategy)
		: 
		    PRM(data.getSpaceInformation(), starStrategy), 
		    m_mode(), 
		    m_vertex_state_id_map()
		{
			if (data.numVertices() > 0)
			{
                // mapping between vertex id from PlannerData and Vertex in Boost.Graph
				std::map<unsigned int, Vertex> vertices;
                // helper function to create vertices as needed and update the vertices mapping
				const auto &getOrCreateVertex = [&](unsigned int vertex_index) {
					if (!vertices.count(vertex_index))
					{
						const auto &data_vertex = data.getVertex(vertex_index);
						Vertex graph_vertex = boost::add_vertex(g_);
						stateProperty_[graph_vertex] = si_->cloneState(data_vertex.getState());
						totalConnectionAttemptsProperty_[graph_vertex] = 1;
						successfulConnectionAttemptsProperty_[graph_vertex] = 0;
						vertices[vertex_index] = graph_vertex;
					}
					return vertices.at(vertex_index);
				};

                specs_.multithreaded = false;  // temporarily set to false since nn_ is used only in single thread
                nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
                specs_.multithreaded = true;
                nn_->setDistanceFunction([this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });

                for (size_t vertex_index = 0; vertex_index < data.numVertices(); ++vertex_index)
                {
                	Vertex m = getOrCreateVertex(vertex_index);
                	std::vector<unsigned int> neighbor_indices;
                	data.getEdges(vertex_index, neighbor_indices);
                	if (neighbor_indices.empty())
                	{
                		disjointSets_.make_set(m);
                	}
                	else
                	{
                		for (const unsigned int neighbor_index : neighbor_indices)
                		{
                			Vertex n = getOrCreateVertex(neighbor_index);
                			totalConnectionAttemptsProperty_[n]++;
                			successfulConnectionAttemptsProperty_[n]++;
                			base::Cost weight;
                			data.getEdgeWeight(vertex_index, neighbor_index, &weight);
                			const Graph::edge_property_type properties(weight);
                			boost::add_edge(m, n, properties, g_);
                			uniteComponents(m, n);
                		}
                	}
                	nn_->add(m);
                }
            }
        }

        PRM::Vertex CriticalPRM::addMilestone(base::State *state)
        {
        	std::lock_guard<std::mutex> _(graphMutex_);

        	PRM::Vertex m = boost::add_vertex(g_);
        	stateProperty_[m] = state;
        	totalConnectionAttemptsProperty_[m] = 1;
        	successfulConnectionAttemptsProperty_[m] = 0;

     // Initialize to its own (dis)connected component.
        	disjointSets_.make_set(m);

     // Which milestones will we attempt to connect to?
        	const std::vector<PRM::Vertex> &neighbors = connectionStrategy_(m);

        	foreach (PRM::Vertex n, neighbors)
        	if (connectionFilter_(n, m))
        	{
        		totalConnectionAttemptsProperty_[m]++;
        		totalConnectionAttemptsProperty_[n]++;
        		if (si_->checkMotion(stateProperty_[n], stateProperty_[m]))
        		{
        			successfulConnectionAttemptsProperty_[m]++;
        			successfulConnectionAttemptsProperty_[n]++;
        			const base::Cost weight = opt_->motionCost(stateProperty_[n], stateProperty_[m]);
        			const Graph::edge_property_type properties(weight);
        			boost::add_edge(n, m, properties, g_);
        			boost::add_edge(m, n, properties, g_);        			
        			uniteComponents(n, m);
        		}
        	}

        	nn_->add(m);

        	return m;
        }

        void CriticalPRM::expandRoadmap(double expandTime)
        {
        	CriticalPRM::expandRoadmap(base::timedPlannerTerminationCondition(expandTime));
        }        

        void CriticalPRM::expandRoadmap(const base::PlannerTerminationCondition &ptc)
        {
        	if (!simpleSampler_)
        		simpleSampler_ = si_->allocStateSampler();

        	int max_random_bounce_steps = 5; // not in curr version
        	std::vector<base::State *> states(max_random_bounce_steps);
        	si_->allocStates(states);
        	CriticalPRM::expandRoadmap(ptc, states);
        	si_->freeStates(states);
        }        

        void CriticalPRM::expandRoadmap(const base::PlannerTerminationCondition &ptc,
        	std::vector<base::State *> &workStates)
        {
     // construct a probability distribution over the vertices in the roadmap
     // as indicated in
     //  "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces"
     //        Lydia E. Kavraki, Petr Svestka, Jean-Claude Latombe, and Mark H. Overmars

        	PDF<PRM::Vertex> pdf;
        	foreach (PRM::Vertex v, boost::vertices(g_))
        	{
        		const unsigned long int t = totalConnectionAttemptsProperty_[v];
        		pdf.add(v, (double)(t - successfulConnectionAttemptsProperty_[v]) / (double)t);
        	}

        	if (pdf.empty())
        		return;

        	while (!ptc)
        	{
        		iterations_++;
        		PRM::Vertex v = pdf.sample(rng_.uniform01());
        		unsigned int s =
        		si_->randomBounceMotion(simpleSampler_, stateProperty_[v], workStates.size(), workStates, false);
        		if (s > 0)
        		{
        			s--;
        			PRM::Vertex last = CriticalPRM::addMilestone(si_->cloneState(workStates[s]));

        			graphMutex_.lock();
        			for (unsigned int i = 0; i < s; ++i)
        			{
                 // add the vertex along the bouncing motion
        				PRM::Vertex m = boost::add_vertex(g_);
        				stateProperty_[m] = si_->cloneState(workStates[i]);
        				totalConnectionAttemptsProperty_[m] = 1;
        				successfulConnectionAttemptsProperty_[m] = 0;
        				disjointSets_.make_set(m);

                 // add the edge to the parent vertex
        				const base::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[m]);
        				const Graph::edge_property_type properties(weight);
        				boost::add_edge(v, m, properties, g_);
        				boost::add_edge(m, v, properties, g_);        				
        				uniteComponents(v, m);

                 // add the vertex to the nearest neighbors data structure
        				nn_->add(m);
        				v = m;
        			}

             // if there are intermediary states or the milestone has not been connected to the initially sampled vertex,
             // we add an edge
        			if (s > 0 || !sameComponent(v, last))
        			{
                 // add the edge to the parent vertex
        				const base::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[last]);
        				const Graph::edge_property_type properties(weight);
        				boost::add_edge(v, last, properties, g_);
        				boost::add_edge(last, v, properties, g_);        				
        				uniteComponents(v, last);
        			}
        			graphMutex_.unlock();
        		}
        	}
        }        

        void CriticalPRM::growRoadmap(double growTime)
        {
        	CriticalPRM::growRoadmap(base::timedPlannerTerminationCondition(growTime));
        }

        void CriticalPRM::growRoadmap(const base::PlannerTerminationCondition &ptc)
        {
        	if (!isSetup())
        		setup();
        	if (!sampler_)
        		sampler_ = si_->allocValidStateSampler();

        	base::State *workState = si_->allocState();
        	CriticalPRM::growRoadmap(ptc, workState);
        	si_->freeState(workState);
        }

        void CriticalPRM::growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *workState)
        {
     /* grow roadmap in the regular fashion -- sample valid states, add them to the roadmap, add valid connections */
        	while (!ptc)
        	{
        		iterations_++;
         // search for a valid state
        		bool found = false;
        		while (!found && !ptc)
        		{
        			unsigned int attempts = 0;
        			do
        			{
        				found = sampler_->sample(workState);
        				attempts++;
        			} while (attempts < magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found);
        		}
         // add it as a milestone
        		if (found)
        			CriticalPRM::addMilestone(si_->cloneState(workState));
        	}
        }

        // void CriticalPRM::getPlannerData(base::PlannerData& data) const 
        // {
        // 	Planner::getPlannerData(data);
  
        //     // Explicitly add start and goal states:
        // 	for (unsigned long i : startM_)
        // 		data.addStartVertex(
        // 			base::CritPRMVertex(stateProperty_[i], const_cast<CriticalPRM *>(this)->disjointSets_.find_set(i)));

        // 	for (unsigned long i : goalM_)
        // 		data.addGoalVertex(
        // 			base::CritPRMVertex(stateProperty_[i], const_cast<CriticalPRM *>(this)->disjointSets_.find_set(i)));

        //     // Adding edges and all other vertices simultaneously
        // 	foreach (const Edge e, boost::edges(g_))
        // 	{
        // 		const Vertex v1 = boost::source(e, g_);
        // 		const Vertex v2 = boost::target(e, g_);
        // 		data.addEdge(base::PlannerDataVertex(stateProperty_[v1]), base::PlannerDataVertex(stateProperty_[v2]));

        //         // Add the reverse edge, since we're constructing an undirected roadmap
        // 		data.addEdge(base::CritPRMVertex(stateProperty_[v2]), base::CritPRMVertex(stateProperty_[v1]));

        //         // Add tags for the newly added vertices
        // 		data.tagState(stateProperty_[v1], const_cast<CriticalPRM *>(this)->disjointSets_.find_set(v1));
        // 		data.tagState(stateProperty_[v2], const_cast<CriticalPRM *>(this)->disjointSets_.find_set(v2));
        // 	}        	
        // }



		// void CriticalPRM::addCriticalConnections(const std::string& f_critical_states)
		// {
		// 	// Read and load critical ompl milestones from file
		// 	std::vector<base::State*> critical_roadmap_milestones; 
		// 	if (!ReadMilestonesFromFile(si_->getStateSpace().get(), 
		// 		f_critical_states, critical_roadmap_milestones)) 
		// 	{
		// 		OMPL_ERROR("Failed to read critical milestones from file %s", 
		// 			f_critical_states.c_str()); 
		// 	}

		// 	// Convert ompl state to graph ids 
		// 	std::vector<unsigned long int> critical_vertex_ids; 
		// 	for (auto m : critical_roadmap_milestones) {
		// 		auto s = toSMPLState(m); 
		// 		if (m_vertex_state_id_map.find(s) == m_vertex_state_id_map.end()) {
		// 			OMPL_ERROR("Failed to find OMPL state in vertex map"); 
		// 		}
		// 		critical_vertex_ids.push_back(m_vertex_state_id_map[s]); 
		// 	}

		// 	// Connect critical states to all other vertices 
		// 	for (const auto& m : critical_vertex_ids) {

		// 		const std::vector<Vertex>& connect_targets = m_vertices; 		

		// 		for (const auto& t : connect_targets) {
		// 			if (connectionFilter_(t, m)) // default set to always return true
		// 			{
		// 				totalConnectionAttemptsProperty_[m]++; 
		// 				totalConnectionAttemptsProperty_[t]++; 						
		// 				if (si_->checkMotion(stateProperty_[m], stateProperty_[t]))
		// 				{
		// 					successfulConnectionAttemptsProperty_[m]++; 
		// 					successfulConnectionAttemptsProperty_[t]++; 							
		// 					const base::Cost weight = opt_->motionCost(stateProperty_[t], stateProperty_[m]);
		// 					const Graph::edge_property_type properties(weight);
		// 					boost::add_edge(t, m, properties, g_);
		// 					uniteComponents(t, m);
		// 				}
		// 			}
		// 		}
		// 	}
		// }

		// void CriticalPRM::checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution)
		// {
		// 	auto *goal = static_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
		// 	while (!ptc && !addedNewSolution_)
		// 	{
  //        // Check for any new goal states
		// 		if (goal->maxSampleCount() > goalM_.size())
		// 		{
		// 			const base::State *st = pis_.nextGoal();
		// 			if (st != nullptr)
		// 				goalM_.push_back(addMilestone(si_->cloneState(st)));
		// 		}

  //        // Check for a solution
		// 		addedNewSolution_ = CriticalPRM::maybeConstructSolution(startM_, goalM_, solution);
  //        // Sleep for 1ms
		// 		if (!addedNewSolution_)
		// 			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		// 	}
		// }

		// ompl::base::PathPtr CriticalPRM::constructSolution(
		// 	const Vertex &start,
		// 	const Vertex &goal)
		// {
		// 	// OMPL_INFORM("calling crit construct solution"); 
		// 	std::lock_guard<std::mutex> _(graphMutex_);
		// 	boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));

		// 	try
		// 	{
  //        // Consider using a persistent distance_map if it's slow
		// 		boost::astar_search(
		// 			g_, start, [this, goal](Vertex v) { return costHeuristic(v, goal); },
		// 			boost::predecessor_map(prev)
		// 			.distance_compare([this](base::Cost c1, base::Cost c2) { return opt_->isCostBetterThan(c1, c2); })
		// 			.distance_combine([this](base::Cost c1, base::Cost c2) { return opt_->combineCosts(c1, c2); })
		// 			.distance_inf(opt_->infiniteCost())
		// 			.distance_zero(opt_->identityCost())
		// 			.visitor(AStarGoalVisitor<Vertex>(goal)));
		// 	}
		// 	catch (AStarFoundGoal &)
		// 	{
		// 	}

		// 	if (prev[goal] == goal) { 
		// 		OMPL_INFORM("Could not find solution path wtf"); 
		// 		return nullptr; 
		// 		// throw Exception(name_, "Could not find solution path");
		// 	}

		// 	auto p(std::make_shared<PathGeometric>(si_));
		// 	for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
		// 		p->append(stateProperty_[pos]);
		// 	p->append(stateProperty_[start]);
		// 	p->reverse();

		// 	return p;
		// }

		// bool CriticalPRM::maybeConstructSolution(
		// 	const std::vector< Vertex >& starts,
		// 	const std::vector< Vertex >& goals,
		// 	base::PathPtr &solution)		
		// {

		// 	// OMPL_INFORM("calling crit maybe construct solution"); 			
		// 	base::Goal *g = pdef_->getGoal().get();
		// 	base::Cost sol_cost(opt_->infiniteCost());
		// 	foreach (Vertex start, starts)
		// 	{
		// 		foreach (Vertex goal, goals)
		// 		{
  //                   // we lock because the connected components algorithm is incremental and may change disjointSets_
		// 			graphMutex_.lock();
		// 			bool same_component = sameComponent(start, goal);
		// 			graphMutex_.unlock();

		// 			if (same_component && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
		// 			{
		// 				base::PathPtr p = CriticalPRM::constructSolution(start, goal);
		// 				if (p)
		// 				{
		// 					base::Cost pathCost = p->cost(opt_);
		// 					if (opt_->isCostBetterThan(pathCost, bestCost_))
		// 						bestCost_ = pathCost;

  //                           // Check if optimization objective is satisfied
		// 					if (opt_->isSatisfied(pathCost))
		// 					{
		// 						solution = p;
		// 						return true;
		// 					}
		// 					if (opt_->isCostBetterThan(pathCost, sol_cost))
		// 					{
		// 						solution = p;
		// 						sol_cost = pathCost;
		// 					}
		// 				}
		// 			}
		// 		}
		// 	}

		// 	return false;			

		// }

		bool CriticalPRM::constructStateToVertexMap()
		{
			auto v_state_map = boost::get(PRM::vertex_state_t(), g_); 
			for (auto v : boost::make_iterator_range(boost::vertices(g_))) {
				auto s = toSMPLState(v_state_map[v]); 
				if (m_vertex_state_id_map.find(s) != m_vertex_state_id_map.end()) {
					OMPL_ERROR("Duplicate vertex found"); 
				}
				m_vertex_state_id_map[s] = v; 
			}						
			OMPL_INFORM("Constructed state to vertex mapping of size %zu", 
				m_vertex_state_id_map.size()); 
		}

		base::PlannerStatus CriticalPRM::solve(const base::PlannerTerminationCondition& ptc) 
		{
			checkValidity();
			base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

			// OMPL_INFORM("%s: Solving with %zu starts and %zu goals", getName().c_str(), 
			// 	startM_.size(), goalM_.size()); 

			if (m_vertex_state_id_map.empty()) {
				OMPL_ERROR("%s: State to vertex mapping undefined, call 'constructStateToVertexMap()'", 
					getName().c_str()); 
			}

			// validity check since we can't add any milestones
			if (!goal)
			{
				OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
				return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
			}

            // Add the valid start states as milestones   
			while (const base::State *st = pis_.nextStart()) {
				switch (m_mode) {
					case Mode::CONSTRUCT:
					    // st assumed to be in prm 
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