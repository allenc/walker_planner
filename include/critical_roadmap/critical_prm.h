#ifndef CRITICAL_PRM_H
#define CRITICAL_PRM_H

// standard includes
#include <unordered_map> 

// system includes
#include <smpl/types.h> 
#include <smpl/utils/logging_utils.h>

#include <Eigen/Dense>
#include <boost/functional/hash.hpp> 

#include <ompl/base/Planner.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>

// project includes
#include "utils/utils.h"

namespace ompl {
	namespace geometric {

		// Critical PRM with modes for roadmap construction and querying 
		class CriticalPRM : public PRM
		{			
		public: 
			CriticalPRM(const base::SpaceInformationPtr& si, 
				bool starStrategy = false) : PRM(si, starStrategy), m_mode(), 
			    m_writer(), m_save(), m_vertices(), m_vertex_state_id_map() {} 

			// Planner Functions 
			base::PlannerStatus solve(const base::PlannerTerminationCondition& ptc) override;  
			void growRoadmap(double growTime); 
			void growRoadmap(const base::PlannerTerminationCondition &ptc); 
			void growRoadmap(const base::PlannerTerminationCondition& ptc,
				base::State* workState); 

			// Critical PRM Functions
			enum Mode {CONSTRUCT, QUERY}; 			
			void setMode(Mode& mode) { m_mode = mode; }
			void setLoggingParameters(bool& save, const std::string& f_roadmap); 
			void growRoadmapFromFile(const std::string& f_roadmap);
			void addCriticalConnections(const std::string& f_critical_states); 
			
		private: // Helpers 
		    std::vector<double> toSMPLState(const base::State* state); 

		protected: // Critical connection member attributes 
		    Mode m_mode; 
		    std::vector<Vertex> m_vertices; 		    
		    using VertexMap = std::unordered_map<smpl::RobotState, Vertex, 
		        smpl::VectorHash<double> >; 
		    VertexMap m_vertex_state_id_map; 

		protected: // Logging member attributes 
			bool m_save; // For saving prm construction process
			smpl::Writer* m_writer; 
		};
		 
	} // namespace geometric
} // namespace ompl
#endif