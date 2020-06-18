#ifndef CRITICAL_PRM_H
#define CRITICAL_PRM_H

// standard includes
#include <unordered_map> 

// system includes
#include <Eigen/Dense>

#include <smpl/types.h> 
#include <smpl/utils/logging_utils.h>

#include <boost/functional/hash.hpp> 
#include <boost/serialization/export.hpp>

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/util/RandomNumbers.h>

// project includes
#include "utils/utils.h"

namespace ompl {
	namespace base {
		class CritPRMVertex : public PlannerDataVertex
		{
        public:
            /// \brief Constructor.  Takes a state pointer and an optional integer tag.
            CritPRMVertex(const State *st, int tag = 0) : state_(st), tag_(tag) {}
            /// \brief Copy constructor.
            CritPRMVertex(const CritPRMVertex &rhs) : state_(rhs.state_), tag_(rhs.tag_) {}
            // virtual ~PlannerDataVertex() {}

            /// \brief Return a clone of this object, allocated from the heap.
            virtual CritPRMVertex* clone() const
            {
                return new CritPRMVertex(*this);
            }

            // /// \brief Equivalence operator.  Return true if the state pointers are equal.
            virtual bool operator==(const CritPRMVertex &rhs) const
            {
                // States should be unique
                return state_ == rhs.state_;
            }

            /// \brief Returns true if this vertex is not equal to the argument.
            /// This is the complement of the == operator.
            bool operator!=(const CritPRMVertex &rhs) const
            {
                return !(*this == rhs);
            }

        protected:
            CritPRMVertex() {}

            friend class boost::serialization::access;
            template <class Archive>
            void serialize(Archive & ar, const unsigned int /*version*/)
            {
                // ar & tag_;
                ar & tag_; 
                // ar & boost::serialization::base_object<PlannerDataVertex>(*this); 
                // Serialization of the state pointer is handled by PlannerDataStorage
            }

            /// \brief The state represented by this vertex
            const State *state_;
            /// \brief A generic integer tag for this state.  Not used for equivalence checking.
            int tag_;

            friend class PlannerData;
            friend class PlannerDataStorage;
        };		

	} // namespace base 
	namespace geometric {

		// Critical PRM with modes for roadmap construction and querying 
		class CriticalPRM : public PRM
		{			
		public:			
			enum Mode {CONSTRUCT, QUERY};

			CriticalPRM(const base::SpaceInformationPtr& si,
				bool starStrategy=false) : PRM(si, starStrategy), m_mode(), 
				m_vertex_state_id_map() {}

			CriticalPRM(const base::PlannerData& data, bool starStrategy=false); 

			// PRM Functions 
			bool constructStateToVertexMap(); // call this after base prm is done and before solve
			base::PlannerStatus solve(const base::PlannerTerminationCondition& ptc) override;  
            void growRoadmap(double growTime);             
            void growRoadmap(const base::PlannerTerminationCondition &ptc);             

            void expandRoadmap(double expandTime); 
            void expandRoadmap(const base::PlannerTerminationCondition &ptc); 

			// Critical PRM Functions
			inline void setMode(Mode& mode) { m_mode = mode; }			
			// void addCriticalConnections(const std::string& f_critical_states); // needs refactoring 

		protected: 
            Vertex addMilestone(base::State* state); 
            void growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *workState);   
            void expandRoadmap(const base::PlannerTerminationCondition &ptc,
                std::vector<base::State *> &workStates); 

            Mode m_mode; 

		    using VertexMap = std::unordered_map<smpl::RobotState, Vertex, 
		        smpl::VectorHash<double> >;
		    VertexMap m_vertex_state_id_map; 		    

		private: 
		    std::vector<double> toSMPLState(const base::State* state); 		    
		}; 

	} // namespace geometric
} // namespace ompl

// BOOST_CLASS_EXPORT(ompl::base::CritPRMVertex);
#endif