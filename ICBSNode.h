#pragma once
#include "MDD.h"


class ICBSNode
{
public:
	list<std::shared_ptr<Conflict>> cardinalConf;
	list<std::shared_ptr<Conflict>> semiConf;
	list<std::shared_ptr<Conflict>> nonConf;
	list<std::shared_ptr<Conflict>> unknownConf;

	ICBSNode* parent;
	std::shared_ptr<Conflict> conflict; // the chosen conflict
	int agent_id; // the agent that constraints are imposed on
	list<Constraint> constraints;
	list<pair<int, vector<PathEntry>>> new_paths; // (agent id + its new path)

	int g_val;
	int h_val;
	int f_val;
	int depth; // depth in the CT
	int makespan;
	int num_of_collisions;

	bool partialExpansion = false;

	uint64_t time_expanded;
	uint64_t time_generated;

	// the following is used to comapre nodes in the OPEN list
	struct compare_node {
		bool operator()(const ICBSNode* n1, const ICBSNode* n2) const {
			return n1->f_val >= n2->f_val;
		}
	};  // used by OPEN to compare nodes by sum_min_f_vals (top of the heap has min sum_min_f_vals)

	// the following is used to comapre nodes in the FOCAL list
	struct secondary_compare_node {
		bool operator()(const ICBSNode* n1, const ICBSNode* n2) const {
			if (n1->num_of_collisions == n2->num_of_collisions)
			{
				if (n1->g_val == n2->g_val)
					return n1->time_generated > n2->time_generated; // break ties towards earilier generated nodes (FIFO manner)
				return n1->g_val <= n2->g_val;  // break ties towards larger g_val
			}
			return n1->num_of_collisions >= n2->num_of_collisions;
		}
	};  // used by FOCAL to compare nodes by num_of_collisions (top of the heap has min h-val)

	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<compare_node> >
		::handle_type open_handle_t;
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<secondary_compare_node> >
		::handle_type focal_handle_t;
	open_handle_t open_handle;
	focal_handle_t focal_handle;

	void clear();
	ICBSNode();
	ICBSNode(ICBSNode* parent);
};

