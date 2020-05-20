#pragma once
#include "MDD.h"
#include "lpa_star.h"

#include <list>
#include <vector>

#define LPA
#define LATEST_CONFLICT_WITHIN_CLASS
#define CBS_UP_AND_DOWN


class ICBSSearch;

class ICBSNode
{
public:
	vector<std::shared_ptr<Conflict>> cardinalGoalConf;  // One side's F potentially increases by a lot
	vector<std::shared_ptr<Conflict>> cardinalConf;
	vector<std::shared_ptr<Conflict>> semiCardinalGoalConf;
	vector<std::shared_ptr<Conflict>> semiCardinalConf;
	vector<std::shared_ptr<Conflict>> nonCardinalConf;
	vector<std::shared_ptr<Conflict>> unknownConf;  // All of those hold pointers just so the conflict member can be a pointer, which can be nullptr.
	void count_conflicts() {
		num_of_conflicts = (int) cardinalGoalConf.size() +
						   (int) cardinalConf.size() +
						   (int) semiCardinalGoalConf.size() +
						   (int) semiCardinalConf.size() +
						   (int) nonCardinalConf.size() +
						   (int) unknownConf.size();

//        for (auto conf : cardinalGoalConf) {
//            int cost_increase = ;
//            if (cost_increase < ((cbs->focal_w - 1) * (f_val + 1)) {  // The right child would be within the same bound as the left
//                num_of_conflicts += cost_increase * cost_increase * MDD_SIZE;  // A wait can be added anywhere
//                                                                               // (unless we're forced to re-visit a location, in which case we're not necessarily
//                                                                               // able to add a wait at the first time we visit it).
//                                                                               // The next wait can be added anywhere in the enlarged MDD, and so on.
//                                                                               // So a cost increase of 3 means enlarging the MDD size by:
//                                                                               // MDD_SIZE + 2*MDD_SIZE + 4*MDD_SIZE = ~3^2 * MDD_SIZE
//                num_of_conflicts--;  // Don't over-count, we already counted the conflict as 1 above.
//            }
//        }
//        for (auto conf : cardinalGoalConf) {
//            int cost_increase = ;
//            if (cost_increase < ((cbs->focal_w - 1) * (f_val + 1))
//                num_of_conflicts--;  // Don't over-count, we already counted the conflict as 1 above.
//        }
	}

	ICBSSearch* cbs;
	ICBSNode* parent;
	std::shared_ptr<Conflict> conflict; // the chosen conflict
	bool branch_on_first_agent;  // For the MVC_BASED disjoint split
	int agent_id; // the agent that constraints are imposed on
	vector<list<Constraint>> positive_constraints;
	vector<list<Constraint>> negative_constraints;
	list<pair<int, vector<PathEntry>>> new_paths; // (agent id + its new path)

    vector<Path *>* all_paths;  // Only populated while node is being expanded

	int g_val;
	int h_val;
	int f_val;
	uint32_t depth; // depth in the CT
	int makespan;
	int num_of_conflicts;
	uint64_t branch;

	bool partialExpansion = false;

    typedef enum {
        YES,
        NO,
        MAYBE
    } WillCostIncrease;
    WillCostIncrease left_cost_will_increase = WillCostIncrease::MAYBE;  // For disjoint splitting, left refers to the
                                    // agent, not the node, since the agent that gets the positive/negative constraints
                                    // is chosen at a later stage.
    WillCostIncrease right_cost_will_increase = WillCostIncrease::MAYBE;
    WillCostIncrease left_f_will_increase = WillCostIncrease::MAYBE;
    WillCostIncrease right_f_will_increase = WillCostIncrease::MAYBE;
    bool is_left_child = false;

	uint64_t time_expanded;
	uint64_t time_generated;

	vector<LPAStar*> lpas;

    uint32_t get_up_and_down_distance(ICBSNode *other);

    ///////////////////////////////////////////////////////////////////////////////
    // NOTE -- Normally, compare_node (lhs,rhs) is supposed to return true if lhs<rhs.
    //         However, Heaps in STL and Boost are implemented as max-Heap.
    //         Hence, to achieve min-Heap, we return true if lhs>=rhs
    ///////////////////////////////////////////////////////////////////////////////

	// the following is used to compare nodes in the OPEN list
	struct compare_node {
		bool operator()(const ICBSNode* n1, const ICBSNode* n2) const {
			return n1->f_val >= n2->f_val;
		}
	};  // used by OPEN to compare nodes by sum_min_f_vals (top of the heap has min sum_min_f_vals)

	// Used to compare nodes in the FOCAL list. Should prefer nodes with less work towards the goal
	// Prefers higher g_val, then fewer conflicts, then more depth, then older nodes.
	struct secondary_compare_node {
		bool operator()(const ICBSNode* n1, const ICBSNode* n2) const {
			if (n1->g_val == n2->g_val)
			{
				if (n1->num_of_conflicts == n2->num_of_conflicts) {
                    if (n1->depth == n2->depth) {
                        return n1->time_generated > n2->time_generated; // break ties towards earlier generated nodes -
                                                                        // explore the tree more broadly (despite preferring depth)
                                                                        // (time_generated can't be equal)
                    }
                    return n1->depth < n2->depth;  // break ties towards *more* depth - more work was done, even if it didn't reduce the number of conflicts or increase the cost yet
                }
				return n1->num_of_conflicts > n2->num_of_conflicts;  // break ties towards fewer conflicts
			}
			return n1->g_val < n2->g_val;  // break ties towards *larger* g_val
		}
	};  // used by FOCAL to compare nodes by num_of_conflicts (top of the heap has min h-val)

	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<compare_node> >
		::handle_type open_handle_t;
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<secondary_compare_node> >
		::handle_type focal_handle_t;
	open_handle_t open_handle;
	bool in_open = false;
	focal_handle_t focal_handle;
	bool in_focal = false;

	// Returns the number of generated nodes
	int add_constraint(const Constraint&, const ConflictAvoidanceTable* cat, bool same_lpa_star = false, bool propagate_positives_up = false);
	int add_lpa_constraint(int to_constrain, const Constraint&, const ConflictAvoidanceTable* cat, bool same_lpa_star = false);
	int pop_constraint(const ConflictAvoidanceTable* cat, bool propagated_positives_up = false);  // Calls pop_lpa_constraint
	int pop_lpa_constraint(const ConflictAvoidanceTable* cat, bool propagated_positives_up = false);
	int pop_lpa_constraint(int to_constrain, const Constraint& constraint, const ConflictAvoidanceTable* cat);

	void clear();

#ifndef LPA
	ICBSNode(int num_of_agents);
#else
	ICBSNode(vector<LPAStar*>& lpas);
#endif
	ICBSNode(ICBSNode* parent, bool is_left_child);

	~ICBSNode() {}
};
