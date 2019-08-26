#include "ICBSNode.h"

#ifndef LPA
ICBSNode::ICBSNode(int num_of_agents) : parent(nullptr)
{
	g_val = 0;
	makespan = 0;
	depth = 0;
	positive_constraints.resize(num_of_agents);
	negative_constraints.resize(num_of_agents);
}
#else
ICBSNode::ICBSNode(vector<LPAStar*>& lpas) : parent(nullptr), lpas(lpas)
{
	g_val = 0;
	makespan = 0;
	depth = 0;
	positive_constraints.resize(lpas.size());
	negative_constraints.resize(lpas.size());
}
#endif

ICBSNode::ICBSNode(ICBSNode* parent) : parent(parent), lpas(parent->lpas.size())
{
	g_val = parent->g_val;
	makespan = parent->makespan;
	depth = parent->depth + 1;
#ifndef LPA
	positive_constraints.resize(parent->positive_constraints.size());
	negative_constraints.resize(parent->negative_constraints.size());
#else
	positive_constraints.resize(lpas.size());
	negative_constraints.resize(lpas.size());
	for (int j = 0; j < parent->lpas.size(); ++j) {
		lpas[j] = parent->lpas[j];
	}
#endif
}

void ICBSNode::clear()
{
	cardinalConf.clear();
	semiConf.clear();
	nonConf.clear();
	unknownConf.clear();
#ifndef LPA
#else
	// TODO: Free the LPAStar instances that aren't shared with any of the children
	lpas.clear();
#endif
}

// In the LPA* variant, this creates a copy of the LPA* instance of the agent before adding the constraint to it,
// unless same_lpa_star is true.
// Returns the number of nodes generated
int ICBSNode::add_constraint(const Constraint& constraint, const std::vector < std::unordered_map<int, AvoidanceState > >* cat,
        bool same_lpa_star /* = false*/)
{
    auto [loc1, loc2, constraint_timestep, positive_constraint] = constraint;
    if (positive_constraint)
	    positive_constraints[agent_id].push_back(constraint);
    else
        negative_constraints[agent_id].push_back(constraint);
#ifndef LPA
	return 0;
#else
    if (lpas[agent_id] != nullptr) {
        auto [loc1, loc2, timestep, positive_constraint] = constraint;
        int generated_before = lpas[agent_id]->allNodes_table.size();
        if (positive_constraint == false)  // negative constraint
        {
            if (same_lpa_star == false)
                lpas[agent_id] = new LPAStar(*lpas[agent_id]);
            if (loc2 == -1)  // vertex constraint
                lpas[agent_id]->addVertexConstraint(loc1, timestep, *cat);
            else
                lpas[agent_id]->addEdgeConstraint(loc1, loc2, timestep, *cat);
        } else {
            // TODO: When LPA* gets support for positive constraints, handle it here
            // Replanning for the other agents when a positive constraint is added happens elsewhere
        }
        return lpas[agent_id]->allNodes_table.size() - generated_before;
    } else
        return 0;
#endif
}

int ICBSNode::pop_constraint(const std::vector < std::unordered_map<int, AvoidanceState > >* cat)
{
#ifndef LPA
    negative_constraints[agent_id].pop_back();
    return 0;
#else
    if (lpas[agent_id] != nullptr) {
        auto [loc1, loc2, timestep, positive_constraint] = negative_constraints[agent_id].back();
        // TODO: Support positive constraints
        int generated_before = lpas[agent_id]->allNodes_table.size();
        if (positive_constraint == false)  // negative constraint
        {
            if (loc2 == -1)  // vertex constraint
                lpas[agent_id]->popVertexConstraint(loc1, timestep, *cat);
            else
                lpas[agent_id]->popEdgeConstraint(loc1, loc2, timestep, *cat);
        } else {
            // TODO: When LPA* gets support for positive constraints, handle it here
            // Replanning for the other agents when a positive constraint is added happens elsewhere
        }

        negative_constraints[agent_id].pop_back();
        return lpas[agent_id]->allNodes_table.size() - generated_before;
    } else {
        std::cout << "LPA* " << agent_id << " unexpectedly null!" << std::endl;
        std::abort();
        negative_constraints[agent_id].pop_back();
        return 0;
    }
#endif
}
