#include "ICBSNode.h"

#ifndef LPA
ICBSNode::ICBSNode() : parent(nullptr)
{
	g_val = 0;
	makespan = 0;
	depth = 0;
}
#else
ICBSNode::ICBSNode(vector<LPAStar*>& lpas) : parent(nullptr), lpas(lpas)
{
	g_val = 0;
	makespan = 0;
	depth = 0;
}
#endif

ICBSNode::ICBSNode(ICBSNode* parent) : parent(parent), lpas(parent->lpas.size())
{
	g_val = parent->g_val;
	makespan = parent->makespan;
	depth = parent->depth + 1;
#ifndef LPA
#else
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

int ICBSNode::add_constraint(const Constraint& constraint, const std::vector < std::unordered_map<int, AvoidanceState > >* cat)
{
	constraints.push_back(constraint);
#ifndef LPA
	return 0;
#else
    auto [loc1, loc2, timestep, positive_constraint] = constraint;
    if (lpas[agent_id] != nullptr) {
        lpas[agent_id] = new LPAStar(*lpas[agent_id]);
        int generated_before = lpas[agent_id]->allNodes_table.size();
        if (positive_constraint == false)  // negative constraint
        {
            if (loc2 == -1)  // vertex constraint
                lpas[agent_id]->addVertexConstraint(loc1, timestep, *cat);
            else
                lpas[agent_id]->addEdgeConstraint(loc1, loc2, timestep, *cat);
        }
        return lpas[agent_id]->allNodes_table.size() - generated_before;
    } else  // Replanning for the other agents when a positive constraint is added happens elsewhere
        return 0;
#endif
}
