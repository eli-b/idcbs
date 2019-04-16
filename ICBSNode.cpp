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

int ICBSNode::add_constraint(const Constraint& constraint)
{
	constraints.push_back(constraint);
#ifndef LPA
	return 0;
#else
    if (lpas[agent_id] != nullptr) {
        lpas[agent_id] = new LPAStar(*lpas[agent_id]);
        int generated_before = lpas[agent_id]->allNodes_table.size();
        if (get<3>(constraint) == false)  // negative constraint
        {
            if (get<1>(constraint) == -1)
                lpas[agent_id]->addVertexConstraint(get<0>(constraint), get<2>(constraint));
            else
                lpas[agent_id]->addEdgeConstraint(get<0>(constraint), get<1>(constraint), get<2>(constraint));
        }
        return lpas[agent_id]->allNodes_table.size() - generated_before;
    } else
        return 0;
#endif
}
