#include "ICBSNode.h"

#include <cassert>

#ifndef LPA
ICBSNode::ICBSNode(int num_of_agents) : parent(nullptr), all_paths(nullptr)
{
	g_val = 0;
	makespan = 0;
	depth = 0;
	branch = 0;
	positive_constraints.resize(num_of_agents);
	negative_constraints.resize(num_of_agents);
}
#else
ICBSNode::ICBSNode(vector<LPAStar*>& lpas) : parent(nullptr), lpas(lpas), all_paths(nullptr)
{
	g_val = 0;
	makespan = 0;
	depth = 0;
	branch = 0;
	positive_constraints.resize(lpas.size());
	negative_constraints.resize(lpas.size());
}
#endif

ICBSNode::ICBSNode(ICBSNode* parent, bool is_left_child) : parent(parent),
    positive_constraints(parent->positive_constraints.size()),
    negative_constraints(parent->negative_constraints.size()),
#ifndef LPA
        lpas(parent->lpas.size())
#else
        lpas(parent->lpas)
#endif
{
	g_val = parent->g_val;
	makespan = parent->makespan;
	depth = parent->depth + 1;
	branch = parent->branch;
	this->is_left_child = is_left_child;
	if (!is_left_child)
	    branch |= (0x8000000000000000U >> (depth - 1));
}

void ICBSNode::clear()
{
	cardinalGoalConf.clear();
	cardinalConf.clear();
	semiCardinalGoalConf.clear();
	semiCardinalConf.clear();
	nonCardinalConf.clear();
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
// Assumes all_paths is populated correctly
int ICBSNode::add_constraint(const Constraint& constraint, const ConflictAvoidanceTable* cat,
                             bool same_lpa_star /* = false*/, bool propagate_positives_up /* = false*/)
{
    int ret = 0;
    const auto& [loc1, loc2, constraint_timestep, positive_constraint] = constraint;
    if (positive_constraint) {
        positive_constraints[agent_id].push_back(constraint);
#ifndef LPA
#else
        if (lpas[agent_id] != nullptr)
            ret += add_lpa_constraint(agent_id, constraint, cat, same_lpa_star);
#endif

        if (propagate_positives_up)
            // The MDD levels up to the positive constraint are a superset of the MDD levels of the MDD for reaching
            // the location of the positive constraint at the time of the positive constraint, so any 1-width level
            // among the levels up to that of the positive constraint is also a 1-width level in the MDD for the agent
            // to reach the positive constraint (we know the positive constraint is reachable so it can't be a
            // 0-width level in the smaller MDD). Every 1-width level's node can also be added as a positive constraint
            // - we must pass through it to reach the positive constraint we added.
        {
            for (int i = constraint_timestep; i > 0; i--)
            {
                if ((*(*all_paths)[agent_id])[i].builtMDD == false)  // No MDD for this level. When does this happen?
                    break;
                else if ((*(*all_paths)[agent_id])[i].single == false)  // Not a 1-width MDD level.
                    continue;
                else if ((*(*all_paths)[agent_id])[i - 1].builtMDD && (*(*all_paths)[agent_id])[i - 1].single) {  // Safe because i > 0
                    // This level is narrow and the previous one too - add a positive edge constraint between their
                    // nodes. It's preferable over two positive constraints for some reason.
                    positive_constraints[agent_id].emplace_back((*(*all_paths)[agent_id])[i - 1].location, (*(*all_paths)[agent_id])[i].location, i, true);
#ifndef LPA
#else
                    if (lpas[agent_id] != nullptr)
                        ret += add_lpa_constraint(agent_id, make_tuple((*(*all_paths)[agent_id])[i - 1].location, (*(*all_paths)[agent_id])[i].location, i, true), cat, same_lpa_star);
#endif
                }
                else if (i < constraint_timestep && !(*(*all_paths)[agent_id])[i + 1].single) {
                    // This level is narrow, and the *next* one isn't (so we didn't already add a positive edge
                    // constraint between them) - add a positive vertex constraint for this level's node
                    positive_constraints[agent_id].emplace_back((*(*all_paths)[agent_id])[i].location, -1, i, true);
#ifndef LPA
#else
                    if (lpas[agent_id] != nullptr)
                        ret += add_lpa_constraint(agent_id, make_tuple((*(*all_paths)[agent_id])[i].location, -1, i, true), cat, same_lpa_star);
#endif
                }
            }
        }
    }
    else {
        negative_constraints[agent_id].push_back(constraint);
#ifndef LPA
#else
        if (lpas[agent_id] != nullptr)
            ret += add_lpa_constraint(agent_id, constraint, cat, same_lpa_star);
#endif
    }
    return ret;
}


int ICBSNode::add_lpa_constraint(int to_constrain, const Constraint& constraint,
                                 const ConflictAvoidanceTable* cat, bool same_lpa_star /* = false*/) {
    const auto& [loc1, loc2, timestep, positive_constraint] = constraint;
    int generated_before = lpas[to_constrain]->allNodes_table.size();
    if (positive_constraint == false)  // negative constraint
    {
        if (same_lpa_star == false)
            lpas[to_constrain] = new LPAStar(*lpas[to_constrain]);
        if (loc2 == -1)  // vertex constraint
            lpas[to_constrain]->addVertexConstraint(loc1, timestep, *cat);
        else
            lpas[to_constrain]->addEdgeConstraint(loc1, loc2, timestep, *cat);
    } else {
        // TODO: When LPA* gets support for positive constraints, handle it here
        // Replanning for the other agents when a positive constraint is added happens elsewhere
    }
    return lpas[to_constrain]->allNodes_table.size() - generated_before;
}

int ICBSNode::pop_constraint(const ConflictAvoidanceTable* cat, bool propagated_positives_up /* = false*/)
{
    int ret = 0;
    if (negative_constraints[agent_id].empty() == false) {
#ifdef LPA
        ret += pop_lpa_constraint(cat);
#endif
        negative_constraints[agent_id].pop_back();
    } else {
        auto[location1, location2, timestep, positive] = positive_constraints[agent_id].back();
#ifdef LPA
        ret += pop_lpa_constraint(cat, propagated_positives_up);
#endif
        positive_constraints[agent_id].pop_back();
        if (propagated_positives_up)
            // The MDD levels up to the positive constraint are a superset of the MDD levels of the MDD for reaching
            // the location of the positive constraint at the time of the positive constraint, so any 1-width level
            // among the levels up to that of the positive constraint is also a 1-width level in the MDD for the agent
            // to reach the positive constraint (we know the positive constraint is reachable so it can't be a
            // 0-width level in the smaller MDD). Every 1-width level's node can also be added as a positive constraint
            // - we must pass through it to reach the positive constraint we added.
        {
            // TODO: Can't I just pop all of the positive constraints and be done with it?
            for (int i = timestep; i > 0; i--) {
                if ((*(*all_paths)[agent_id])[i].builtMDD == false)  // No MDD for this level. When does this happen?
                    break;
                else if ((*(*all_paths)[agent_id])[i].single == false)  // Not a 1-width MDD level.
                    continue;
                else if ((*(*all_paths)[agent_id])[i - 1].builtMDD &&
                         (*(*all_paths)[agent_id])[i - 1].single) {  // Safe because i > 0
                    // This level is narrow and the previous one too - add a positive edge constraint between their
                    // nodes. It's preferable over two positive constraints for some reason.
                    positive_constraints[agent_id].pop_back();
                } else if (i < timestep && !(*(*all_paths)[agent_id])[i + 1].single) {
                    // This level is narrow, and the *next* one isn't (so we didn't already add a positive edge
                    // constraint between them) - add a positive vertex constraint for this level's node
                    positive_constraints[agent_id].pop_back();
                }
            }
        }
    }
    return ret;
}

// Only for negative constraints
int ICBSNode::pop_lpa_constraint(const ConflictAvoidanceTable* cat, bool propagated_positives_up /* = false*/)
{
    int ret = 0;
    if (negative_constraints[agent_id].empty()) {
        cerr << "ERROR: No constraint to pop according to" << std::endl;
        assert(false);
    }
    const auto [loc1, loc2, timestep, positive_constraint] = negative_constraints[agent_id].back();  // NOT auto& because the constraint is about to be removed
    if (positive_constraint == false)
        ret += pop_lpa_constraint(agent_id, negative_constraints[agent_id].back(), cat);
    else {
        ret += pop_lpa_constraint(agent_id, positive_constraints[agent_id].back(), cat);
        if (propagated_positives_up) {
            for (int i = timestep; i > 0; i--) {
                if ((*(*all_paths)[agent_id])[i].builtMDD == false)  // No MDD for this level. When does this happen?
                    break;
                else if ((*(*all_paths)[agent_id])[i].single == false)  // Not a 1-width MDD level.
                    continue;
                else if ((*(*all_paths)[agent_id])[i - 1].builtMDD &&
                         (*(*all_paths)[agent_id])[i - 1].single) {  // Safe because i > 0
                    // This level is narrow and the previous one too - add a positive edge constraint between their
                    // nodes. It's preferable over two positive constraints for some reason.

                    ret += pop_lpa_constraint(agent_id, positive_constraints[agent_id].back(), cat);
                } else if (i < timestep && !(*(*all_paths)[agent_id])[i + 1].single) {
                    // This level is narrow, and the *next* one isn't (so we didn't already add a positive edge
                    // constraint between them) - add a positive vertex constraint for this level's node

                    ret += pop_lpa_constraint(agent_id, positive_constraints[agent_id].back(), cat);
                }
            }
        }
    }
    return ret;
};

int ICBSNode::pop_lpa_constraint(int to_constrain, const Constraint& constraint, const ConflictAvoidanceTable* cat)
{
    const auto [loc1, loc2, timestep, positive_constraint] = constraint;  // NOT auto& because the constraint is about to be removed
    // TODO: Support positive constraints
    int generated_before = lpas[to_constrain]->allNodes_table.size();
    if (positive_constraint == false)  // negative constraint
    {
        if (loc2 == -1)  // vertex constraint
            lpas[to_constrain]->popVertexConstraint(loc1, timestep, *cat);
        else
            lpas[to_constrain]->popEdgeConstraint(loc1, loc2, timestep, *cat);
    } else {
        // TODO: When LPA* gets support for positive constraints, handle it here
        // Replanning for the other agents when a positive constraint is added happens elsewhere
    }

    return lpas[to_constrain]->allNodes_table.size() - generated_before;
};

uint32_t ICBSNode::get_up_and_down_distance(ICBSNode *other) {
    int lower_depth = std::min(depth, other->depth);
    uint32_t j;
    for (j = 0; j < lower_depth; ++j) {
        if ((branch & (0x8000000000000000U >> j)) != (other->branch & (0x8000000000000000U >> j))) {
            break;
        }
    }
    int lowest_common_ancestor_depth = j;
    return depth - lowest_common_ancestor_depth + other->depth - lowest_common_ancestor_depth;
}
