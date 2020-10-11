#include "ICBSSearch.h"
#include <filesystem>  // For exists, rename
#include <set>
#include <sstream>
#include <cstring>  // For memcpy
#include <cstdlib>  // For mkstemp, abs
#include <cstdio>  // For fopen, fdopen
#include <spdlog/spdlog.h>
//#include <random>  // For default_random_engine

//////////////////// HIGH LEVEL HEURISTICS ///////////////////////////
void ICBSSearch::remove_model_constraints(vector<vector<vector<GRBConstr>>>& Constraints, vector<vector<bool>>& CG, vector<bool>& nodeHasNontrivialEdges) {
    for (int i = 0; i < num_of_agents; ++i) {
        if (!nodeHasNontrivialEdges[i])
            continue;

        // Remove all edges connected to agent i
        for (int other_agent = 0; other_agent < CG[i].size(); ++other_agent) {
            if (CG[i][other_agent]) {
                for (auto constraint : Constraints[i][other_agent]) { // No need to work with references - GrbConstr objects are just pointers - cheap to copy
//                    cerr << "Removing constraint for agents " << i << " and " << other_agent << endl;
                    mvc_model.remove(constraint);
                }
                CG[other_agent][i] = false;
                // No need to clear the symmetric part of the matrix - we're not going to use it again
            }
        }
    }
//    mvc_model.reset(0);
}

void ICBSSearch::calc_mvc_mode_node_degrees(ICBSNode& curr, vector<int>& nodeDegrees) {
    vector<vector<bool>> countedEdges(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
        countedEdges[i].resize(num_of_agents, false);
    for (const auto& conf : curr.cardinalGoalConf)
    {
        const auto& [agent1, agent2, loc1, loc2, timestep] = *conf;
        if (countedEdges[agent1][agent2] == false) {
            nodeDegrees[agent1]++;
            nodeDegrees[agent2]++;
            countedEdges[agent1][agent2] = true;
            countedEdges[agent2][agent1] = true;
        }
    }
    for (const auto& conf : curr.cardinalConf)
    {
        const auto& [agent1, agent2, loc1, loc2, timestep] = *conf;
        if (countedEdges[agent1][agent2] == false) {
            nodeDegrees[agent1]++;
            nodeDegrees[agent2]++;
            countedEdges[agent1][agent2] = true;
            countedEdges[agent2][agent1] = true;
        }
    }
}

// And increment h for trivial CG edges
void ICBSSearch::add_mvc_model_constraints_from_conflicts(vector<shared_ptr<Conflict>>& conflicts, vector<vector<bool>>& CG,
                                                          vector<int>& CgNodeDegrees, vector<vector<vector<GRBConstr>>>& Constraints,
                                                          int& num_of_nontrivial_CG_edges, vector<bool>& nodeHasNontrivialEdges,
                                                          int& h) {
    for (const auto& conf : conflicts)
    {
        const auto& [agent1, agent2, loc1, loc2, timestep] = *conf;
        bool alreadySeenConflictBetweenTheseTwo = CG[agent1][agent2];  // The two agents may have more than one cardinal conflict between them

        if (!alreadySeenConflictBetweenTheseTwo) {  // No point in adding the same mvc model constraint twice
            if ((CgNodeDegrees[agent1] != 1) || (CgNodeDegrees[agent2] != 1)) {
                GRBConstr constraint =  mvc_model.addConstr(mvc_vars[agent1] + mvc_vars[agent2] >= 1);
                Constraints[agent1][agent2].push_back(constraint);
                Constraints[agent2][agent1].push_back(constraint);
                num_of_nontrivial_CG_edges++;
                nodeHasNontrivialEdges[agent1] = true;
                nodeHasNontrivialEdges[agent2] = true;
            }
            else
                h++;

            CG[agent1][agent2] = true;
            CG[agent2][agent1] = true;
        }
    }
}

void ICBSSearch::re_add_mvc_model_constraints_of_agent(vector<shared_ptr<Conflict>>& conflicts, int i,
                                                       vector<vector<vector<GRBConstr>>>& Constraints) {
    for (const auto &conf: conflicts) {
        const auto& [agent1, agent2, loc1, loc2, timestep] = *conf;
        if ((agent1 == i) || (agent2 == i)) {  // The edge involves our agent
            GRBConstr constraint = mvc_model.addConstr(mvc_vars[agent1] + mvc_vars[agent2] >= 1);

            Constraints[agent1][agent2].push_back(constraint);
            Constraints[agent2][agent1].push_back(constraint);
        }
    }
}

// compute a heuristic estimate for the high-level node, and also choose a more promising conflict is one is found
// TODO: Separate the second part into a new function
#ifndef USE_GUROBI
int ICBSSearch::computeHeuristic(ICBSNode& curr)
{
	// Build the cardinal conflict graph
	vector<vector<bool>> CG(num_of_agents);
	int num_of_CGnodes = 0, num_of_CGedges = 0;
	for (int i = 0; i < num_of_agents; i++)
		CG[i].resize(num_of_agents, false);
	for (const auto& conf : curr.cardinalConf)
	{
		const auto& [agent1, agent2, loc1, loc2, timestep] = *conf;
		if (!CG[agent1][agent2])
		{
			CG[agent1][agent2] = true;
			CG[agent2][agent1] = true;
			num_of_CGedges++;
		}
	}

	if (num_of_CGedges < 2)
		return num_of_CGedges;

	// Compute #CG nodes that have edges
	for (int i = 0; i < num_of_agents; i++)
	{
		for (int j = 0; j < num_of_agents; j++)
		{
			if (CG[i][j])
			{
				num_of_CGnodes++;
				break;
			}
		}
	}

	if (num_of_CGedges == 2 && num_of_CGnodes == 4)  // Two disjoint edges
	    return 2;
    if (num_of_CGedges == 2 && num_of_CGnodes == 3)  // Two edges that share a vertex
        return 1;

	// Minimum Vertex Cover
	if (curr.parent == NULL) // root node of CBS tree
	{
		int i = 1;
		while (!KVertexCover(CG, num_of_CGnodes, num_of_CGedges, i))
			i++;
		return i;
	}
	else if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, curr.parent->h_val - 1))
		return curr.parent->h_val - 1;
	else if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, curr.parent->h_val))
		return curr.parent->h_val;
	else
		return curr.parent->h_val + 1;
}
#else
int ICBSSearch::computeHeuristic(ICBSNode& curr)
{
    if (curr.cardinalGoalConf.empty() && curr.cardinalConf.empty())
        return 0;
    if (curr.cardinalGoalConf.size() + curr.cardinalConf.size() == 1)
        return 1;

    // Build the cardinal conflict graph
    vector<vector<bool>> CG(num_of_agents);  // Including trivial edges
    vector<vector<vector<GRBConstr>>> Constraints(num_of_agents);  // There can be both a cardinal goal conflict and
                                                                   // a regular cardinal conflict between the same two agents,
                                                                   // creating multiple different constraints on the pair of agents
    vector<int> CgNodeDegrees(num_of_agents, 0);  // Including trivial edges, multiple edges between edges only counted once
    vector<bool> nodeHasNontrivialEdges(num_of_agents, false);
#ifndef GUROBI_WITH_GOAL_CONSTRAINTS
#else
    vector<int> latestGoalConflictTimestep(num_of_agents, 0);  // For an optimization of an easy case
    vector<vector<int>> bestCostIncreaseForFirstAgent(num_of_agents);
#endif
    int num_of_nontrivial_CG_edges = 0;
    for (int i = 0; i < num_of_agents; i++) {
        CG[i].resize(num_of_agents, false);
        Constraints[i].resize(num_of_agents);
#ifndef GUROBI_WITH_GOAL_CONSTRAINTS
#else
        bestCostIncreaseForFirstAgent[i].resize(num_of_agents, 0);
#endif
    }
    auto orig_conflict = curr.conflict;
    int h = 0;
    calc_mvc_mode_node_degrees(curr, CgNodeDegrees);
    // Handle cardinal goal conflicts
#ifndef GUROBI_WITH_GOAL_CONSTRAINTS
    // Treat cardinal goal conflicts as regular cardinal conflicts
    add_mvc_model_constraints_from_conflicts(curr.cardinalGoalConf, CG, CgNodeDegrees, Constraints,
                                             num_of_nontrivial_CG_edges,
                                             nodeHasNontrivialEdges, h);
#else
    for (const auto& conf: curr.cardinalGoalConf) {
        // Find the latest timestep of a cardinal goal conflict between each two agents
        const auto& [agent1, agent2, loc1, loc2, timestep] = *conf;
        int cost_increase = timestep + 1 - ((*curr.all_paths)[agent2]->size() - 1);  // conflict happens _after_ agent2 reaches its goal
                                                                                     // (because constraints are on the same time as the conflict,
                                                                                     // or earlier due to propagation)
        if (cost_increase > bestCostIncreaseForFirstAgent[agent2][agent1])
            bestCostIncreaseForFirstAgent[agent2][agent1] = cost_increase;
        if (timestep > latestGoalConflictTimestep[agent2])
            latestGoalConflictTimestep[agent2] = timestep;
    }
    for (const auto& conf: curr.cardinalGoalConf) {
        const auto& [agent1, agent2, loc1, loc2, timestep] = *conf;
        int cost_increase = bestCostIncreaseForFirstAgent[agent2][agent1];
        bool alreadySeenConflictBetweenTheseTwo = CG[agent1][agent2];  // The two agents may have more than one cardinal goal conflict between them,
                                                                       // but we're only adding the constraint for the latest one
        if (!alreadySeenConflictBetweenTheseTwo) {  // No point in adding the same constraint twice
            if ((CgNodeDegrees[agent1] != 1) || (CgNodeDegrees[agent2] != 1)) {
                GRBConstr constraint = mvc_model.addConstr(
                        mvc_vars[agent1] * cost_increase + mvc_vars[agent2] >= cost_increase);
                Constraints[agent1][agent2].push_back(constraint);
                Constraints[agent2][agent1].push_back(constraint);
                num_of_nontrivial_CG_edges++;
                nodeHasNontrivialEdges[agent1] = true;
                nodeHasNontrivialEdges[agent2] = true;
            }
            else
                h++;

            CG[agent1][agent2] = true;
            CG[agent2][agent1] = true;
        }
    }
#endif
    // Handle f-cardinal, semi-f-cardinal, and cardinal conflicts
    add_mvc_model_constraints_from_conflicts(curr.cardinalConf, CG, CgNodeDegrees, Constraints,
                                             num_of_nontrivial_CG_edges,
                                             nodeHasNontrivialEdges, h);

    if (num_of_nontrivial_CG_edges == 0)  // Then there are no f-cardinal conflicts to be found
        return h;

    if ((num_of_nontrivial_CG_edges == 2) &&  // Exactly three vertices and two edges - two edges that share a single vertex.
                                              // (We already handled trivial edges earlier)
        !preferFCardinals)  // Otherwise we need to run the model
    {
#ifndef GUROBI_WITH_GOAL_CONSTRAINTS

            h += 1;
#else
            // Find the middle agent
            for (int i = 0; i < num_of_agents; i++) {
                if (CgNodeDegrees[i] == 2) {  // That's the middle agent
                    if (latestGoalConflictTimestep[i] >= (*curr.all_paths)[i]->size() - 1 + 1)  // One of its conflicts is a goal conflict
                        h += 2;  // Assign 1 to the agent that conflicts with the middle agent after the middle agent
                                 // reached its goal, and 1 to one of the other agents the isn't at its goal when it
                                 // conflicts with the other
                    else
                        h += 1;
                    break;
                }
            }
#endif
        remove_model_constraints(Constraints, CG, nodeHasNontrivialEdges);
        return h;
    }


    mvc_model.optimize();
    // Get the size of the mvc
    int nontrivial_mvc_size = mvc_model.getObjective().getValue();
    h += nontrivial_mvc_size;

    if (preferFCardinals && num_of_nontrivial_CG_edges != 0  // Look for an f-cardinal conflict among the cardinal conflicts,
                                                             // there's a chance of finding one (removing trivial edges always decreases the h by 1).
//        && (eagerly_search_for_f_cardinals || curr.cardinalGoalConf.empty())  // Otherwise just stick with the at-least-semi-f-cardinal cardinal-goal conflict that we have
        ) {
        // Find an edge that increases the cost without decreasing the size of the mvc -
        // splitting on it will increase the F of one of the children, because that node's cost will increase (by 1, usually)
        // and its h will stay the same. That's better than splitting on conflicts where the F of both children stays the same.

#ifndef GUROBI_WITH_GOAL_CONSTRAINTS
        int best_h_decrease = 1;  // When removing a vertex, the size of the mvc can only decrease by at most 1 (a lower decrease is better)
#else
        // We could have a larger than 1 cost decrease from removing a vertex. For example, if 10 agents
        // have a conflict with the same agent 3 timesteps after it reaches its goal, removing that agent would decrease
        // the h by 4.
        int best_h_decrease = numeric_limits<int>::max();  // A lower decrease is better
#endif
        bool f_cardinal_found = false;
        for (int i = 0; i < num_of_agents && !f_cardinal_found; ++i) {
            if (nodeHasNontrivialEdges[i] == false)  // Trivial edges always increase the h by exactly 1
                continue;

            // Remove all nontrivial edges connected to agent i
            for (int other_agent = 0; other_agent < num_of_agents; ++other_agent) {
                if (other_agent == i)
                    continue;
                if (nodeHasNontrivialEdges[other_agent] == false)  // Trivial edges always increase the h by exactly 1
                    continue;

                for (auto constraint : Constraints[i][other_agent]) { // No need to work with references - GrbConstr objects are just pointers - cheap to copy
                    mvc_model.remove(constraint);
                }
                Constraints[i][other_agent].clear();
                Constraints[other_agent][i].clear();
            }

#ifndef GUROBI_WITH_GOAL_CONSTRAINTS
#else
            // In theory, we would need to re-add the agent's cardinal-goal edges where it is the one that is at its goal
            // with a 1-decrease on the coefficient of the other agent, but since we're not doing this if there are any
            // cardinal goal conflicts, this isn't needed
            // FIXME: WE ARE NOW DOING THIS, so do it
#endif

            // Re-optimize
            mvc_model.optimize();
            int nontrivial_mvc_size_without_the_edges_of_the_node = mvc_model.getObjective().getValue();
            int h_decrease = nontrivial_mvc_size - nontrivial_mvc_size_without_the_edges_of_the_node;
            if (h_decrease < best_h_decrease) {
                best_h_decrease = h_decrease;
            }

            if (h_decrease == 0) {
                for (const auto& conf: curr.cardinalGoalConf) {
                    const auto&[agent1, agent2, loc1, loc2, timestep] = *conf;
                    if (agent1 == i) {  // Our agent is the non-goal side - we found a full f-cardinal conflict!
                                        // Can't improve that, skip adding the edges back - we're going to remove them all anyway.
                        f_cardinal_found = true;
                        curr.conflict = conf;
                        ++f_cardinal_conflicts_found;
#ifndef GUROBI_WITH_GOAL_CONSTRAINTS
                        ++h;
                        break;
#endif
                    }
                }
                if (curr.cardinalGoalConf.empty()) {  // No cardinal-goal conflicts => in particular, the chosen conflict isn't cardinal-goal.
                                                      // Cardinal-goal conflicts are better than semi-f-cardinal conflicts
                    for (const auto& conf: curr.cardinalConf) {
                        const auto&[agent1, agent2, loc1, loc2, timestep] = *conf;
                        if ((agent1 == i) ||
                            (agent2 == i)) {  // The edge involves our agent - we found a semi-f-cardinal conflict!
                                              // Can't improve that, skip adding the edges back - we're going to remove them all anyway.
                            curr.conflict = conf;  // Arbitrarily choose the agent's first semi-f-cardinal conflict
                            f_cardinal_found = true;
                            curr.branch_on_first_agent = (agent1 == i);  // Branch on the this agent - adding a positive constraint on
                                                                         // it might increase the cost of other agents besides the conflicting agent and
                                                                         // make the conflict fully f-cardinal
                            ++semi_f_cardinal_conflicts_found;
                            break;
                        }
                    }
                }
            }
            // Add all the edges back (since the agent has a nontrivial edge, all its edges are nontrivial so we should add them all)
            if (!f_cardinal_found) {
#ifndef GUROBI_WITH_GOAL_CONSTRAINTS
                re_add_mvc_model_constraints_of_agent(curr.cardinalGoalConf, i, Constraints);
#else
                re_add_mvc_model_goal_constraints_of_agent(curr.cardinalGoalConf, i, Constraints);
#endif
                re_add_mvc_model_constraints_of_agent(curr.cardinalConf, i, Constraints);
            }
        }

        if (f_cardinal_found) {
            if (curr.conflict == orig_conflict)
                spdlog::info("The already chosen conflict {}"
                             " will increase the F value of at least one of its children", *curr.conflict);
            else
                spdlog::info("Switched from conflict {} to conflict {}"
                             " - it will increase the F value of at least one of its children", *orig_conflict, *curr.conflict);
        }
    }

    remove_model_constraints(Constraints, CG, nodeHasNontrivialEdges);

    return h;
}
#endif

// return true if there exists a k-vertex cover
bool ICBSSearch::KVertexCover(const vector<vector<bool>>& CG, int num_of_CGnodes, int num_of_CGedges, int k)
{
	if (num_of_CGedges == 0)
		return true;
	else if (num_of_CGedges > k * num_of_CGnodes - k)
		return false;

	vector<int> node(2);
	bool flag = true;
	for (int i = 0; i < num_of_agents - 1 && flag; i++) // to find an edge
	{
		for (int j = i + 1; j < num_of_agents && flag; j++)
		{
			if (CG[i][j])
			{
				node[0] = i;
				node[1] = j;
				flag = false;
			}
		}
	}
	for (int i = 0; i < 2; i++)
	{
		vector<vector<bool>> CG_copy(num_of_agents);
		for (int j = 0; j < num_of_agents; j++)
		{
			CG_copy[j].resize(num_of_agents);
			CG_copy.assign(CG.cbegin(), CG.cend());
		}
		int num_of_CGedges_copy = num_of_CGedges;
		for (int j = 0; j < num_of_agents; j++)
		{
			if (CG_copy[node[i]][j])
			{
				CG_copy[node[i]][j] = false;
				CG_copy[j][node[i]] = false;
				num_of_CGedges_copy--;
			}
		}
		if (KVertexCover(CG_copy, num_of_CGnodes - 1, num_of_CGedges_copy, k - 1))
			return true;
	}
	return false;
}

//////////////////// CONSTRAINTS ///////////////////////////

// collect constraints from ancestors
void ICBSSearch::collectConstraints(ICBSNode* curr, std::list<pair<int, Constraint>> &constraints)
{
	while (curr != nullptr)
	{
		for (const auto& con : curr->positive_constraints[curr->agent_id])
		{
			constraints.push_back(make_pair(curr->agent_id, con));
		}
		for (const auto& con : curr->negative_constraints[curr->agent_id])
		{
			constraints.push_back(make_pair(curr->agent_id, con));
		}
		curr = curr->parent;
	}
	// TODO: Use this function.
}

// build the constraint table for replanning agent <agent_id>,
// and find the two closest landmarks for agent <agent_id> that have the new constraint at time step <newConstraintTimestep> between them.
// TODO: Consider splitting into two functions
// update cons_table: cons_table[time_step, location].vertex or .edge = true iff they're a negative constraint for the agent
// update start and goal: the two landmarks for the agent such that its path between them violates the newly posted constraint (at <newConstraintTimestep>)
// return last goal constraint timestep - the time of the last constraint on an agent's goal
int ICBSSearch::buildConstraintTable(ICBSNode* curr, int agent_id, int newConstraintTimestep,
                                     XytHolder<ConstraintState>& cons_table,
                                     pair<int,int>& start, pair<int,int>& goal)
{
	int lastGoalConsTimestep = -1;

	// extract all constraints on agent_id
	list < Constraint > constraints_negative;
	while (curr != nullptr)
	{
		for (const auto& con: curr->negative_constraints[agent_id]) {
			const auto& [loc1, loc2, constraint_timestep, positive_constraint] = con;
			constraints_negative.push_back(con);
			if (loc1 == goal.first && loc2 < 0 && lastGoalConsTimestep < constraint_timestep)
				lastGoalConsTimestep = constraint_timestep;
		}

		for (int j = 0; j < num_of_agents ; ++j) {
			if (j == agent_id) // For our agent, those are landmarks
			{
				for (const auto& con: curr->positive_constraints[j]) {
					const auto& [loc1, loc2, constraint_timestep, positive_constraint] = con;

					if (loc2 < 0) // vertex constraint
					{
						if (start.second < constraint_timestep && constraint_timestep < newConstraintTimestep)  // This landmark is between (start.second, newConstraintTimestep)
						{ // update start
							start.first = loc1;
							start.second = constraint_timestep;
						} else if (newConstraintTimestep <= constraint_timestep && constraint_timestep < goal.second)  // the landmark is between [newConstraintTimestep, goal.second)
						{ // update goal
							goal.first = loc1;
							goal.second = constraint_timestep;
							// If the landmark is at the same timestep as the new constraint, planning the new path will surely fail
						}
					} else // edge constraint, viewed as two landmarks on the from-vertex and the to-vertex
					{
						if (start.second < constraint_timestep && constraint_timestep < newConstraintTimestep)  // the second landmark is between (start.second, newConstraintTimestep)
						{ // update start
							start.first = loc2;
							start.second = constraint_timestep;
						} else if (newConstraintTimestep <= constraint_timestep - 1 && constraint_timestep - 1 < goal.second)  // the first landmark is between [newConstraintTimestep, goal.second)
						{ // update goal
							goal.first = loc1;
							goal.second = constraint_timestep - 1;
						}
					}
//					constraints_positive.push_back(con);
				}
			}
			else {  // for other agents, and specifically our agent, it is equivalent to a negative constraint
				for (const auto& con: curr->positive_constraints[j]) {
					const auto& [loc1, loc2, constraint_timestep, positive_constraint] = con;
					if (loc1 == goal.first && loc2 < 0 && lastGoalConsTimestep < constraint_timestep) {
						lastGoalConsTimestep = constraint_timestep;
					}
					constraints_negative.push_back(con);
				}
			}
		}
		curr = curr->parent;
	}
	if (lastGoalConsTimestep > goal.second) // because we only need to replan paths before goal.second
		lastGoalConsTimestep = -1;

	// Build the agent's negative constraints table
	for (const auto& conflict : constraints_negative)
	{
		const auto& [loc1, loc2, constraint_timestep, positive_constraint] = conflict;
		if (!positive_constraint)  // So it's a negative constraint on this agent specifically
		{
			if (loc2 < 0)  // vertex constraint
			{
			    auto [found, state] = cons_table.get(loc1, constraint_timestep);
			    if (!found) {
			        state = new ConstraintState;
			        cons_table.set(loc1, constraint_timestep, state);
			    }
			    state->vertex = true;
			}
			else // edge constraint
			{
				for(int i = 0; i < MapLoader::valid_moves_t::WAIT_MOVE; i++)
				{
					if (loc2 - loc1 == moves_offset[i])
					{
						auto [found, state] = cons_table.get(loc2, constraint_timestep);
						if (!found) {
						    state = new ConstraintState();
						    cons_table.set(loc2, constraint_timestep, state);
						}
						state->edge[i] = true;
						break;
					}
				}
			}
		}
		else if (loc2 < 0)  // positive vertex constraint for another agent
		{
			auto [found, state] = cons_table.get(loc1, constraint_timestep);
			if (!found) {
				state = new ConstraintState;
				cons_table.set(loc1, constraint_timestep, state);
			}
			state->vertex = true;
		}
		else  // positive edge constraint for another agent
		{
			auto [found, state] = cons_table.get(loc1, constraint_timestep - 1);
			if (!found) {
				state = new ConstraintState;
				cons_table.set(loc1, constraint_timestep - 1, state);
			}
			state->vertex = true;

			auto [found2, state2] = cons_table.get(loc2, constraint_timestep);
			if (!found) {
				state2 = new ConstraintState;
				cons_table.set(loc2, constraint_timestep, state2);
			}
			state2->vertex = true;

			for (int i = 0; i < MapLoader::valid_moves_t::WAIT_MOVE; i++)
			{
				if (loc1 - loc2 == moves_offset[i]) {
                    auto [found_edge, state_edge] = cons_table.get(loc1, constraint_timestep);
                    if (!found) {
                        state_edge = new ConstraintState;
                        cons_table.set(loc1, constraint_timestep, state_edge);
                    }
                    state->edge[i] = true;
                    break;
                }
			}
		}
	}
	return lastGoalConsTimestep;
}

// build conflict avoidance table
// update cat: Set cat[time_step][location].vertex or .edge[direction] to the number of other agents that plan to use it
void ICBSSearch::buildConflictAvoidanceTable(const ICBSNode &node, int exclude_agent, ConflictAvoidanceTable &cat)
{
	if (node.makespan == 0)
		return;
	for (int ag = 0; ag < num_of_agents; ag++)
	{
		if (ag != exclude_agent &&
			(*node.all_paths)[ag]->size() != 0  // Happens when computing the initial paths for the root node
			)
		{
			addPathToConflictAvoidanceTable(*(*node.all_paths)[ag], cat, ag);
		}
	}
}

void ICBSSearch::addPathToConflictAvoidanceTable(Path &path, ConflictAvoidanceTable &cat, int agent_id) {
    if (spdlog::default_logger()->level() <= spdlog::level::debug) {
        ostringstream msg;
        msg << "Adding the path (size " << path.size() - 1 << ") of agent " << agent_id << " to the CAT: ";
        if (path.size() < 100)
        {
            int t = 0;
            for ( ; t < path.size() - 1; t++)
                msg << t << ": (" << path[t].location / num_map_cols << ","
                          << path[t].location % num_map_cols << "), ";
            msg << t << ": (" << path[t].location / num_map_cols << ","
                      << path[t].location % num_map_cols << ")";
        } else
            msg << "(path too long to print)";
        spdlog::debug(msg.str());
    }

    auto cat_start = std::clock();
    auto wall_cat_start = std::chrono::system_clock::now();
    int first_wait_at_goal_timestep = path.size();
    for (int j = path.size() - 2; j >= 0 ; --j) {
        if (path[j].location == path[j + 1].location)
            --first_wait_at_goal_timestep;
        else
            break;
    }
    cat.add_wait_at_goal(first_wait_at_goal_timestep, path.back().location);
    for (size_t timestep = 1; timestep < first_wait_at_goal_timestep; timestep++)
    {
        int to = path[timestep].location;
        int from = path[timestep - 1].location;
        cat.add_action(timestep, from, to);
    }
    cat_runtime += std::clock() - cat_start;
    wall_cat_runtime += std::chrono::system_clock::now() - wall_cat_start;
}

void ICBSSearch::removePathFromConflictAvoidanceTable(Path &path, ConflictAvoidanceTable &cat, int agent_id) {
    if (spdlog::default_logger()->level() <= spdlog::level::debug) {
        ostringstream msg;
        msg << "Removing the path (size " << path.size() - 1 << ") of agent " << agent_id << " from the CAT: ";

        if (path.size() < 100) {
            int t = 0;
            for ( ; t < path.size() - 1; t++)
                msg << t << ": (" << path[t].location / num_map_cols << ","
                    << path[t].location % num_map_cols << "), ";
            msg << t << ": (" << path[t].location / num_map_cols << ","
                << path[t].location % num_map_cols << ")";
        } else
            msg << "(path too long to print)";
        spdlog::debug(msg.str());
    }

	auto cat_start = std::clock();
	auto wall_cat_start = std::chrono::system_clock::now();
	int first_wait_at_goal_timestep = path.size();
	cat.remove_wait_at_goal(first_wait_at_goal_timestep, path.back().location);
	for (size_t timestep = 1; timestep < path.size(); timestep++)
	{
		int to = path[timestep].location;
		int from = path[timestep - 1].location;
		cat.remove_action(timestep, from, to);
	}
	cat_runtime += std::clock() - cat_start;
	wall_cat_runtime += std::chrono::system_clock::now() - wall_cat_start;
}

//////////////////// CONFLICTS ///////////////////////////
// copy conflicts from the parent node if the paths of both agents in the conflict remain unchanged from the parent
void ICBSSearch::copyConflictsFromParent(ICBSNode& curr)
{
	auto hl_node_verification_start = std::clock();
	auto wall_hl_node_verification_start = std::chrono::system_clock::now();

	// Mark which paths remain unchanged from the parent node
	vector<bool> unchanged(num_of_agents, true);
	for (const auto & id_and_new_path : curr.new_paths) {
	    const auto& [agent_id, path] = id_and_new_path;
	    unchanged[agent_id] = false;
	    if (path.size() == 1 && path.back().location < 0)  // Pathfinding was delayed for the agent -
	                                                       // temporarily keep its conflicts so as not to
	                                                       // give the node priority in FOCAL for work we haven't
	                                                       // done yet (resolving them).
	        unchanged[agent_id] = true;
	}
	// Copy conflicts of agents both whose paths remain unchanged
	copyConflicts(unchanged, curr.parent->cardinalGoalConf, curr.cardinalGoalConf);
	copyConflicts(unchanged, curr.parent->cardinalConf, curr.cardinalConf);
	copyConflicts(unchanged, curr.parent->semiCardinalGoalConf, curr.semiCardinalGoalConf);
	copyConflicts(unchanged, curr.parent->semiCardinalConf, curr.semiCardinalConf);
	copyConflicts(unchanged, curr.parent->nonCardinalConf, curr.nonCardinalConf);
	copyConflicts(unchanged, curr.parent->unknownConf, curr.unknownConf);

    hl_node_verification_runtime += std::clock() - hl_node_verification_start;
    wall_hl_node_verification_runtime += std::chrono::system_clock::now() - wall_hl_node_verification_start;
}

void ICBSSearch::clearConflictsOfAgent(ICBSNode &curr, int ag)
{
    auto hl_node_verification_start = std::clock();
    auto wall_hl_node_verification_start = std::chrono::system_clock::now();

	// Mark which paths remain unchanged from the parent node
	bool unchanged[num_of_agents];
	for (int j = 0; j < num_of_agents; ++j) {
		unchanged[j] = true;
	}
	unchanged[ag] = false;

	// Keep conflicts of agents both whose paths remain unchanged
	clearConflictsOfAffectedAgents(unchanged, curr.cardinalGoalConf);
	clearConflictsOfAffectedAgents(unchanged, curr.cardinalConf);
	clearConflictsOfAffectedAgents(unchanged, curr.semiCardinalGoalConf);
	clearConflictsOfAffectedAgents(unchanged, curr.semiCardinalConf);
	clearConflictsOfAffectedAgents(unchanged, curr.nonCardinalConf);
	clearConflictsOfAffectedAgents(unchanged, curr.unknownConf);

    hl_node_verification_runtime += std::clock() - hl_node_verification_start;
    wall_hl_node_verification_runtime += std::chrono::system_clock::now() - wall_hl_node_verification_start;
}

// copy conflicts from "from" to "to" if the paths of both agents remain unchanged
// according to the given array
void ICBSSearch::copyConflicts(const vector<bool>& unchanged,
	const vector<std::shared_ptr<Conflict>>& from, vector<std::shared_ptr<Conflict>>& to)
{
	for (const auto& conflict : from)
	{
		const auto& [agent1, agent2, loc1, loc2, timestep] = *conflict;
		if (unchanged[agent1] && unchanged[agent2])
			to.push_back(conflict);
	}
}

// Of changed agents
void ICBSSearch::clearConflictsOfAffectedAgents(bool *unchanged,
												vector<std::shared_ptr<Conflict>> &lst)
{
	auto it = lst.begin();
	while (it != lst.end()) {
		const auto& [agent1, agent2, loc1, loc2, timestep] = **it;
		if (!unchanged[agent1] || !unchanged[agent2])
			it = lst.erase(it);
		else
			++it;

	}
}

// find all conflict in the solution of the node
// assumes all_paths is populated correctly
void ICBSSearch::findConflicts(ICBSNode& curr)
{
	auto hl_node_verification_start = std::clock();
	auto wall_hl_node_verification_start = std::chrono::system_clock::now();
	if (curr.parent != nullptr)  // not root node
	{
		// detect conflicts that occur on the new planned paths
		vector<bool> detectedTheConflictsOfThisAgent(num_of_agents, false);
		for (const auto& pair : curr.new_paths)
		{
			const auto& [a1, path] = pair;
			detectedTheConflictsOfThisAgent[a1] = true;
			for (int a2 = 0; a2 < num_of_agents; a2++)
			{
				if (detectedTheConflictsOfThisAgent[a2])
					continue;
				findConflicts(curr, a1, a2);
			}
		}
	}
	else  // root node
	{  // detect conflicts among all paths
		for(int a1 = 0; a1 < num_of_agents ; a1++)
		{
			for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
			{
				findConflicts(curr, a1, a2);
			}
		}
	}
    hl_node_verification_runtime += std::clock() - hl_node_verification_start;
    wall_hl_node_verification_runtime += std::chrono::system_clock::now() - wall_hl_node_verification_start;
}

void ICBSSearch::findConflicts(ICBSNode& curr, vector<int>& agents_with_new_paths) {
    // detect conflicts that occur on the new planned paths
    auto hl_node_verification_start = std::clock();
    auto wall_hl_node_verification_start = std::chrono::system_clock::now();

    vector<bool> detectedTheConflictsOfThisAgent(num_of_agents, false);
    for (auto a1 : agents_with_new_paths) {
        detectedTheConflictsOfThisAgent[a1] = true;
        for (int a2 = 0; a2 < num_of_agents; a2++) {
            if (detectedTheConflictsOfThisAgent[a2])
                continue;
            findConflicts(curr, a1, a2);
        }
    }

    hl_node_verification_runtime += std::clock() - hl_node_verification_start;
    wall_hl_node_verification_runtime += std::chrono::system_clock::now() - wall_hl_node_verification_start;
}

// find conflicts between paths of agents a1 and a2
void ICBSSearch::findConflicts(ICBSNode &curr, int a1, int a2)
{
	Path& a1_path = *(*curr.all_paths)[a1];
	Path& a2_path = *(*curr.all_paths)[a2];
	size_t min_path_length = a1_path.size() < a2_path.size() ? a1_path.size() : a2_path.size();
	for (int timestep = 0; timestep < min_path_length; timestep++)
	{
		int loc1 = a1_path[timestep].location;
		int loc2 = a2_path[timestep].location;
		if (loc1 == loc2)
		{
			curr.unknownConf.push_back(std::make_shared<Conflict>(a1, a2, loc1, -1, timestep));
		}
		else if (timestep < min_path_length - 1
			&& loc1 == a2_path[timestep + 1].location
			&& loc2 == a1_path[timestep + 1].location)
		{
			curr.unknownConf.push_back(std::make_shared<Conflict>(a1, a2, loc1, loc2, timestep + 1)); // edge conflict
		}
	}
	if (a1_path.size() != a2_path.size())  // check whether there are conflicts that occur after one agent reaches its goal
	{
		int agent_with_shorter_path = a1_path.size() < a2_path.size() ? a1 : a2;
		int agent_with_longer_path = a1_path.size() < a2_path.size() ? a2 : a1;
		Path& shorter_path = *(*curr.all_paths)[agent_with_shorter_path];
		Path& longer_path = *(*curr.all_paths)[agent_with_longer_path];
		int loc1 = shorter_path.back().location;
		for (int timestep = (int)min_path_length; timestep < longer_path.size(); timestep++)
		{
			int loc2 = longer_path[timestep].location;
			if (loc1 == loc2)
			{ // It's at least a semi cardinal conflict
				curr.unknownConf.push_back(std::make_shared<Conflict>(
				    agent_with_longer_path, agent_with_shorter_path, loc1, -1, timestep));
			}
		}
	}
}

// Classify conflicts into the different cardinality classes and populate the node's conflict lists with them.
// If a CBS heuristic isn't used, return the first (f-)cardinal conflict that was found immediately, without finishing
// the classification of conflicts.
// The CAT is only used if LPA* is used to build the MDDs for checking if conflicts are cardinal
void ICBSSearch::classifyConflicts(ICBSNode &node, ConflictAvoidanceTable* cat /* = nullptr*/)
{
	if (node.cardinalGoalConf.empty() &&
		node.cardinalConf.empty() && node.semiCardinalGoalConf.empty() &&
		node.semiCardinalConf.empty() && node.nonCardinalConf.empty() && node.unknownConf.empty())
		return; // No conflict

	// Classify all conflicts in unknownConf
	while (!node.unknownConf.empty())
	{
		if (std::clock() > start + time_limit * CLOCKS_PER_SEC)  // timeout
		{
			spdlog::warn("Timed out classifying conflicts!");
			break;
		}
		std::shared_ptr<Conflict> con = node.unknownConf.back();
		const auto& [agent1, agent2, loc1, loc2, timestep] = *con;

		Path & a1_path = *(*node.all_paths)[agent1];
		Path & a2_path = *(*node.all_paths)[agent2];

		bool cardinal1, cardinal2;
		bool goal_conflict1 = false;
		bool goal_conflict2 = false;
		if (loc2 >= 0) // Edge conflict (no need to check if it happens after one agent has already reached its goal)
		{
			bool success = buildMDD(node, agent1, timestep, 0, cat);  // Build an MDD for the agent's current cost, unless we already know if it's narrow at <timestep>
			if (!success)
			    break;
			success = buildMDD(node, agent2, timestep, 0, cat);
            if (!success)
                break;
            // TODO: If we can already tell the conflict is at most semi-g-cardinal and won't factor into the heuristic,
            //       Skip building the other MDD and push the conflict into a semi-g-cardinal-or-non-cardinal vector.
            //       We might never need the other agent's MDD.
			success = buildMDD(node, agent1, timestep - 1, 0, cat);  // Build an MDD for the agent's current cost, unless we already know if it's narrow at <timestep - 1>
            if (!success)
                break;
			success = buildMDD(node, agent2, timestep - 1, 0, cat);
            if (!success)
                break;
			cardinal1 = a1_path[timestep].single && a1_path[timestep - 1].single;
			cardinal2 = a2_path[timestep].single && a2_path[timestep - 1].single;
		}
		else // vertex conflict
		{
			if (timestep >= a1_path.size()) {
				goal_conflict1 = true;
				cardinal1 = true;
			}
			else
			{
				bool success = buildMDD(node, agent1, timestep, 0, cat);
                if (!success)
                    break;
				cardinal1 = a1_path[timestep].single;
                // TODO: If we can already tell the conflict is at most semi-g-cardinal and won't factor into the heuristic,
                //       Skip building the other MDD and push the conflict into a semi-g-cardinal-or-non-cardinal vector.
			}
			if (timestep >= a2_path.size()) {
				goal_conflict2 = true;
				cardinal2 = true;
			}
			else
			{
				bool success = buildMDD(node, agent2, timestep, 0, cat);
                if (!success)
                    break;
				cardinal2 = a2_path[timestep].single;
			}
		}

		if (cardinal1 && cardinal2)
		{
			if (goal_conflict1 || goal_conflict2) {
				if (goal_conflict2) {
					node.cardinalGoalConf.push_back(con);
					++cardinal_goal_conflicts_found;
				} else {  // then goal_conflict1 - make sure the agent with the lower f-increase is first
					if (loc2 < 0) {
                        node.cardinalGoalConf.push_back(
                                std::make_shared<Conflict>(agent2, agent1, loc1, -1, timestep));
                        ++cardinal_goal_conflicts_found;
                    }
					else {
                        node.cardinalGoalConf.push_back(
                                std::make_shared<Conflict>(agent2, agent1, loc2, loc1, timestep));
                        ++cardinal_goal_conflicts_found;
                    }
				}

				if (HL_heuristic == highlevel_heuristic::NONE && preferGoalConflicts)  // Found a cardinal goal conflict and they're not used to
                                                                                       // compute heuristics. Return it immediately.
                                                                                       // FIXME: Don't we need to find all of them so we can choose the one with the highest cost increase for the second agent?
				{
					conflictType = conflict_type::CARDINAL_GOAL;
					node.unknownConf.pop_back();  // Only pop it from unknown after we put it in the correct bin
					++cardinal_goal_conflicts_found;
					return;
				}
			}
			else {
				node.cardinalConf.push_back(con);
				++cardinal_conflicts_found;

				if (HL_heuristic == highlevel_heuristic::NONE && !preferGoalConflicts)  // Found a goal conflict and they're not used to complete heuristics. Return it immediately.
				{
					conflictType = conflict_type::CARDINAL_GOAL;
					node.unknownConf.pop_back();  // Only pop it from unknown after we put it in the correct bin
                    ++cardinal_goal_conflicts_found;
					return;
				}
			}

			if (HL_heuristic == highlevel_heuristic::NONE)  // Found a cardinal conflict and they're not used to complete heuristics. Return it immediately.
			{
				if (goal_conflict1 || goal_conflict2) {
					conflictType = conflict_type::CARDINAL_GOAL;
				}
				else
					conflictType = conflict_type::CARDINAL;
				node.unknownConf.pop_back();  // Only pop it from unknown after we put it in the correct bin
                ++cardinal_conflicts_found;
				return;
			}
		}
		else if (cardinal2)
		{
			if (goal_conflict2) {
                node.semiCardinalGoalConf.push_back(con);
                ++semi_cardinal_goal_conflicts_found;
            }
			else {
                node.semiCardinalConf.push_back(con);
                ++semi_cardinal_conflicts_found;
            }
		}
		else if (cardinal1)  // Make sure the non-cardinal agent is first
		{
			if (goal_conflict1) {
				if (loc2 >= 0) {
                    node.semiCardinalGoalConf.push_back(
                            std::make_shared<Conflict>(agent2, agent1, loc2, loc1, timestep));
                    ++semi_cardinal_goal_conflicts_found;
                }
				else {
                    node.semiCardinalGoalConf.push_back(
                            std::make_shared<Conflict>(agent2, agent1, loc1, loc2, timestep));
                    ++semi_cardinal_goal_conflicts_found;
                }
			}
			else
			{
				if (loc2 >= 0) {
                    node.semiCardinalConf.push_back(
                            std::make_shared<Conflict>(agent2, agent1, loc2, loc1, timestep));
                    ++semi_cardinal_conflicts_found;
                }
				else {
                    node.semiCardinalConf.push_back(
                            std::make_shared<Conflict>(agent2, agent1, loc1, loc2, timestep));
                    ++semi_cardinal_conflicts_found;
                }
			}
		}
		else
		{
			node.nonCardinalConf.push_back(con);
			++non_cardinal_conflicts_found;
		}

		node.unknownConf.pop_back();  // Only pop it from unknown after we put it in the correct bin
	}
}

// Primary priority - cardinal conflicts, then semi-cardinal and non-cardinal conflicts
std::shared_ptr<Conflict> ICBSSearch::getHighestPriorityConflict(ICBSNode &node)
{
    if (!node.cardinalGoalConf.empty() && preferGoalConflicts)
    {
        conflictType = conflict_type::CARDINAL_GOAL;
        node.left_cost_will_increase = ICBSNode::WillCostIncrease::YES;
        node.right_cost_will_increase = ICBSNode::WillCostIncrease::YES;
        node.left_f_will_increase = ICBSNode::WillCostIncrease::MAYBE;  // Hard to say no without generating the child,
                                                                        // new cardinal conflicts might emerge
        node.right_f_will_increase = ICBSNode::WillCostIncrease::YES;
        // Find the conflict with the highest cost increase for the at-goal node (linearly)
        std::shared_ptr<Conflict> ret;
        int best_cost_increase = 0;
        for (const auto& conf : node.cardinalGoalConf) {
            const auto& [agent1, agent2, loc1, loc2, timestep] = *conf;
            int cost_increase = timestep + 1 - ((*node.all_paths)[agent2]->size() - 1);
            if (cost_increase > best_cost_increase) {
                best_cost_increase = cost_increase;
                ret = conf;
            }
        }
        return ret;
    }

	if (!node.cardinalConf.empty() || !node.cardinalGoalConf.empty())
	{
		conflictType = conflict_type::CARDINAL;  // Even if it's cardinal-goal, we'll report it as cardinal
		node.left_cost_will_increase = ICBSNode::WillCostIncrease::YES;
		node.right_cost_will_increase = ICBSNode::WillCostIncrease::YES;
		node.left_f_will_increase = ICBSNode::WillCostIncrease::MAYBE;  // Hard to say no without generating the child,
		                                                                // new cardinal conflicts might emerge
		node.right_f_will_increase = ICBSNode::WillCostIncrease::MAYBE;  // Hard to say no without generating the child,
                                                                         // new cardinal conflicts might emerge
        if (preferGoalConflicts)  // We would've entered their block above if there were any
		    return getHighestPriorityConflict(node, node.cardinalConf);
        else {
            auto temp = node.cardinalGoalConf;  // A full copy
            for (const auto& conf : node.cardinalConf)
                temp.push_back(conf);
            return getHighestPriorityConflict(node, temp);
        }
	}

	if (!node.semiCardinalGoalConf.empty() && preferGoalConflicts)
    {
        conflictType = conflict_type::SEMICARDINAL_GOAL;
        node.left_cost_will_increase = ICBSNode::WillCostIncrease::NO;  // Even with disjoint splitting, the cost of the
                                                                        // left agent will not increase. I thought hard
                                                                        // about it.
        node.right_cost_will_increase = ICBSNode::WillCostIncrease::YES;  // We made sure the cardinal agent is second
        node.left_f_will_increase = ICBSNode::WillCostIncrease::MAYBE;  // Hard to say no without generating the child,
                                                                        // new cardinal conflicts might emerge
        node.right_f_will_increase = ICBSNode::WillCostIncrease::MAYBE;  // Hard to say no without generating the child,
                                                                         // new cardinal conflicts might emerge
        // Find the conflict with the highest cost increase for the at-goal node (linearly)
        std::shared_ptr<Conflict> ret;
        int best_cost_increase = 0;
        for (const auto& conf : node.semiCardinalGoalConf) {
            const auto& [agent1, agent2, loc1, loc2, timestep] = *conf;
            int cost_increase = timestep + 1 - ((*node.all_paths)[agent2]->size() - 1);
            if (cost_increase > best_cost_increase) {
                best_cost_increase = cost_increase;
                ret = conf;
            }
        }
        return ret;
    }

	if (!node.semiCardinalConf.empty() || !node.semiCardinalGoalConf.empty())
	{
		conflictType = conflict_type::SEMICARDINAL;
        node.left_cost_will_increase = ICBSNode::WillCostIncrease::NO;
        node.right_cost_will_increase = ICBSNode::WillCostIncrease::YES;  // We made sure the cardinal agent is second
        node.left_f_will_increase = ICBSNode::WillCostIncrease::MAYBE;  // Hard to say no without generating the child,
                                                                        // new cardinal conflicts might emerge
        node.right_f_will_increase = ICBSNode::WillCostIncrease::MAYBE;  // Hard to say no without generating the child,
                                                                         // new cardinal conflicts might emerge
        if (preferGoalConflicts)
		    return getHighestPriorityConflict(node, node.semiCardinalConf);  // We would've entered their block above if there were any
        else {
            auto temp = node.semiCardinalGoalConf;
            for (const auto& conf : node.semiCardinalConf)
                temp.push_back(conf);
//            shuffle(temp.begin(), temp.end(), std::default_random_engine(seed));  // More fair, but takes time
            return getHighestPriorityConflict(node, temp);
        }
	}
	else if (!node.nonCardinalConf.empty()) {
        conflictType = conflict_type::NONCARDINAL;
        node.left_cost_will_increase = ICBSNode::WillCostIncrease::NO;
        node.right_cost_will_increase = ICBSNode::WillCostIncrease::NO;
        node.left_f_will_increase = ICBSNode::WillCostIncrease::MAYBE;  // Hard to say no without generating the child,
                                                                        // new cardinal conflicts might emerge
        node.right_f_will_increase = ICBSNode::WillCostIncrease::MAYBE;  // Hard to say no without generating the child,
                                                                         // new cardinal conflicts might emerge
        return getHighestPriorityConflict(node, node.nonCardinalConf);
    }
	else if (!node.unknownConf.empty())  // If classifyConflicts timed out before any conflict was classified, this can happen
    {
        conflictType = conflict_type::NONCARDINAL;
        node.left_cost_will_increase = ICBSNode::WillCostIncrease::NO;
        node.right_cost_will_increase = ICBSNode::WillCostIncrease::NO;
        node.left_f_will_increase = ICBSNode::WillCostIncrease::MAYBE;  // Hard to say no without generating the child,
                                                                        // new cardinal conflicts might emerge
        node.right_f_will_increase = ICBSNode::WillCostIncrease::MAYBE;  // Hard to say no without generating the child,
                                                                         // new cardinal conflicts might emerge
        return getHighestPriorityConflict(node, node.unknownConf);
	} else {
        node.left_cost_will_increase = ICBSNode::WillCostIncrease::NO;  // No conflicts - no children
        node.right_cost_will_increase = ICBSNode::WillCostIncrease::NO;
        node.left_f_will_increase = ICBSNode::WillCostIncrease::NO;
        node.right_f_will_increase = ICBSNode::WillCostIncrease::NO;
        return nullptr;
    }
}

// Secondary priority within a primary priority group
std::shared_ptr<Conflict> ICBSSearch::getHighestPriorityConflict(ICBSNode &node, const vector<std::shared_ptr<Conflict>> &confs)
{
	std::shared_ptr<Conflict> choice = confs.front();

	if (split == split_strategy::SINGLETONS)
	{
		vector<int> metric(num_of_agents, 0);
		vector<double> averageMDDWidth(num_of_agents, 0);
		for (int i = 0; i < num_of_agents; i++)
		{
			Path & path_i = *(*node.all_paths)[i];
			for (int j = 0; j < path_i.size(); j++)
				averageMDDWidth[i] += path_i[j].numMDDNodes;
			averageMDDWidth[i] /= path_i.size();
		}

		for (const auto& conf : confs)
		{
			const auto& [agent1, agent2, loc1, loc2, timestep] = *conf;

			Path & a1_path = *(*node.all_paths)[agent1];
			Path & a2_path = *(*node.all_paths)[agent2];

			int metric1 = 0;
			int maxSingles = 0;
			double minWidth = 0;
			if (timestep < (int)a1_path.size())
				for (int j = 0; j < timestep; j++)
					if (a2_path[j].builtMDD && a1_path[j].single)
						metric1++;
			int metric2 = 0;
			if (timestep < (int)a2_path.size())
				for (int j = 0; j < timestep; j++)
					if (a2_path[j].builtMDD && a2_path[j].single)
						metric2++;
			if (max(metric1, metric2) > metric[0])
			{
				choice = conf;
				maxSingles = max(metric1, metric2);
				minWidth = min(averageMDDWidth[agent1], averageMDDWidth[agent2]);
			}
			else if (max(metric1, metric2) == maxSingles && min(averageMDDWidth[agent1], averageMDDWidth[agent2]) < minWidth)
			{
				choice = conf;
				minWidth = min(averageMDDWidth[agent1], averageMDDWidth[agent2]);
			}
		}
	}
	else if (split == split_strategy::WIDTH ||  // Choose lowest multiplication of MDD widths at conflict timestep,
                                                // then the conflict which has the agent with the most MDD singletons
                                                // leading up to the conflict timestep
            split == split_strategy::MVC_BASED)  // In this case the choice may be changed (currently for cardinal conflicts only) in computeHeuristic
	{
        int choice_left_width, choice_right_width;
        int choice_left_singletons = 0, choice_right_singletons = 0;
        const auto& [choice_agent1, choice_agent2, choice_loc1, choice_loc2, choice_timestep] = *choice;
        Path & choice_a1_path = *(*node.all_paths)[choice_agent1];
        Path & choice_a2_path = *(*node.all_paths)[choice_agent2];
        if (choice_a1_path.size() <= choice_timestep)
            choice_left_width = 1;
        else
            choice_left_width = choice_a1_path[choice_timestep].numMDDNodes;
        if (choice_a2_path.size() <= choice_timestep)
            choice_right_width = 1;
        else
            choice_right_width = choice_a2_path[choice_timestep].numMDDNodes;
        for (int j = 0; j <= choice_timestep && j < choice_a1_path.size() ; j++)
            if (choice_a1_path[j].builtMDD && choice_a1_path[j].single)
                choice_left_singletons++;
        for (int j = 0; j <= choice_timestep && j < choice_a2_path.size() ; j++)
            if (choice_a2_path[j].builtMDD && choice_a2_path[j].single)
                choice_right_singletons++;
        int choice_steps_from_goal = std::abs(((int)choice_a1_path.size()) - 1 - choice_timestep) +
                                     std::abs(((int)choice_a2_path.size()) - 1 - choice_timestep);

		for (const auto& conf : confs)
		{
			const auto& [agent1, agent2, loc1, loc2, timestep] = *conf;
            int left_width, right_width;

			Path & a1_path = *(*node.all_paths)[agent1];
			Path & a2_path = *(*node.all_paths)[agent2];

			if (a1_path.size() <= timestep)
                left_width = 1;
			else
                left_width = a1_path[timestep].numMDDNodes;
			if (a2_path.size() <= timestep)
                right_width = 1;
			else
                right_width = a2_path[timestep].numMDDNodes;
			if (left_width * right_width <= choice_left_width * choice_right_width) {  // Conflict looks good generally
                int left_singletons = 0, right_singletons = 0;
                for (int j = 0; j <= timestep && j < a1_path.size() ; j++)
                    if (a1_path[j].builtMDD && a1_path[j].single)
                        left_singletons++;
                for (int j = 0; j <= timestep && j < a2_path.size(); j++)
                    if (a2_path[j].builtMDD && a2_path[j].single)
                        right_singletons++;
                int steps_from_goal = std::abs(((int)a1_path.size()) - 1 - timestep) +
                                      std::abs(((int)a2_path.size()) - 1 - timestep);

                if ((left_width * right_width < choice_left_width * choice_right_width) ||
                    ((left_width * right_width == choice_left_width * choice_right_width) &&
#if !defined(LATEST_CONFLICT_WITHIN_CLASS)
                     (max(left_singletons, right_singletons) > max(choice_left_singletons, choice_right_singletons)))
#else
                     (max(left_singletons, right_singletons) > max(choice_left_singletons, choice_right_singletons))) ||
                    ((left_width * right_width == choice_left_width * choice_right_width) &&
                     (max(left_singletons, right_singletons) == max(choice_left_singletons, choice_right_singletons)) &&
                     (steps_from_goal < choice_steps_from_goal)
                    )
#endif
                   ) {
                    choice = conf;
                    choice_left_width = left_width;
                    choice_right_width = right_width;
                    choice_left_singletons = left_singletons;
                    choice_right_singletons = right_singletons;
                    choice_steps_from_goal = steps_from_goal;
                }
            }
		}
	}
    else  // No special strategy
    {
#if !defined(LATEST_CONFLICT_WITHIN_CLASS)
        // uniformly at random
		int id = rand() % confs.size();
		return confs[id];
#else
         // Closer to the goal - better for LPA*
		 // Can't just choose the conflict with the latest timestep. A later conflict in a very very long path
		 //	   might not be better than an earlier conflict in a short path.
		int min_steps_from_goal = std::numeric_limits<int>::max();
		for (const auto& conf : confs)
		{
			const auto& [agent1, agent2, loc1, loc2, timestep] = *conf;
			Path * a1_path = (*node.all_paths)[agent1];
			Path * a2_path = (*node.all_paths)[agent2];
			int steps_from_goal = std::abs(((int) a1_path->size()) - 1 - timestep) +
			                      std::abs(((int) a2_path->size()) - 1 - timestep);
			if (steps_from_goal < min_steps_from_goal) {
				choice = conf;
				min_steps_from_goal = steps_from_goal;
			}
		}
#endif
    }
	return choice;
}

// build MDDs if needed
// For every step of the path of the given agent, save whether the MDD at that step has a single node
// Returns whether building was successful
// The cat is only used if LPA* is used to build the MDD
bool ICBSSearch::buildMDD(ICBSNode &curr, int ag, int timestep, int lookahead /* = 0*/, ConflictAvoidanceTable *cat /* = nullptr*/)
{
	Path & path = *(*curr.all_paths)[ag];
	if (path[timestep].builtMDD)
		return true;

	spdlog::info("Building MDD of length {} for agent {}", path.size(), ag);

	// Find the last node on this branch that computed a new path for this agent
	ICBSNode* node = &curr; // Back to where we get the path
	bool found = false;
	while (node->parent != nullptr)
	{
		for (const auto& agent_id_and_new_path : node->new_paths)
		{
		    const auto& [agent_id, new_path] = agent_id_and_new_path;
			if (agent_id == ag)
			{
				found = true;
				break;
			}
		}
		if (found)
			break;
		else
			node = node->parent;
	}

	std::clock_t mdd_building_start = std::clock();
	auto wall_mddStart = std::chrono::system_clock::now();

#ifndef LPA
	// Build a constraint table with entries for each timestep up to the makespan at the last node that added a
	// constraint for the agent. There can't be constraints that occur later than that because each agent must plan a
	// path that at least lets every constraint have the opportunity to affect it.
	std::vector < std::unordered_map<int, ConstraintState > > cons_table(node->makespan + 1);
	pair<int, int> start_location_and_time = make_pair(search_engines[ag]->start_location, 0);
	pair<int, int> goal_location_and_time = make_pair(search_engines[ag]->goal_location, (int)path.size() - 1);
	buildConstraintTable(node, ag, timestep, cons_table, start_location_and_time, goal_location_and_time);

	MDD mdd;
	bool success = mdd.buildMDD(cons_table, start_location_and_time, goal_location_and_time, lookahead, *search_engines[ag], start + time_limit * CLOCKS_PER_SEC);
	if (!success) {
        highLevelMddBuildingTime += std::clock() - mdd_building_start;
        wall_mddTime += std::chrono::system_clock::now() - wall_mddStart;
        return false;
    }

	for (int i = 0; i < mdd.levels.size(); i++)
	{
	    path[i + start_location_and_time.second].single = mdd.levels[i].size() == 1;
	    path[i + start_location_and_time.second].numMDDNodes = (int)mdd.levels[i].size();
	    path[i + start_location_and_time.second].builtMDD = true;
	}
#else
    // <experiment>
//    // Build a constraint table with entries for each timestep up to the makespan at the last node that added a
//    // constraint for the agent. There can't be constraints that occur later than that because each agent must plan a
//    // path that at least lets every constraint have the opportunity to affect it.
//    cerr << "experiment" << endl;
//    std::vector < std::unordered_map<int, ConstraintState > > cons_table(node->makespan + 1);
//    pair<int, int> start = make_pair(search_engines[ag]->start_location, 0);
//    pair<int, int> goal = make_pair(search_engines[ag]->goal_location, (int)path.size() - 1);
//    int last_goal_constraint_timestep = buildConstraintTable(node, ag, timestep, cons_table, start, goal);
//
//    if (last_goal_constraint_timestep + 1 != curr.lpas[ag]->min_goal_timestep) {
//        cerr << "Last goal constraint timestep of node #" << node->time_generated << " is " <<
//            last_goal_constraint_timestep << " but LPA*'s min goal timestep is " << curr.lpas[ag]->min_goal_timestep <<
//            " instead of " << last_goal_constraint_timestep + 1 << endl;
//        abort();
//    }
//    for (int i = 1 ; i < cons_table.size() ; ++i) {
//        for (const auto& pair: cons_table[i]) {
//            if (pair.second.vertex && !curr.lpas[ag]->dcm.isDynCons(pair.first, pair.first, i)) {
//                cerr << "Constraint for agent " << ag << " missing from dcm in timestep " << i << ". " << pair.first << " is supposed to be vertex constrained." << endl;
//                abort();
//            }
//            for (int direction = 0; direction < MapLoader::WAIT_MOVE ; direction++) {
//                if (!pair.second.edge[direction])
//                    continue;
//                auto prev_loc_id = pair.first - moves_offset[direction];
//                if (!curr.lpas[ag]->dcm.isDynCons(prev_loc_id, pair.first, i)) {
//                    cerr << "Constraint for agent " << ag << " missing from dcm in timestep " << i << ". " <<
//                         "(" << pair.first << "," << prev_loc_id << ") is supposed to be edge constrained." << endl;
//                    abort();
//                }
//            }
//        }
//
//        if (i < curr.lpas[ag]->dcm.dyn_constraints_.size()) {
//            for (const auto &pair : curr.lpas[ag]->dcm.dyn_constraints_[i]) {
//                int delta = pair.second - pair.first;
//                for (int direction = 0; direction < 5; ++direction) {
//                    if (delta != this->moves_offset[direction])
//                        continue;
//                    if (!search_engines[ag]->isConstrained(direction, pair.second, i, cons_table)) {
//                        cerr << "Superfluous constraint for agent " << ag << " in dcm timestep " << i << ". " <<
//                             "((" << pair.first / 8 << "," << pair.first % 8 << "),(" << pair.second / 8 << ","
//                             << pair.second % 8 << ")) is not supposed to be constrained." << endl;
//                        abort();
//                    }
//                    else
//                        break;
//                }
//            }
//        }
//    }
//    MDD mdd;
//    mdd.buildMDD(cons_table, start, goal, lookahead, *search_engines[ag]);
    // </experiment>

    bool success = curr.lpas[ag]->buildMdd(cat, start + time_limit * CLOCKS_PER_SEC);
    if (!success) {
        highLevelMddBuildingTime += std::clock() - mdd_building_start;
        wall_mddTime += std::chrono::system_clock::now() - wall_mddStart;
        return false;
    }

    // <experiment>
//	if (curr.lpas[ag]->mddLevelSizes.size() != mdd.levels.size()) {
//	    cerr << "iMDD wrong size " << curr.lpas[ag]->mddLevelSizes.size() << "!=" << mdd.levels.size() << endl;
//	    abort();
//	}
//    for (int i = path.size() - 2; i >= 1; i--) // The first and last step are already marked as singletons
//    {
//        if (curr.lpas[ag]->mddLevelSizes[i] != mdd.levels[i].size()) {
//            cerr << "Agent " << ag << " " << (int)path.size() << "-MDD level " << i << " size mismatch! " <<
//                 curr.lpas[ag]->mddLevelSizes[i] << "!=" << mdd.levels[i].size() << endl;
//            abort();
//        }
//    }
    // </experiment>

    for (int i = path.size() - 2; i >= 1; i--) // The first and last step are already marked as singletons
	{
	    path[i].single = curr.lpas[ag]->mddLevelSizes[i] == 1;
	    path[i].numMDDNodes = (int)curr.lpas[ag]->mddLevelSizes[i];
	    path[i].builtMDD = true;
	}
#endif

    highLevelMddBuildingTime += std::clock() - mdd_building_start;
    wall_mddTime += std::chrono::system_clock::now() - wall_mddStart;

    return true;
}

int ICBSSearch::countMddSingletons(vector<Path *> &the_paths, int agent_id, int conflict_timestep)
{
	int num_singletons = 0;
	if (conflict_timestep < (int)the_paths[agent_id]->size())
		for (int j = 0; j < conflict_timestep; j++)
		{
			if (the_paths[agent_id]->at(j).builtMDD && the_paths[agent_id]->at(j).single)
				num_singletons++;
		}
	return num_singletons;
}

int ICBSSearch::getTotalMddWidth(vector<Path *> &the_paths, int agent_id)
{
	int width = 0;
	for (int j = 0; j < the_paths[agent_id]->size(); j++)
	{
		width += the_paths[agent_id]->at(j).numMDDNodes;
	}
	return width;
}

void ICBSSearch::addPositiveConstraintsOnNarrowLevelsLeadingToPositiveConstraint(Path &parent_path,
    int new_positive_constraint_timestep, ICBSNode* child, const ConflictAvoidanceTable* catp)
{
	if (posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem)
	// The MDD levels up to the positive constraint are a superset of the MDD levels of the MDD for reaching
	// the location of the positive constraint at the time of the positive constraint, so any 1-width level
	// among the levels up to that of the positive constraint is also a 1-width level in the MDD for the agent
	// to reach the positive constraint (we know the positive constraint is reachable so it can't be a
	// 0-width level in the smaller MDD). Every 1-width level's node can also be added as a positive constraint
	// - we must pass through it to reach the positive constraint we added.
	{
		for (int i = new_positive_constraint_timestep; i > 0; i--)
		{
			if (!parent_path[i].builtMDD)  // No MDD for this level. When does this happen?
				break;
			else if (!parent_path[i].single)  // Not a 1-width MDD level.
				continue;
			else if (parent_path[i - 1].builtMDD && parent_path[i - 1].single) {  // Safe because i > 0
				// This level is narrow and the previous one too - add a positive edge constraint between their
				// nodes. It's preferable over two positive constraints for some reason.
				LL_num_generated += child->add_constraint(make_tuple(parent_path[i - 1].location, parent_path[i].location, i, true), catp);
			}
			else if (i < new_positive_constraint_timestep && !parent_path[i + 1].single) {
				// This level is narrow, and the *next* one isn't (so we didn't already add a positive edge
				// constraint between them) - add a positive vertex constraint for this level's node
				LL_num_generated += child->add_constraint(make_tuple(parent_path[i].location, -1, i, true), catp);
			}
		}
	}
}

void ICBSSearch::removePositiveConstraintsOnNarrowLevelsLeadingToPositiveConstraint(Path &parent_path,
     int new_positive_constraint_timestep, ICBSNode* child, const ConflictAvoidanceTable* catp)
{
    if (posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem)
        // The MDD levels up to the positive constraint are a superset of the MDD levels of the MDD for reaching
        // the location of the positive constraint at the time of the positive constraint, so any 1-width level
        // among the levels up to that of the positive constraint is also a 1-width level in the MDD for the agent
        // to reach the positive constraint (we know the positive constraint is reachable so it can't be a
        // 0-width level in the smaller MDD). Every 1-width level's node can also be added as a positive constraint
        // - we must pass through it to reach the positive constraint we added.
    {
        for (int i = new_positive_constraint_timestep; i > 0; i--)
        {
            if (!parent_path[i].builtMDD)  // No MDD for this level. When does this happen?
                break;
            else if (!parent_path[i].single)  // Not a 1-width MDD level.
                continue;
            else if (parent_path[i - 1].builtMDD && parent_path[i - 1].single) {  // Safe because i > 0
                // This level is narrow and the previous one too - add a positive edge constraint between their
                // nodes. It's preferable over two positive constraints for some reason.
                LL_num_generated += child->pop_constraint(catp);
            }
            else if (i < new_positive_constraint_timestep && !parent_path[i + 1].single) {
                // This level is narrow, and the *next* one isn't (so we didn't already add a positive edge
                // constraint between them) - add a positive vertex constraint for this level's node
                LL_num_generated += child->pop_constraint(catp);
            }
        }
    }
}


//////////////////// GENERATE A NODE ///////////////////////////
// Non lpa-jump version
void ICBSSearch::disjoint_branch_on_agent(ICBSNode* parent, ICBSNode* child1, ICBSNode* child2, int agent) {
    auto [agent1_id, agent2_id, location1, location2, timestep] = *parent->conflict;  // Not auto& because I might change the locations

    ConflictAvoidanceTable* catp = nullptr;
#ifndef LPA
#else
    // build a conflict-avoidance table for the agent we'll constrain
    ConflictAvoidanceTable cat(this->moves_offset, this->map_size);
    buildConflictAvoidanceTable(*parent, agent, cat);
    catp = &cat;
#endif

    child1->agent_id = agent;
    child2->agent_id = agent;
    if (location2 >= 0 && agent == agent2_id) // Adding an edge constraint for the second agent - the constraint will
        // be on traversing the edge in the opposite direction
    {
        int temp = location1;
        location1 = location2;
        location2 = temp;
    }

    child1->all_paths = parent->all_paths;  // Temporarily
    child2->all_paths = parent->all_paths;  // Temporarily
    LL_num_generated += child1->add_constraint(make_tuple(location1, location2, timestep, true), catp, false, posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem);
    LL_num_generated += child2->add_constraint(make_tuple(location1, location2, timestep, false), catp, false, posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem);
    child1->all_paths = nullptr;
    child2->all_paths = nullptr;

    spdlog::info("Disjoint branching on agent {} (positive constraint first)", agent);
}


// add constraints to child nodes
void ICBSSearch::branch(ICBSNode* parent, ICBSNode* child1, ICBSNode* child2)
{
	const auto& [agent1_id, agent2_id, location1, location2, timestep] = *parent->conflict;

	if (split == split_strategy::RANDOM)  // A disjoint split that chooses the agent to work on randomly
	{
		int id;
		if (rand() % 2 == 0) {
			id = agent1_id;
		}
		else {
			id = agent2_id;
		}

		disjoint_branch_on_agent(parent, child1, child2, id);
	}
	else if (split == split_strategy::SINGLETONS)  // A disjoint split that chooses the agent to work on to be the one
												   // with the smaller number of 1-width levels in each agent's MDD,
												   // and if they're equal, the one with the smaller total width of all
												   // levels in its MDD, divided by the size of the MDD
	{
		int num_singletons_1 = countMddSingletons(*parent->all_paths, agent1_id, timestep);
		int num_singletons_2 = countMddSingletons(*parent->all_paths, agent2_id, timestep);
		int id;
		if (num_singletons_1 > num_singletons_2)
			id = agent1_id;
		else if (num_singletons_1 < num_singletons_2)
			id = agent2_id;
		else {
			double normalized_width1 = float(getTotalMddWidth(*parent->all_paths, agent1_id)) / (*parent->all_paths)[agent1_id]->size();
			double normalized_width2 = float(getTotalMddWidth(*parent->all_paths, agent2_id)) / (*parent->all_paths)[agent2_id]->size();
			if (normalized_width1 < normalized_width2)
				id = agent1_id;
			else
				id = agent2_id;
		}

		disjoint_branch_on_agent(parent, child1, child2, id);
	}
	else if (split == split_strategy::WIDTH)
	{
		int id;
		if (timestep >= (*parent->all_paths)[agent1_id]->size())
			id = agent2_id;
		else if (timestep >= (*parent->all_paths)[agent2_id]->size())
			id = agent1_id;
		else if ((*parent->all_paths)[agent1_id]->at(timestep).numMDDNodes < (*parent->all_paths)[agent2_id]->at(timestep).numMDDNodes)
			id = agent1_id;
		else
			id = agent2_id;

		disjoint_branch_on_agent(parent, child1, child2, id);
	}
	else if (split == split_strategy::MVC_BASED) {  // A disjoint split that chooses the agent that's more likely
	                                                // to increase the f-value of both child nodes
		disjoint_branch_on_agent(parent, child1, child2, parent->branch_on_first_agent ? agent1_id : agent2_id);
	}
	else  // Do a non-disjoint split
	{
		child1->agent_id = agent1_id;
		child2->agent_id = agent2_id;

		ConflictAvoidanceTable* catp = nullptr;
#ifndef LPA
#else
		// build a conflict-avoidance table for the agent we'll constrain
		ConflictAvoidanceTable cat(this->moves_offset, this->map_size);
		buildConflictAvoidanceTable(*parent, agent1_id, cat);
		catp = &cat;
#endif

		child1->all_paths = parent->all_paths;  // Temporarily
		LL_num_generated += child1->add_constraint(make_tuple(location1, location2, timestep, false), catp, false, posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem);
		child1->all_paths = nullptr;

#ifndef LPA
#else
		removePathFromConflictAvoidanceTable(*(*parent->all_paths)[agent1_id], *catp, agent1_id);
		addPathToConflictAvoidanceTable(*(*parent->all_paths)[agent2_id], *catp, agent2_id);
#endif

		child2->all_paths = parent->all_paths;  // Temporarily
		if (location2 >= 0) // Adding an edge constraint for the second agent - the constraint will
							// be on traversing the edge in the opposite direction
			LL_num_generated += child2->add_constraint(make_tuple(location2, location1, timestep, false), catp, false, posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem);
		else
			LL_num_generated += child2->add_constraint(make_tuple(location1, location2, timestep, false), catp, false, posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem);
		child1->all_paths = nullptr;
	}
}

// Returns whether a bypass was found for the conflict and the parent should be re-expanded
bool ICBSSearch::disjoint_branch_and_generate_with_up_and_down(ICBSNode* parent, ICBSNode* child1, ICBSNode* child2,
		ConflictAvoidanceTable *cat, int agent_id)
{
	child1->agent_id = agent_id;
	child2->agent_id = agent_id;

	auto [agent1_id, agent2_id, location1, location2, timestep] = *parent->conflict;  // NOT auto& since I might change the locations

	// Add the positive constraint to child1:
	removePathFromConflictAvoidanceTable(*(*parent->all_paths)[agent_id], *cat, agent_id);

	if (location2 >= 0 && agent_id == agent2_id) // Adding an edge constraint for the second agent - the constraint will
												 // be on traversing the edge in the opposite direction
	{
		int temp = location1;
		location1 = location2;
		location2 = temp;
	}

    vector<Path*> temp(*parent->all_paths);
    child1->all_paths = parent->all_paths;  // Temporarily

	LL_num_generated += child1->add_constraint(make_tuple(location1, location2, timestep, true), cat, true, posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem);
	auto [child1_or_nullptr, continue_to_next_child1] = generateChild(child1, cat); // plan paths for child1, storing them in its new_paths
    if (child1_or_nullptr != nullptr) {
        spdlog::info("Generated left child #{} with f=  {}+{} and {} conflicts ",
                     child1->time_generated, child1->g_val, child1->h_val, child1->num_of_conflicts);
    } else if (continue_to_next_child1) {
        spdlog::info("No feasible solution for left child!");
    }
    if (!continue_to_next_child1)
        spdlog::info("Left child found a bypass - adopting its new paths without splitting");
    if (child1_or_nullptr != nullptr && continue_to_next_child1)  // Child might have been deleted in generateChild. FIXME: Bad design.
	    child1->all_paths = nullptr;
#ifndef LPA
#else
	LL_num_generated += child1->pop_lpa_constraint(cat);
	// TODO: AND ALL THOSE THAT WERE PROPAGATED!
#endif
    if (!continue_to_next_child1) {
        addPathToConflictAvoidanceTable(*(*parent->all_paths)[agent_id], *cat, agent_id);
        return true;
    }

    *parent->all_paths = temp;
	LL_num_generated += child2->add_constraint(make_tuple(location1, location2, timestep, false), cat, true, posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem);
	child2->all_paths = parent->all_paths;  // Temporarily
    auto [child2_or_nullptr, continue_to_next_child2] = generateChild(child2, cat); // plan paths for child2
    if (child2_or_nullptr != nullptr)
    {
        spdlog::info("Generated right child #{} with f=  {}+{} and {} conflicts ",
                     child2->time_generated, child2->g_val, child2->h_val, child2->num_of_conflicts);
    }
    else if (continue_to_next_child2)
    {
        spdlog::info("No feasible solution for right child!");
    }
    if (!continue_to_next_child2)
        spdlog::info("Right child found a bypass - adopting its new paths without splitting");
    if (child2_or_nullptr != nullptr && continue_to_next_child2)  // Child might have been deleted in generateChild. FIXME: Bad design.
	    child2->all_paths = nullptr;
#ifndef LPA
#else
	LL_num_generated += child2->pop_lpa_constraint(cat);
    // TODO: AND ALL THOSE THAT WERE PROPAGATED!
#endif

	addPathToConflictAvoidanceTable(*(*parent->all_paths)[agent_id], *cat, agent_id);

	if (!continue_to_next_child2)
	    return true;

    *parent->all_paths = temp;
    if (child1_or_nullptr != nullptr)
	    push_child_into_lists(child1);
    if (child2_or_nullptr != nullptr)
        push_child_into_lists(child2);

	return false;
}

// Add constraints to child nodes and plan paths using the CBS-up-and-down technique
// Returns whether a bypass was found and the parent needs to be re-expanded
bool ICBSSearch::branch_and_generate_with_up_and_down(ICBSNode* parent, ICBSNode* child1, ICBSNode* child2, ConflictAvoidanceTable *cat)
{
	const auto& [agent1_id, agent2_id, location1, location2, timestep] = *parent->conflict;

	if (split == split_strategy::RANDOM)  // A disjoint split that chooses the agent to work on randomly
	{
		int id;
		if (rand() % 2 == 0) {
			id = agent1_id;
		}
		else {
			id = agent2_id;
		}

		return disjoint_branch_and_generate_with_up_and_down(parent, child1, child2, cat, id);
	}
	else if (split == split_strategy::SINGLETONS)  // A disjoint split that chooses the agent to work on to be the one
		// with the smaller number of 1-width levels in each agent's MDD,
		// and if they're equal, the one with the smaller total width of all
		// levels in its MDD, divided by the size of the MDD
	{
		int num_singletons_1 = countMddSingletons(*parent->all_paths, agent1_id, timestep);
		int num_singletons_2 = countMddSingletons(*parent->all_paths, agent2_id, timestep);
		int id;
		if (num_singletons_1 > num_singletons_2)
			id = agent1_id;
		else if (num_singletons_1 < num_singletons_2)
			id = agent2_id;
		else {
			double normalized_width1 = float(getTotalMddWidth(*parent->all_paths, agent1_id)) / (*parent->all_paths)[agent1_id]->size();
			double normalized_width2 = float(getTotalMddWidth(*parent->all_paths, agent2_id)) / (*parent->all_paths)[agent2_id]->size();
			if (normalized_width1 < normalized_width2)
				id = agent1_id;
			else
				id = agent2_id;
		}

		return disjoint_branch_and_generate_with_up_and_down(parent, child1, child2, cat, id);
	}
	else if (split == split_strategy::WIDTH)
	{
		int id;
		if (timestep >= (*parent->all_paths)[agent1_id]->size())
			id = agent2_id;
		else if (timestep >= (*parent->all_paths)[agent2_id]->size())
			id = agent1_id;
		else if ((*parent->all_paths)[agent1_id]->at(timestep).numMDDNodes < (*parent->all_paths)[agent2_id]->at(timestep).numMDDNodes)
			id = agent1_id;
		else
			id = agent2_id;

		return disjoint_branch_and_generate_with_up_and_down(parent, child1, child2, cat, id);
	}
	else  // Do a classic non-disjoint split
	{
		child1->agent_id = agent1_id;
		child2->agent_id = agent2_id;

		removePathFromConflictAvoidanceTable(*(*parent->all_paths)[agent1_id], *cat, agent1_id);  // When not using LPA* this is just for the pathfinding

		LL_num_generated += child1->add_constraint(make_tuple(location1, location2, timestep, false), cat, true, posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem);

		vector<Path*> temp(*parent->all_paths);
		child1->all_paths = parent->all_paths;
        auto [child1_or_nullptr, continue_to_next_child1] = generateChild(child1, cat); // plan paths for child1, storing them in its new_paths
        if (child1_or_nullptr != nullptr) {
            spdlog::info("Generated left child #{} with f= {}+{} and {} conflicts ",
                         child1->time_generated, child1->g_val, child1->h_val, child1->num_of_conflicts);
        } else if (continue_to_next_child1) {
            spdlog::info("No feasible solution for left child!");
        }
        if (!continue_to_next_child1)
            spdlog::info("Left child found a bypass - adopting its new paths without splitting");
		if (child1_or_nullptr != nullptr && continue_to_next_child1) {  // Otherwise child1 was deleted and this is unsafe
            child1->all_paths = nullptr;
        }

		if (continue_to_next_child1)  // Otherwise the parent's all_paths was already updated during the bypass, and temp is obsolete
            *parent->all_paths = temp;
        addPathToConflictAvoidanceTable(*(*parent->all_paths)[agent1_id], *cat, agent1_id);

#ifndef LPA
#else
		LL_num_generated += parent->pop_lpa_constraint(agent1_id, make_tuple(location1, location2, timestep, false), cat);  // The child might have been deleted (if no path was found)
#endif

		if (!continue_to_next_child1)
		    return true;

		removePathFromConflictAvoidanceTable(*(*parent->all_paths)[agent2_id], *cat, agent2_id);  // TODO: Maintain a set of indices of agents whose paths are currently in the CAT - for debugging
		if (location2 >= 0) // Adding an edge constraint for the second agent - the constraint will
			// be on traversing the edge in the opposite direction
			LL_num_generated += child2->add_constraint(make_tuple(location2, location1, timestep, false), cat, true, posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem);
		else
			LL_num_generated += child2->add_constraint(make_tuple(location1, location2, timestep, false), cat, true, posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem);

		child2->all_paths = parent->all_paths;
		auto [child2_or_nullptr, continue_to_next_child2] = generateChild(child2, cat); // plan paths for child2

        if (child2_or_nullptr != nullptr)
        {
            spdlog::info("Generated right child #{} with f= {}+{} and {} conflicts ",
                         child2->time_generated, child2->g_val, child2->h_val, child2->num_of_conflicts);
        }
        else if (continue_to_next_child2)
        {
            spdlog::info("No feasible solution for right child!");
        }
        if (!continue_to_next_child2)
            spdlog::info("Right child found a bypass - adopting its new paths without splitting");
		if (child2_or_nullptr != nullptr && continue_to_next_child2) {  // Otherwise child1 was deleted and this is unsafe
		    child2->all_paths = nullptr;
		}
		if (continue_to_next_child2)  // Otherwise the parent's all_paths was already updated during the bypass, and temp is obsolete
		    *parent->all_paths = temp;
		addPathToConflictAvoidanceTable(*(*parent->all_paths)[agent2_id], *cat, agent2_id);

#ifndef LPA
#else
		if (location2 >= 0)
			LL_num_generated += parent->pop_lpa_constraint(agent2_id, make_tuple(location2, location1, timestep, false), cat);  // The child might have been deleted (if no path was found)
		else
			LL_num_generated += parent->pop_lpa_constraint(agent2_id, make_tuple(location1, location2, timestep, false), cat);  // The child might have been deleted (if no path was found)
#endif

		if (!continue_to_next_child2)
			return true;

		if (child1_or_nullptr != nullptr)
			push_child_into_lists(child1);
		if (child2_or_nullptr != nullptr)
			push_child_into_lists(child2);

		return false;
	}
}

// Plan paths for a node.
// Returns a child node (or nullptr if a path couldn't be found or if a bypass was found), and whether to go on to the
// next child (false if a bypass was found).
std::tuple<ICBSNode *, bool> ICBSSearch::generateChild(ICBSNode *child, ConflictAvoidanceTable *cat)
{
	int h = 0;  // TODO: Consider computing the heuristic minus this agent
	vector<Path *>& parent_paths = *child->all_paths;  // The child temporarily points to the parent's paths
	for (const auto& con : child->negative_constraints[child->agent_id])  // TODO: When rectangle conflict resolution is added, more than one negative constraint
	                                                                      //       might be added at once. Replan once after all negative constraints are accounted for!
	{
		const auto& [loc1, loc2, timestep, positive_constraint] = con;
		int a[2];
		if (split == split_strategy::DISJOINT3)
		{
			a[0] = child->agent_id / num_of_agents - 1;
			a[1] = child->agent_id % num_of_agents;
		}
		else
		{
			a[0] = child->agent_id;
			a[1] = -1;
		}
		for (int i = 0; i < 2 && a[i] >= 0; i++)
		{
			if (loc2 < 0 && timestep >= parent_paths[a[i]]->size())  // The agent is forced out of its goal - the cost will surely increase
				// FIXME: Assumes the cost function is sum-of-costs
			{
				// Partial expansion - delay path finding:
				pair<int, vector<PathEntry>> newPath;
				newPath.first = a[i];
				newPath.second.resize(1);
				newPath.second.back().location = -timestep;  // HACK: store the timestep of the new constraint
				child->new_paths.push_back(newPath);
				int h_increase = timestep + 1 - ((int)parent_paths[a[i]]->size() - 1);
				h += h_increase;
				child->partialExpansion = true;
                spdlog::info("Partial expansion! Conflict timestep {} > {} timestep agent {} currently "
                             "reaches its goal. Not planning a path yet - we know the cost will increase! "
                             "Pushing back with h increased by {}", timestep, parent_paths[a[i]]->size() - 1,
                             a[i], h_increase);
				continue;
			}
			int lowerbound;
			if (timestep >= parent_paths[a[i]]->size() - 1 + 1) // conflict happens _after_ the agent reaches its goal
				                                                // (because constraints are on the same time as the conflict,
				                                                // or earlier due to propagation)
				lowerbound = timestep + 1;
			else if (!parent_paths[a[i]]->at(timestep).builtMDD) // unknown
				lowerbound = (int)parent_paths[a[i]]->size() - 1;
			else if (!parent_paths[a[i]]->at(timestep).single) // not cardinal
				lowerbound = (int)parent_paths[a[i]]->size() - 1;
			else if (loc1 >= 0 && loc2 < 0) // cardinal vertex
				lowerbound = (int)parent_paths[a[i]]->size() - 1 + 1;
			else if (parent_paths[a[i]]->at(timestep - 1).builtMDD && parent_paths[a[i]]->at(timestep - 1).single) // Cardinal edge
				lowerbound = (int)parent_paths[a[i]]->size() - 1 + 1;
			else // Not cardinal edge
				lowerbound = (int)parent_paths[a[i]]->size() - 1;

			if (!findPathForSingleAgent(child, cat, timestep, lowerbound, a[i], false))
			{
				delete child;
				return make_tuple(nullptr, true);
			}
		}
	}
	for (int ag = 0; ag < num_of_agents; ++ag) {
		if (ag == child->agent_id)
		{
			continue;  // Positive constraints affect all agents except the specified agent
		}

		for (const auto& con : child->positive_constraints[child->agent_id])  // TODO: Replan once after all positive constraints are applied instead of this way.
		                                                                      //       This way is OK for now since only a single positive constraint is added at a time.
		{
			const auto& [loc1, loc2, timestep, positive_constraint] = con;
			if (loc2 < 0 &&  // vertex constraint
				getAgentLocation(parent_paths, timestep, ag) == loc1)  // The agent is in violation of the positive constraint
			{
				if (timestep > parent_paths[ag]->size())  // The agent is forced out of its goal - the cost will surely increase
					// FIXME: Assumes the cost function is sum-of-costs
				{
					// Partial expansion - postpone path finding:
					pair<int, vector<PathEntry>> newPath;
					newPath.first = ag;
					newPath.second.resize(1);  // Just a placeholder
					newPath.second.back().location = -timestep;  // Hack: Store the timestep of the constraint on the placeholder path
					child->new_paths.push_back(newPath);
					int h_increase = timestep + 1 - ((int)parent_paths[ag]->size() - 1);
					h += h_increase;
					child->partialExpansion = true;
                    spdlog::info("Partial expansion! Conflict timestep {} > {} timestep agent {} currently "
                                 "reaches its goal. Not planning a path yet - we know the cost will increase! "
                                 "Pushing back with h increased by {}", timestep, parent_paths[ag]->size() - 1,
                                 ag, h_increase);
				}
				else if (!findPathForSingleAgent(child, cat, timestep,
                                                 max((int) parent_paths[ag]->size() - 1, timestep), ag, false))
				{
					delete child;
					return make_tuple(nullptr, true);
				}
			}
			else if (loc2 >= 0)  // edge constraint
			{
				if (getAgentLocation(parent_paths, timestep - 1, ag) == loc2 &&
					getAgentLocation(parent_paths, timestep, ag) == loc1) // move from "to" to "from"
				{
					if (!findPathForSingleAgent(child, cat, timestep,
                                                max((int) parent_paths[ag]->size() - 1, timestep), ag, false))
					{
						delete child;
						return make_tuple(nullptr, true);
					}
				}
				else if (getAgentLocation(parent_paths, timestep - 1, ag) == loc1) // stay in location "from"
				{
					if (!findPathForSingleAgent(child, cat, timestep - 1,
                                                max((int) parent_paths[ag]->size() - 1, timestep), ag, false))
					{
						delete child;
						return make_tuple(nullptr, true);
					}
				}
				else if (getAgentLocation(parent_paths, timestep, ag) == loc2) // stay in location "to"
				{
					if (!findPathForSingleAgent(child, cat, timestep,
                                                max((int) parent_paths[ag]->size() - 1, timestep), ag, false))
					{
						delete child;
						return make_tuple(nullptr, true);
					}
				}
			}
		}
	}

	if (!child->partialExpansion) {
		// Check for partial expansion carried over from the parent or something
		// TODO: Can happen?
		for (const auto& pair : child->new_paths) {
		    const auto& [agent_id, path] = pair;
			if (path.size() == 1 && path.back().location < 0) {
				child->partialExpansion = true;
				break;
			}
		}
	}

	uint64_t delta_g = child->g_val - child->parent->g_val;
    if (delta_g == 0)
        ++num_delta_g_0;
    else if (delta_g == 1)
        ++num_delta_g_1;
    else {
        sum_delta_g_2_or_more += delta_g;
        ++num_delta_g_2_or_more;
    }

	// compute h value
	if (h > 0) { // from partial expansion
		child->h_val = max(h, child->parent->h_val);  // The paren't H is still valid, since we haven't replanned, and might be much larger
		child->f_val = child->g_val + child->h_val;
	}
	else
	{
		child->h_val = max(child->parent->f_val - child->g_val, 0);
		child->f_val = child->g_val + child->h_val;
	}

	// Find the node's conflicts
	copyConflictsFromParent(*child);
	if (!child->partialExpansion)
	{
		findConflicts(*child);
		child->count_conflicts();
		sum_num_conflicts += child->num_of_conflicts;
		++num_num_conflicts;
	} else
		child->count_conflicts();  // Just so it has some count and not zero, even if the count will be overridden later

    if (spdlog::default_logger()->level() < spdlog::level::warn)
		arePathsConsistentWithConstraints(parent_paths, child); // check the solution

	HL_num_generated++;
	child->time_generated = HL_num_generated;

	// Check for a bypass
	if (!child->partialExpansion &&
			(child->g_val == child->parent->g_val) && (child->num_of_conflicts < child->parent->num_of_conflicts)) {
		ICBSNode* parent = child->parent;
		// Copy the child's new paths to the parent
		vector<int> agents_with_new_paths;
		for (const auto& pair : child->new_paths) {
			int agent = pair.first;
			bool found = false;
			auto it = parent->new_paths.begin();
			for ( ; it != parent->new_paths.end(); ++it)
			{
				if (it->first == agent)  // Replace the agent's old path
				{
					found = true;
					break;
				}
				if (it->first > agent)  // Passed it - insert here
					break;
			}
			if (!found)
				it = parent->new_paths.emplace(it, pair);  // Inserts before the iterator
			else {
				it->second = pair.second;  // Override the content of the iterator - the old path
			}
			(*parent->all_paths)[agent] = &it->second;  // The child's copy is going to be deleted, along with the child
			clearConflictsOfAgent(*parent, agent);
			agents_with_new_paths.push_back(agent);
		}
		findConflicts(*parent, agents_with_new_paths);
		int old_parent_conflict_count = parent->num_of_conflicts;
		parent->count_conflicts();
        sum_num_conflicts += parent->num_of_conflicts;
        ++num_num_conflicts;
		if (parent->num_of_conflicts >= old_parent_conflict_count) {
		    ostringstream msg;
		    msg << "Same number of conflicts?? parent conflicts:" << endl;
			printConflicts(msg, *parent);
            msg << "Same number of conflicts?? child conflicts:" << endl;
            printConflicts(msg, *child);
            spdlog::critical(msg.str());
            abort();
		}
		delete child;
		return make_tuple(nullptr, false);
	}

	return make_tuple(child, true);
}

void ICBSSearch::push_child_into_lists(ICBSNode *child) {
    child->open_handle = open_list.push(child);
    child->in_open = true;
    if (child->f_val <= focal_list_threshold) {
        child->focal_handle = focal_list.push(child);
        child->in_focal = true;
    }
    allNodes_table.push_back(child);
}

// plan a path for an agent in the node, also put the path in the_paths
bool ICBSSearch::findPathForSingleAgent(ICBSNode *node, ConflictAvoidanceTable *cat, int newConstraintTimestep,
                                        int earliestGoalTimestep,
                                        int ag, bool skipNewpaths)
{
	// extract all constraints on agent ag, and build constraint table
	ICBSNode* curr = node;
	pair<int,int> start_location_and_time(search_engines[ag]->start_location, 0), goal_location_and_time(search_engines[ag]->goal_location, numeric_limits<int>::max());

	// build constraint table
	XytHolder<ConstraintState> cons_table(this->map_size);
	int lastGoalConTimestep = buildConstraintTable(curr, ag, newConstraintTimestep, cons_table, start_location_and_time, goal_location_and_time);  // TODO: Skip if LPA and (start.second == 0 && goal.second == numeric_limits<int>::max() &&node->lpas[ag] != nullptr)

	ConflictAvoidanceTable local_scope_cat(this->moves_offset, this->map_size);
	if (cat == nullptr) {
		// build conflict-avoidance table
		buildConflictAvoidanceTable(*node, ag, local_scope_cat);
		cat = &local_scope_cat;
	}
	else {
//		// <experiment>
//        std::cerr << "experiment" << std::endl;
//		buildConflictAvoidanceTable(*node, ag, local_scope_cat);
//		for (auto pair : cat->at_goal) {
//			auto local_it = local_scope_cat.at_goal.find(pair.first);
//			if (local_it == local_scope_cat.at_goal.end())
//				std::abort();
//			if (local_it->second != pair.second)
//				std::abort();
//		}
//		// compare the CATs
//		for (int j = 0; j < cat->toward_goal.size() ; ++j) {
//			if (j < local_scope_cat.toward_goal.size()) {
//				for (auto pair : cat->toward_goal[j]) {
//					auto local_it = local_scope_cat.toward_goal[j].find(pair.first);
//					if (local_it == local_scope_cat.toward_goal[j].end()) {
//						if ((pair.second.vertex == 0 && pair.second.edge[0] == 0 && pair.second.edge[1] == 0 &&
//							 pair.second.edge[2] == 0 && pair.second.edge[3] == 0 && pair.second.edge[4] == 0) == false)
//							std::abort();
//					} else {
//						auto local_val = local_it->second;
//						if (pair.second.vertex != local_val.vertex)
//							std::abort();
//						for (int k = 0; k < 5; ++k) {
//							if (pair.second.edge[k] != local_val.edge[k])
//								std::abort();
//						}
//					}
//				}
//			} else {
//				for (auto pair : cat->toward_goal[j]) {
//					if ((pair.second.vertex == 0 && pair.second.edge[0] == 0 && pair.second.edge[1] == 0 &&
//						 pair.second.edge[2] == 0 && pair.second.edge[3] == 0 && pair.second.edge[4] == 0) == false)
//						std::abort();
//				}
//			}
//		}
//		// </experiment>
	}

	// Prepare the place we'll put the path
	Path* pNewPath = nullptr;
	if (!skipNewpaths) {
		// Linear lookup:
		bool found = false;
		auto it = node->new_paths.begin();
		for ( ; it != node->new_paths.end(); ++it)
			// Check if the agent has an empty path in new_paths. If it does, update its path entry.
		{
			if (it->first == ag && it->second.size() == 1 && it->second.back().location < 0)  // replace the partial expansion placeholder
			{
				found = true;
				break;
			}
			if (it->first > ag)  // Passed it - insert here
				break;
		}
		if (!found)
			it = node->new_paths.emplace(it, ag, *((*node->all_paths)[ag]));  // Inserts before the iterator
		else {
			it->second = *((*node->all_paths)[ag]);  // Copy the old path into the entry so when we search for partial path (thanks to landmarks), the rest of it will be there.
			                                         // TODO: It would be cheaper to only copy segments we aren't replanning
		}
		pNewPath = &it->second;
	} else
		pNewPath = new Path();
	Path& newPath = *pNewPath;

	// A path w.r.t cons_table (and prioritize by the conflict-avoidance table).
	bool foundSol;

	// TODO: Pass the timeout to the low level
	if (goal_location_and_time.second  >= (*node->all_paths)[ag]->size())
	{
#ifndef LPA
		clock_t ll_start = std::clock();
		auto wall_ll_start = std::chrono::system_clock::now();
		foundSol = search_engines[ag]->findShortestPath(newPath, cons_table, *cat,
														start_location_and_time, goal_location_and_time, earliestGoalTimestep, lastGoalConTimestep);
		lowLevelTime += std::clock() - ll_start;
		wall_lowLevelTime += std::chrono::system_clock::now() - wall_ll_start;
		LL_num_expanded += search_engines[ag]->num_expanded;
		LL_num_generated += search_engines[ag]->num_generated;
#else
		if (start_location_and_time.second == 0 && goal_location_and_time.second == numeric_limits<int>::max() &&
				node->lpas[ag] != nullptr
				) {
			spdlog::debug("Calling LPA* again for agent {}", ag);
			int generated_before = node->lpas[ag]->allNodes_table.size();
			clock_t ll_start = std::clock();
			auto wall_ll_start = std::chrono::system_clock::now();

			foundSol = node->lpas[ag]->findPath(*cat, earliestGoalTimestep, lastGoalConTimestep, start + time_limit * CLOCKS_PER_SEC);
			if (foundSol) {
                if (
                        (node->parent != nullptr &&  // We're running best-first traversal
                            (((node->parent->left_cost_will_increase == ICBSNode::WillCostIncrease::NO) && node->is_left_child) ||
                             ((node->parent->right_cost_will_increase == ICBSNode::WillCostIncrease::NO) && !node->is_left_child))
                        ) ||

                        (node->parent == nullptr &&  // We're running iterative-deepening or DFS traversal
                            (((node->left_cost_will_increase == ICBSNode::WillCostIncrease::NO) && (ag == get<0>(*node->conflict))) ||
                             ((node->right_cost_will_increase == ICBSNode::WillCostIncrease::NO) && (ag == get<1>(*node->conflict))))
                        )
                ) {
                    if (node->lpas[ag]->path_cost !=
                        ((*node->all_paths)[ag])->size() - 1) {  // TODO: Support non-unit edge costs
                        if (node->parent != nullptr)
                            spdlog::critical("The cost of the path of agent {} in child of #{} increased unexpectedly",
                                             ag, node->parent->time_generated);
                        else
                            spdlog::critical("The cost of the path of agent {} in child of #{} increased unexpectedly",
                                             ag, node->time_generated);
                        abort();
                    }

                    // TODO: Consider using the CAT to check if the current path has any conflicts first.
                    //       No need to do O(MDD size) work just to find an equally-perfect path
                    spdlog::info("Non-incrementally finding a conflict-avoiding path for agent {}", ag);
                    foundSol = node->lpas[ag]->findBetterPath(*cat, start + time_limit * CLOCKS_PER_SEC);
                } else if (
                        (node->parent != nullptr &&  // We're running best-first traversal
                            (((node->parent->left_cost_will_increase == ICBSNode::WillCostIncrease::YES) && node->is_left_child) ||
                             ((node->parent->right_cost_will_increase == ICBSNode::WillCostIncrease::YES) && !node->is_left_child))
                        ) ||

                        (node->parent == nullptr &&  // We're running iterative-deepening traversal
                            (((node->left_cost_will_increase == ICBSNode::WillCostIncrease::YES) && (ag == get<0>(*node->conflict))) ||
                             ((node->right_cost_will_increase == ICBSNode::WillCostIncrease::YES) && (ag == get<1>(*node->conflict))))
                        )
                ) {
                    if (node->lpas[ag]->path_cost <= ((*node->all_paths)[ag])->size() - 1) {  // TODO: Support non-unit edge costs
                        if (node->parent != nullptr)
                            spdlog::critical("The cost of the path of agent {} in child of #{} unexpectedly "
                                             "did not increase", ag, node->parent->time_generated);
                        else
                            spdlog::critical("The cost of the path of agent {} in child of #{} unexpectedly "
                                             "did not increase", ag, node->time_generated);
                        abort();
                    }
                }
            }

			lowLevelTime += std::clock() - ll_start;
			wall_lowLevelTime += std::chrono::system_clock::now() - wall_ll_start;
			if (foundSol) {
				const vector<int> *primitive_path = node->lpas[ag]->getPath();
				//const vector<int> *primitive_path = node->lpas[ag]->getPath(node->lpas[ag]->paths.size() - 1);
				newPath.resize(primitive_path->size());
				for (int j = 0; j < primitive_path->size(); ++j) {
                    newPath[j].location = (*primitive_path)[j];
                    newPath[j].builtMDD = false;
				}
				// Mark the first and last locations as "cardinal locations" regardless of the MDD:
                newPath[0].builtMDD = true;
                newPath[0].single = true;
                newPath[0].numMDDNodes = 1;
                newPath[primitive_path->size() - 1].builtMDD = true;
                newPath[primitive_path->size() - 1].single = true;
                newPath[primitive_path->size() - 1].numMDDNodes = 1;
			}
			LL_num_expanded += node->lpas[ag]->num_expanded;
			//LL_num_expanded += node->lpas[ag]->num_expandeds[node->lpas[ag]->paths.size() - 1];
			LL_num_generated += node->lpas[ag]->allNodes_table.size() - generated_before;
		}
		else
		{
            spdlog::info("Calling normal A* for agent {} instead of LPA* because LPA* can't handle a changed "
						"start, and can't recover if it was unable in the past", ag);
			if (node->lpas[ag] != NULL) {
				delete node->lpas[ag];  // We've just created this copy - it's unused anywhere else
				node->lpas[ag] = NULL;
			}
			clock_t ll_start = std::clock();
			auto wall_ll_start = std::chrono::system_clock::now();
			foundSol = search_engines[ag]->findShortestPath(*pNewPath, cons_table, *cat, start_location_and_time, goal_location_and_time,
                                                            earliestGoalTimestep, lastGoalConTimestep);
			lowLevelTime += std::clock() - ll_start;
			wall_lowLevelTime += std::chrono::system_clock::now() - wall_ll_start;
			LL_num_expanded += search_engines[ag]->num_expanded;
			LL_num_generated += search_engines[ag]->num_generated;
		}
#endif
	}
	else
	{
		clock_t ll_start = std::clock();
		auto wall_ll_start = std::chrono::system_clock::now();
		foundSol = search_engines[ag]->findPath(*pNewPath, cons_table, *cat, start_location_and_time,
												goal_location_and_time, lowlevel_heuristic::DH);
		lowLevelTime += std::clock() - ll_start;
		wall_lowLevelTime += std::chrono::system_clock::now() - wall_ll_start;
		LL_num_expanded += search_engines[ag]->num_expanded;
		LL_num_generated += search_engines[ag]->num_generated;
	}

	if (!foundSol) {
        if (skipNewpaths)
            delete pNewPath;
        return false;
    }

	// update the_paths, g_val, and makespan
	node->g_val = node->g_val - (int) (*node->all_paths)[ag]->size() + (int) pNewPath->size();
	if (skipNewpaths && (*node->all_paths)[ag] != &paths_found_initially[ag])
		delete (*node->all_paths)[ag];  // Delete the old path (its content is backed up in the outer frame in case we need to backtrack)
	(*node->all_paths)[ag] = pNewPath;
	node->makespan = max(node->makespan, (int)pNewPath->size() - 1);

    if (spdlog::default_logger()->level() <= spdlog::level::info)
    {
        ostringstream msg;
        this->printPaths(msg, *node->all_paths);
        spdlog::info(msg.str());
    }

	return true;
}

// plan paths that are not planned yet due to partial expansion
bool ICBSSearch::finishPartialExpansion(ICBSNode *node, ConflictAvoidanceTable *cat)
{
	for (const auto& agent_id_and_path: node->new_paths)
	{
		int agent_id = agent_id_and_path.first;
		if (agent_id_and_path.second.size() == 1 && agent_id_and_path.second.back().location < 0)
		{
#ifdef CBS_LCA_JUMPING
			// Remove the paren't path for the agent we delayed finding a path for
			removePathFromConflictAvoidanceTable(*(*node->all_paths)[agent_id], *cat, agent_id);
#endif
	        int newConstraintTimestep = -agent_id_and_path.second.back().location;  // HACK: It was saved there during partial expansion.
	        int earliestGoalTimestep = newConstraintTimestep + 1;  // We partially expanded because the agent was moved
	                                                               // away from its goal. There can only be vertex
	                                                               // conflicts when staying at the goal and there's a
	                                                               // single goal so the goal can't be reached at the
	                                                               // same timestep
	        bool success = findPathForSingleAgent(node, cat, newConstraintTimestep, earliestGoalTimestep, agent_id, false);
#ifdef CBS_LCA_JUMPING
            // Add the agent's newly found path
            addPathToConflictAvoidanceTable(*(*node->all_paths)[agent_id], *cat, agent_id);
#endif

	        if (!success)
	        {
	            delete node;
	            return false;
	        }
	        clearConflictsOfAgent(*node, agent_id);  // Carried over from the parent
	    }
	}

	uint64_t delta_g = node->g_val - node->parent->g_val;
    if (delta_g == 0)
        ++num_delta_g_0;
    else if (delta_g == 1)
        ++num_delta_g_1;
    else {
        sum_delta_g_2_or_more += delta_g;
        ++num_delta_g_2_or_more;
    }

	node->h_val = max(node->parent->f_val - node->g_val, 0);  // Clear the partial expansion's H - it was used up
	node->f_val = node->g_val + node->h_val;
	findConflicts(*node);  // Conflicts involving agents whose paths were unchanged in this node were already copied from the parent
	node->count_conflicts();
    sum_num_conflicts += node->num_of_conflicts;
    ++num_num_conflicts;

	HL_num_reexpanded++;
	node->partialExpansion = false;
	return true;
}


//////////////////// TOOLS ///////////////////////////
// check whether the new planned path obeys the constraints -- for debug
bool ICBSSearch::arePathsConsistentWithConstraints(vector<Path *> &paths, ICBSNode *curr) const
{
	if (curr->partialExpansion)
		return true;
	ICBSNode* node = curr;
	while (node != nullptr)
	{
		for (int agent = 0; agent < num_of_agents; ++agent) {
			for (const auto& con : node->negative_constraints[agent]) {
				const auto& [loc1, loc2, timestep, positive_constraint] = con;
				if (loc2 < 0) // vertex constraint
				{
					if (paths[agent]->size() > timestep && paths[agent]->at(timestep).location == loc1)
					{
						spdlog::error("The path of agent {} violates negative constraint {} from node #{}",
						              agent, con, node->time_generated);
						exit(1);
					}
					else if (paths[agent]->size() <= timestep && paths[agent]->back().location == loc1)
					{
						spdlog::error("The path of agent {} violates negative constraint {} from node #{}",
                                      agent, con, node->time_generated);
						exit(1);
					}
				}
				else // edge constraint
				{
					if (timestep < paths[agent]->size() &&  // Otherwise we're fine - can't violate an edge constraint by WAITing
						paths[agent]->at(timestep - 1).location == loc1 &&
						paths[agent]->at(timestep).location == loc2)
					{
                        spdlog::error("The path of agent {} violates negative constraint {} from node #{}",
                                      agent, con, node->time_generated);
						exit(1);
					}
				}
			}

			for (const auto& con : node->positive_constraints[agent]) {
				const auto& [loc1, loc2, timestep, positive_constraint] = con;
				for (int other_agent = 0; other_agent < num_of_agents; other_agent++)
				{
					if (other_agent == agent)
					{
						if (loc2 < 0) // vertex constraint
						{
							if (timestep < paths[other_agent]->size() && paths[other_agent]->at(timestep).location != loc1)
							{
                                spdlog::error("The path of agent {} violates positive constraint {} from node #{}",
                                              agent, con, node->time_generated);
								exit(1);
							}
						}
						else // edge constraint
						{
							if (timestep < paths[other_agent]->size() &&
								(paths[other_agent]->at(timestep - 1).location != loc1 ||
								 paths[other_agent]->at(timestep).location != loc2))
							{
                                spdlog::error("The path of agent {} violates positive constraint {} from node #{}",
                                              agent, con, node->time_generated);
								exit(1);
							}
						}
					}
					else  // The positive constraint is on a different agent - an implicit negative constraint for this agent
					{
						if (timestep >= paths[other_agent]->size())
						{
							if (loc2 < 0 && paths[other_agent]->at(paths[other_agent]->size() - 1).location == loc1)
							{
                                spdlog::error("The path of agent {} violates positive constraint {} on agent {} "
                                              "from node #{}", other_agent, con, agent, node->time_generated);
								exit(1);
							}
						}
						else if (loc2 < 0) // vertex constraint
						{
							if (paths[other_agent]->at(timestep).location == loc1)
							{
                                spdlog::error("The path of agent {} violates positive constraint {} on agent {} "
                                              "from node #{}", other_agent, con, agent, node->time_generated);
								exit(1);
							}
						}
						else // edge constraint
						{
							if ((paths[other_agent]->at(timestep - 1).location == loc2 && paths[other_agent]->at(timestep).location == loc1) ||
								paths[other_agent]->at(timestep - 1).location == loc1 ||
								paths[other_agent]->at(timestep).location == loc2)
							{
                                spdlog::error("The path of agent {} violates positive constraint {} on agent {} "
                                              "from node #{}", other_agent, con, agent, node->time_generated);
								exit(1);
							}
						}
					}
				}
			}
		}

		node = node->parent;
	}
	return true;
}

// check whether the paths are feasible -- for debug
void ICBSSearch::isSolutionFeasible()
{
	bool good = true;
	if (!solution_found) {
	    spdlog::error("Solution not found but success was reported?!?!");
		abort();
	}

	if (solution_cost < min_f_val) {
	    spdlog::error("Solution cost {} < min f val {}?!?!", solution_cost, min_f_val);
		good = false;
	}
	if (solution_cost > min_f_val * focal_w) {
	    spdlog::error("Solution cost {} > min f val {} * focal_w {}!", solution_cost, min_f_val, focal_w);
		good = false;
	}
//	vector<Path*> paths(num_of_agents, nullptr);
//	populatePaths(solution_node, paths);
//	vector<Path*>& paths = *solution_node->all_paths;
	int sum = 0;
	for (int i = 0; i < num_of_agents; i++)
	{
		sum += (int)paths[i]->size() - 1;
		if (paths[i]->front().location != search_engines[i]->start_location) {
        	spdlog::error("Wrong start location for agent {}", i);
        	good = false;
		}
		if (paths[i]->back().location != search_engines[i]->goal_location) {
			spdlog::error("Wrong goal location for agent {}", i);
			good = false;
		}
		for (int t = 1; t < paths[i]->size(); t++)
		{
			if (search_engines[0]->my_map[paths[i]->at(t).location]) {
			    spdlog::error("Path of agent {} goes through an obstacle at timestep {}", i, t);
				good = false;
			}
			int move = paths[i]->at(t).location - paths[i]->at(t - 1).location;
			bool valid = false;
			for (int j = 0; j < 5; j++)
				if (move == search_engines[0]->moves_offset[j])
					valid = true;
			if (!valid) {
			    spdlog::error("Invalid move for agent {} from {} to {}",
			                  i, paths[i]->at(t - 1).location, paths[i]->at(t).location);
                good = false;
            }
		}
		for (int j = i + 1; j < num_of_agents; j++)
		{
			if (paths[i]->at(0).location == paths[j]->at(0).location) {
				spdlog::error("Start position collision between agent {} and {}!", i, j);
				good = false;
			}
			int t_min = (int)min(paths[i]->size(), paths[j]->size());
			for (int t = 1; t < t_min; t++)
			{
				if (paths[i]->at(t).location == paths[j]->at(t).location) {
                    spdlog::error("Vertex collision between agent {} and {} at time {}!", i, j, t);
                    good = false;
				}
				else if (paths[i]->at(t).location == paths[j]->at(t - 1).location &&
					paths[i]->at(t - 1).location == paths[j]->at(t).location) {
					spdlog::error("Edge collision between agent {} and {} at time {}!", i, j, t);
					good = false;
				}
			}
			if (paths[i]->size() == paths[j]->size())
				continue;
			int a, b;
			if (paths[i]->size() < paths[j]->size())
			{
				a = i;
				b = j;
			}
			else
			{
				a = j;
				b = i;
			}
			for (int t = t_min; t < paths[b]->size(); t++)
			{
				if (paths[a]->at(t_min - 1).location == paths[b]->at(t).location) {
                    spdlog::error("Vertex collision between agent {} staying at its goal and agent {} going through "
                                  "it at timestep {}!", a, b, t);
                    good = false;
                }
			}
		}
	}
	if (sum != solution_cost) {
        spdlog::error("Solution cost is wrong!");
        good = false;
	}
	if (!good)
	    abort();
}

// Returns the ID of the location the given agent is in at the given timestep according to the given paths
// of the current node we're working on
inline int ICBSSearch::getAgentLocation(vector<Path *> &paths, size_t timestep, int agent_id)
{
	// if last timestep > plan length, agent remains in its last location
	if (timestep >= paths[agent_id]->size())
		return paths[agent_id]->at(paths[agent_id]->size() - 1).location;
	// otherwise, return its location for that timestep
	return paths[agent_id]->at(timestep).location;
}

// Populates the search's paths member by scanning from the given node towards the root node and
// taking the first constrained path encountered for each agent. Agents with no
// constrained path take the original path they got in the root node.
inline void ICBSSearch::populatePaths(ICBSNode *curr, vector<Path *> &the_paths)
{
	vector<bool> path_already_found(num_of_agents, false);  // initialized with false for all agents
	while (curr != nullptr)
	{
		for (auto& pair : curr->new_paths)
		{
			if (!path_already_found[pair.first] && !(pair.second.size() == 1 && pair.second.back().location < 0))
			{
				the_paths[pair.first] = &(pair.second);
				path_already_found[pair.first] = true;
			}
		}
		curr = curr->parent;
	}
	for (int i = 0; i < num_of_agents; i++)
		if (!path_already_found[i])
			the_paths[i] = &paths_found_initially[i];
}

// adding new nodes to FOCAL (those with min-f-val*f_weight between the old and new LB)
void ICBSSearch::updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight)
{
	for (auto it = open_list.ordered_begin() ; it != open_list.ordered_end() ; ++it)  // Note: The ordered_iterator has a bug where the same item is returned multiple times.
	                                                                                  //       This is why we have to verify n isn't already in FOCAL before adding it to FOCAL.
	{
		auto n = *it;
		if (n->f_val > new_lower_bound) {  // F too large, and all the next ones will also be
		    break;
		}
		else if (!n->in_focal && n->f_val > old_lower_bound && n->f_val <= new_lower_bound) {  // Not already in FOCAL, and F isn't too large
            n->focal_handle = focal_list.push(n);
            n->in_focal = true;
        }
	}
}

// Reinsert a node to OPEN (and FOCAL if needed) because of the lazy heuristics
bool ICBSSearch::reinsert(ICBSNode* curr)
{
	if ((!focal_list.empty() && curr->f_val > focal_list_threshold) ||    // there's something better in FOCAL
        (focal_list.empty() && !open_list.empty() && open_list.top()->f_val < curr->f_val))  // FOCAL is empty but there's something better in OPEN
	{
		curr->open_handle = open_list.push(curr);
		curr->in_open = true;
		ICBSNode* open_head = open_list.top();
		if (open_head->f_val > min_f_val)  // TODO: Explain
		{
			min_f_val = open_head->f_val;
			double focal_list_threshold_candidate = min_f_val * focal_w;
			if (focal_list_threshold_candidate > focal_list_threshold) {
                updateFocalList(focal_list_threshold, focal_list_threshold_candidate, focal_w);
                focal_list_threshold = focal_list_threshold_candidate;
            }
		}
		return true;
	}
	else
		// It's still within the bound, and we already took it out of the list, so just stay with it.
		return false;
}


//////////////////// PRINT ///////////////////////////
void ICBSSearch::printPaths(std::ostream& stream, vector<Path *> &the_paths, int max_len) const
{
	for (int i = 0; i < num_of_agents; i++)
	{
        stream << "Agent " << i << " (" << paths_found_initially[i].size() - 1 << " -->" <<
			the_paths[i]->size() - 1 << "): ";
		if (the_paths[i]->size() < max_len) {
		    int t = 0;
			for ( ; t < the_paths[i]->size() - 1; t++)
                stream << t << ": (" << the_paths[i]->at(t).location / num_map_cols << ","
						  << the_paths[i]->at(t).location % num_map_cols << "), ";
            stream << t << ": (" << the_paths[i]->at(t).location / num_map_cols << ","
                      << the_paths[i]->at(t).location % num_map_cols << ")";
		} else
            stream << "(path too long to print)";
	}
}

void ICBSSearch::printConflicts(std::ostream& stream, const ICBSNode &curr) const
{
	for (const auto& conflict: curr.cardinalGoalConf)
	{
        stream << "Cardinal-goal " << (*conflict) << std::endl;
	}
	for (const auto& conflict: curr.cardinalConf)
	{
        stream << "Cardinal " << (*conflict) << std::endl;
	}
	for (const auto& conflict: curr.semiCardinalGoalConf)
	{
        stream << "Semi-cardinal-goal " << (*conflict) << std::endl;
	}
	for (const auto& conflict : curr.semiCardinalConf)
	{
        stream << "Semi-cardinal " << (*conflict) << std::endl;
	}
	for (const auto& conflict : curr.nonCardinalConf)
	{
        stream << "Non-cardinal " << (*conflict) << std::endl;
	}
	for (const auto& conflict : curr.unknownConf)
	{
        stream << "Unknown-cardinality " << (*conflict) << std::endl;
	}
}

void ICBSSearch::printConstraints(std::ostream& stream, const ICBSNode* n) const
{
	const ICBSNode* curr = n;
	while (curr != nullptr)
	{
		for (int j = 0; j < num_of_agents; ++j) {
			for (const auto& con: curr->positive_constraints[j])
			{
                stream << curr->agent_id << ": " << con << std::endl;
			}
			for (const auto& con: curr->negative_constraints[j])
			{
                stream << curr->agent_id << ": " << con << std::endl;
			}
		}
		curr = curr->parent;
	}
}

void ICBSSearch::printResults() const
{
	std::cout << "Status,Cost,Focal Delta,Root Cost,Root f,"
                 "F-Cardinal Conflicts,G-Cardinal Goal Conflicts,Semi-F-Cardinal Conflicts,"
                 "G-Cardinal Conflicts,Semi-G-Cardinal Goal Conflicts,Semi-G-Cardinal Conflicts,"
                 "Non-Cardinal Conflicts,"
                 "deltaH_0,deltaH_-1,deltaH_1,sum_deltaH_le_-2,count_deltaH_le_-2,sum_deltaH_ge_2,count_deltaH_ge_2,"
                 "deltaG_0,deltaG_1,sum_deltaG_ge_2,count_deltaG_ge_2,"
                 "deltaF_0,deltaF_1,sum_deltaF_ge_2,count_deltaF_ge_2,"
                 "deltaF_0_deltaG_1,deltaF_0_deltaG_0,"
                 "deltaF_1_deltaG_2,deltaF_1_deltaG_1,deltaF_1_deltaG_0,"
                 "sum_num_conflicts,count_num_conflicts,"
                 "Wall PrepTime,PrepTime,Wall MDD Time,MDD Time,"
				 "Wall HL Heuristic Time,HL Heuristic Time,"
                 "Wall CAT Time,CAT Time,Wall HL Node Verification Time,HL Node Verification Time,"
				 "Wall Up&Down Time,Up&Down Time,HL Expanded,HL Generated,LL Expanded,LL Generated,Wall HL runtime,HL runtime,"
				 "Wall LL runtime,LL runtime,Wall Runtime,Runtime,Max Mem (kB)" << std::endl;
	if (runtime > time_limit * CLOCKS_PER_SEC)  // timeout
	{
		std::cout << "Timeout,";
	}
	else if (focal_list.empty() && solution_cost < 0)
	{
		std::cout << "No solutions,";
	}
	else
	{
		std::cout << "Optimal,";
	}
	std::cout << solution_cost << "," <<
		min_f_val - root_node->f_val << "," <<
		root_node->g_val << "," << root_node->f_val << "," <<
		f_cardinal_conflicts_found << "," <<
		cardinal_goal_conflicts_found << "," <<
		semi_f_cardinal_conflicts_found << "," <<
		cardinal_conflicts_found << "," <<
		semi_cardinal_goal_conflicts_found << "," <<
		semi_cardinal_conflicts_found << "," <<
		non_cardinal_conflicts_found << "," <<
		num_delta_h_0 << "," << num_delta_h_minus_1 << "," << num_delta_h_1 << "," <<
		sum_delta_h_minus_2_or_more << "," << num_delta_h_minus_2_or_more << "," <<
		sum_delta_h_2_or_more << "," << num_delta_h_2_or_more << "," <<
		num_delta_g_0 << "," << num_delta_g_1 << "," << sum_delta_g_2_or_more << "," << num_delta_g_2_or_more << "," <<
		num_delta_f_0 << "," << num_delta_f_1 << "," << sum_delta_f_2_or_more << "," << num_delta_f_2_or_more << "," <<
		num_delta_f_0_delta_g_1 << "," << num_delta_f_0_delta_g_0 << "," <<
		num_delta_f_1_delta_g_2 << "," << num_delta_f_1_delta_g_1 << "," << num_delta_f_1_delta_g_0 << "," <<
		sum_num_conflicts << "," << num_num_conflicts << "," <<
		((float) wall_prepTime.count()) / 1000000000 << "," <<
		((float) prepTime) / CLOCKS_PER_SEC << "," <<
		((float) wall_mddTime.count()) / 1000000000 << "," <<
		((float) highLevelMddBuildingTime) / CLOCKS_PER_SEC << "," <<
		((float) wall_hlHeuristicTime.count()) / 1000000000 << "," <<
		((float) hlHeuristicTime) / CLOCKS_PER_SEC << "," <<
		((float) wall_cat_runtime.count()) / 1000000000 << "," <<
		((float) cat_runtime) / CLOCKS_PER_SEC << "," <<
		((float) wall_hl_node_verification_runtime.count()) / 1000000000 << "," <<
		((float) hl_node_verification_runtime) / CLOCKS_PER_SEC << "," <<
		((float) wall_up_and_down_runtime.count()) / 1000000000 << "," <<
		((float) up_and_down_runtime) / CLOCKS_PER_SEC << "," <<
		HL_num_expanded << "," << HL_num_generated << "," <<
		LL_num_expanded << "," << LL_num_generated << "," <<
		((float) wall_highLevelTime.count()) / 1000000000 << "," <<
		((float) highLevelTime) / CLOCKS_PER_SEC << "," <<
		((float) wall_lowLevelTime.count()) / 1000000000 << "," <<
		((float) lowLevelTime) / CLOCKS_PER_SEC << "," <<
		((float) wall_runtime.count() /*+ wall_prepTime.count()*/) / 1000000000 << "," <<
		((float) runtime /*+ prepTime*/) / CLOCKS_PER_SEC << "," <<
		max_mem <<
		std::endl;
}

void ICBSSearch::saveResults(const string& outputFile, const string& agentFile, const string& solver) const
{
	ofstream stats;
	if (std::filesystem::exists(outputFile) == false)
	{
		stats.open(outputFile);
		stats << "Cost,Focal Delta,Root Cost,Root f,"
                 "F-Cardinal Conflicts,G-Cardinal Goal Conflicts,Semi-F-Cardinal Conflicts,"
                 "G-Cardinal Conflicts,Semi-G-Cardinal Goal Conflicts,Semi-G-Cardinal Conflicts,"
                 "Non-Cardinal Conflicts,"
                 "deltaH_0,deltaH_-1,deltaH_1,sum_deltaH_le_-2,count_deltaH_le_-2,sum_deltaH_ge_2,count_deltaH_ge_2,"
                 "deltaG_0,deltaG_1,sum_deltaG_ge_2,count_deltaG_ge_2,"
                 "deltaF_0,deltaF_1,sum_deltaF_ge_2,count_deltaF_ge_2,"
                 "deltaF_0_deltaG_1,deltaF_0_deltaG_0,"
                 "deltaF_1_deltaG_2,deltaF_1_deltaG_1,deltaF_1_deltaG_0,"
                 "sum_num_conflicts,count_num_conflicts,"
                 "Wall PrepTime,PrepTime,Wall MDD Time,MDD Time,"
				 "Wall HL Heuristic Time,HL Heuristic Time,"
		         "Wall CAT Time,CAT Time,Wall HL Node Verification Time,HL Node Verification Time,"
				 "Wall Up&Down Time,Up&Down Time,HL Expanded,HL Generated,LL Expanded,LL Generated,Wall HL runtime,HL runtime,"
				 "Wall LL runtime,LL runtime,Wall Runtime,Runtime,Max Mem (kB),solver,instance,Num of Agents" << std::endl;
	}
	else
		stats.open(outputFile, ios::app);
	stats << solution_cost << "," <<
		min_f_val - root_node->f_val << "," <<
		root_node->g_val << "," << root_node->f_val << "," <<
		f_cardinal_conflicts_found << "," <<
		cardinal_goal_conflicts_found << "," <<
		semi_f_cardinal_conflicts_found << "," <<
		cardinal_conflicts_found << "," <<
		semi_cardinal_goal_conflicts_found << "," <<
		semi_cardinal_conflicts_found << "," <<
		non_cardinal_conflicts_found << "," <<
		num_delta_h_0 << "," << num_delta_h_minus_1 << "," << num_delta_h_1 << "," <<
		sum_delta_h_minus_2_or_more << "," << num_delta_h_minus_2_or_more << "," <<
		sum_delta_h_2_or_more << "," << num_delta_h_2_or_more << "," <<
		num_delta_g_0 << "," << num_delta_g_1 << "," << sum_delta_g_2_or_more << "," << num_delta_g_2_or_more << "," <<
		num_delta_f_0 << "," << num_delta_f_1 << "," << sum_delta_f_2_or_more << "," << num_delta_f_2_or_more << "," <<
		num_delta_f_0_delta_g_1 << "," << num_delta_f_0_delta_g_0 << "," <<
		num_delta_f_1_delta_g_2 << "," << num_delta_f_1_delta_g_1 << "," << num_delta_f_1_delta_g_0 << "," <<
		sum_num_conflicts << "," << num_num_conflicts << "," <<
		((float) wall_prepTime.count()) / 1000000000 << "," <<
		((float) prepTime) / CLOCKS_PER_SEC << "," <<
		((float) wall_mddTime.count()) / 1000000000 << "," <<
		((float) highLevelMddBuildingTime) / CLOCKS_PER_SEC << "," <<
		((float) wall_hlHeuristicTime.count()) / 1000000000 << "," <<
		((float) hlHeuristicTime) / CLOCKS_PER_SEC << "," <<
		((float) wall_cat_runtime.count()) / 1000000000 << "," <<
		((float) cat_runtime) / CLOCKS_PER_SEC << "," <<
		((float) wall_hl_node_verification_runtime.count()) / 1000000000 << "," <<
		((float) hl_node_verification_runtime) / CLOCKS_PER_SEC << "," <<
		((float) wall_up_and_down_runtime.count()) / 1000000000 << "," <<
		((float) up_and_down_runtime) / CLOCKS_PER_SEC << "," <<
		HL_num_expanded << "," << HL_num_generated << "," <<
		LL_num_expanded << "," << LL_num_generated << "," <<
		((float) wall_highLevelTime.count()) / 1000000000 << "," <<
		((float) highLevelTime) / CLOCKS_PER_SEC << "," <<
		((float) wall_lowLevelTime.count()) / 1000000000 << "," <<
		((float) lowLevelTime) / CLOCKS_PER_SEC << "," <<
		((float) wall_runtime.count() /*+ wall_prepTime.count()*/) / 1000000000 << "," <<
		((float) runtime /*+ prepTime*/) / CLOCKS_PER_SEC << "," <<
		max_mem << "," <<
		solver << "," << agentFile << "," << num_of_agents << endl;
	stats.close();
}

//////////////////// MAIN FUNCTIONS ///////////////////////////
void ICBSSearch::findShortestPathFromPrevNodeToCurr(ICBSNode *curr, ICBSNode* prev,
													vector<ICBSNode *>& steps_up_from_prev_node_to_lowest_common_ancestor,
													vector<ICBSNode *>& steps_down_from_lowest_common_ancestor_to_curr_node) {
	// TODO: Consider implementing the online lowest common ancestor algorithm from https://hackage.haskell.org/package/lca
	std::set<ICBSNode *> branch_of_curr;
	ICBSNode *node = curr;
	while (node != nullptr) {
		if (node == prev) {  // happy case - node is a direct descendant of prev
			std::reverse(steps_down_from_lowest_common_ancestor_to_curr_node.begin(), steps_down_from_lowest_common_ancestor_to_curr_node.end());
			return;
		}
		steps_down_from_lowest_common_ancestor_to_curr_node.push_back(node);
		branch_of_curr.insert(node);
		node = node->parent;
	}
	node = prev;
	auto end_of_branch_of_curr = branch_of_curr.end();
	while (node != nullptr) {
		if (branch_of_curr.find(node) != end_of_branch_of_curr) {
			while (steps_down_from_lowest_common_ancestor_to_curr_node.back() != node) {
				steps_down_from_lowest_common_ancestor_to_curr_node.pop_back();
			}
			steps_down_from_lowest_common_ancestor_to_curr_node.pop_back();  // this common parent itself shouldn't be a step
			std::reverse(steps_down_from_lowest_common_ancestor_to_curr_node.begin(), steps_down_from_lowest_common_ancestor_to_curr_node.end());
			return;
		}
		steps_up_from_prev_node_to_lowest_common_ancestor.push_back(node);
		node = node->parent;
	}
	spdlog::critical("Lowest common ancestor not found?!?!");
	std::abort();
}

// RUN CBS
bool ICBSSearch::runICBSSearch()
{
	if (HL_heuristic != highlevel_heuristic::NONE)
#ifndef LPA
        spdlog::info("CBSH({}): ", focal_w);
#else
        spdlog::info("CBSH/LPA*: ");
#endif
	else
#ifndef LPA
        spdlog::info("ICBS({}): ", focal_w);
#else
        spdlog::info("ICBS/LPA*: ");
#endif

	// Print the root node's paths
	populatePaths(root_node, paths);
	root_node->all_paths = &paths;
    if (spdlog::default_logger()->level() <= spdlog::level::info)
    {
        ostringstream msg;
        msg << "Root node's paths: " << std::endl;
        this->printPaths(msg, paths);
        spdlog::info(msg.str());
    }

	ConflictAvoidanceTable* cat = nullptr;
#ifdef CBS_LCA_JUMPING
	// Add the path of the last agent to the running CAT
	addPathToConflictAvoidanceTable(*(*root_node->all_paths)[num_of_agents-1], root_cat, num_of_agents-1);
	cat = &root_cat;
#endif

	ICBSNode* prev_node = nullptr;
    int prev_f = -1;  // Has to be maintained separately because we change the F when we do partial expansion

	int curr_child_pref_budget = child_pref_budget;

	bool continue_working_on_last_node = false;
	bool currently_reexpanding_last_node = false;

	// set timer
	start = std::clock() - runtime;  // Count the time spent in the constructor
	runtime = 0;
	auto wall_start = std::chrono::system_clock::now() - wall_runtime;

	while (!focal_list.empty() || continue_working_on_last_node)
	{
		runtime = std::clock() - start;
		wall_runtime = std::chrono::system_clock::now() - wall_start;
		if (runtime > time_limit * CLOCKS_PER_SEC)  // timeout
		{
		    spdlog::warn("Timed out in CBS high level");
			break;
		}

		ICBSNode *curr = nullptr;
		if (!continue_working_on_last_node) {
            currently_reexpanding_last_node = false;
            // pop the best node from FOCAL
#ifndef CBS_LCA_JUMPING
            curr = focal_list.top();
            focal_list.pop();
            curr->in_focal = false;
#else
            // Prefer children of the previous node we worked on
            curr = nullptr;
            if (prev_node != nullptr) {
                if (curr_child_pref_budget > 0) {
                    curr_child_pref_budget--;
                    for (auto it = focal_list.ordered_begin() ; it != focal_list.ordered_end() ; ++it) {  // I don't care if elements are checked twice here
                        ICBSNode *node = *it;
                        if (node->parent == prev_node) {
                            if (node == focal_list.top())
                                curr_child_pref_budget = child_pref_budget;  // We would have taken it anyway
                            else {
                                spdlog::info("Found an immediate child of the previous node in FOCAL! "
                                             "Choosing #{} instead of the top of FOCAL #{}",
                                             node->time_generated, focal_list.top()->time_generated);
                            }
                            focal_list.erase(node->focal_handle);
                            node->in_focal = false;
                            curr = node;
                            break;
                        }
                    }
    				if (curr == nullptr) {  // An immediate child was not found, check the first X entries in FOCAL for the closest node to prev_node
    					int i = 0;
    					int min_dist = numeric_limits<int>::max();
    					for (auto it = focal_list.ordered_begin() ; it != focal_list.ordered_end() && i < max_child_pref_options ; ++it, ++i)
    					    // Ordered iteration seems to have a bug where items appear multiple
    					    // times, but that's not too bad here.
                        {
                            ICBSNode *node = *it;
    						int dist_from_prev = prev_node->get_up_and_down_distance(node);
    						if (dist_from_prev == 2) {  // The minimum, since no immediate children of prev are in FOCAL
                                curr = node;
                                break;
                            }
    						if (dist_from_prev < min_dist) {
    							curr = node;
    							min_dist = dist_from_prev;
    						}
    					}
    					if (curr != nullptr) {
    						if (curr == focal_list.top())
    							curr_child_pref_budget = child_pref_budget;  // We would have taken it anyway
                            else {
                                spdlog::info("Chose #{} instead of the top of FOCAL #{} because it's closer to prev_node.",
                                             curr->time_generated, focal_list.top()->time_generated);
                            }
    						focal_list.erase(curr->focal_handle);
                            curr->in_focal = false;
    					}
    				}
                }
                else {
                    spdlog::info("Child pref budget exhausted. Taking the top of FOCAL and refreshing the budget.");
                }
            }
            // TODO: Prefer nodes that are closer to prev_node in the tree. More precisely, nodes whose path to prev_node
            //	   has the least edges that concern the agents that participate in the conflict they chose.
            //	   (We know updating the CAT, which is done for all HL edges, is very cheap)
            if (curr == nullptr) {
                curr = focal_list.top();
                focal_list.pop();
                curr->in_focal = false;
                curr_child_pref_budget = child_pref_budget;
            }
#endif
            open_list.erase(curr->open_handle);
            curr->in_open = false;
        }
		else {
		    curr = prev_node;
		    continue_working_on_last_node = false;
			currently_reexpanding_last_node = true;
		}


        if (focal_w == 1) {  // Must do this check after the h of curr has been computed.
            // If we check right after we pop from OPEN, we're comparing against
            // the previous node's F after popping it from OPEN, which may still
            // have had remnants of the parent's H even though its H has not yet
            // been computed at the time.
            if (prev_f > curr->f_val) {
                // TODO: Copy this check to IDCBS
                spdlog::error("F value of #{}={} < F value of prev node #{}={}",
                              curr->time_generated, curr->f_val, prev_node->time_generated, prev_f);
                abort();
            }
        }

        if (!currently_reexpanding_last_node) {
            prev_f = curr->f_val;

            // get current solutions
            populatePaths(curr, paths);
            curr->all_paths = &paths;

            update_cat_and_lpas(prev_node, curr, paths, cat);
            prev_node = curr;
        }

		if (curr->partialExpansion)
		{
			spdlog::info("Finishing partial expansion of node #{}", curr->time_generated);
			bool solved = finishPartialExpansion(curr, cat);
			if (!solved) {
				// Node was deleted. Don't use that pointer for anything
                spdlog::info("No solution found! (Probably timed out) Moving on to the next node.");
				allNodes_table.remove(curr);
				continue;
			}
			if (reinsert(curr)) {
                spdlog::info("Pushing #{} back into OPEN with updated cost f= {}+{} and #conflicts={}",
					    curr->time_generated, curr->g_val, curr->h_val, curr->num_of_conflicts);
				continue;
			}
			else {
                spdlog::info("Continuing to work on #{}", curr->time_generated);
			}
		}
		if (HL_heuristic == highlevel_heuristic::NONE) // No heuristics
		{
            classifyConflicts(*curr, cat);  // classify conflicts
            curr->conflict = getHighestPriorityConflict(*curr);  // choose one to work on

            if (spdlog::default_logger()->level() < spdlog::level::warn)
            {
                ostringstream msg;
                printConflicts(msg, *curr);
                spdlog::info(msg.str());
            }
		}
		else if (curr->conflict == nullptr) // CBSH, and h value has not been computed yet
		{
            classifyConflicts(*curr, cat);  // classify conflicts
            curr->conflict = getHighestPriorityConflict(*curr);  // choose one to work on

            if (!currently_reexpanding_last_node) {
                auto hlHeuristicStart = std::clock();
                auto wall_hlHeuristicStart = std::chrono::system_clock::now();
                curr->h_val = computeHeuristic(*curr);
                hlHeuristicTime += std::clock() - hlHeuristicStart;
                wall_hlHeuristicTime += std::chrono::system_clock::now() - wall_hlHeuristicStart;
                curr->f_val = curr->g_val + curr->h_val;
                if (curr->parent != nullptr) {
                    int64_t delta_h = curr->h_val - curr->parent->h_val;
                    uint64_t delta_f = curr->f_val - curr->parent->f_val;

                    if (delta_h == 0)
                        ++num_delta_h_0;
                    else if (delta_h == 1)
                        ++num_delta_h_1;
                    else if (delta_h == -1)
                        ++num_delta_h_minus_1;
                    else if (delta_h < -1) {
                        sum_delta_h_minus_2_or_more += delta_h;
                        ++num_delta_h_minus_2_or_more;
                    } else {
                        sum_delta_h_2_or_more += delta_h;
                        ++num_delta_h_2_or_more;
                    }

                    if (delta_f == 0) {
                        ++num_delta_f_0;
                        // delta_h == 1 is impossible because the cost can't decrease
                        if (delta_h == 0)
                            ++num_delta_f_0_delta_g_0;  // We resolved a non-cardinal conflict (or semi-cardinal from the
                            // non-cardinal side) and didn't get a new cardinal conflict in the
                            // new path (more likely)
                        else if (delta_h == -1)
                            ++num_delta_f_0_delta_g_1;  // We resolved a cardinal conflict from a side that was in the MVC
                    } else if (delta_f == 1) {
                        ++num_delta_f_1;
                        if (delta_h == 1)
                            ++num_delta_f_1_delta_g_0;  // We resolved a non-cardinal (or semi-cardinal from the
                            // non-cardinal side) conflict and got a new cardinal conflict
                            // (less likely)
                        else if (delta_h == 0)
                            ++num_delta_f_1_delta_g_1;  // We resolved a semi-cardinal conflict from the cardinal side
                            // or a cardinal conflict from a side that wasn't in the MVC
                        else if (delta_h == -1)
                            ++num_delta_f_1_delta_g_2;  // We resolved a cardinal goal conflict from the goal side and the
                        // goal side was in the MVC or
                        // resolved a cardinal conflict in a disjoint way and got lucky
                    } else {
                        sum_delta_f_2_or_more += delta_f;
                        ++num_delta_f_2_or_more;
                    }
                }

                if (spdlog::default_logger()->level() <= spdlog::level::info) {
                    ostringstream msg;
                    msg << "****** Computed h for #" << curr->time_generated <<
                              " with f=" << curr->g_val << "+" << curr->h_val << " (";
                    for (int i = 0; i < num_of_agents; i++)
                        msg << paths[i]->size() - 1 << ", ";
                    msg << ") and #conflicts=" << curr->num_of_conflicts << std::endl;
                    printConflicts(msg, *curr);
                    spdlog::info(msg.str());
                }

                if (reinsert(curr)) {
                    spdlog::info("Re-inserted #{} into OPEN with higher h!", curr->time_generated);
                    continue;
                }
            }
            else {
                if (spdlog::default_logger()->level() <= spdlog::level::info) {
                    ostringstream msg;
                    msg << "****** Same node #" << curr->time_generated <<
                              " with f=" << curr->g_val << "+" << curr->h_val << " (";
                    for (int i = 0; i < num_of_agents; i++)
                        std::cout << paths[i]->size() - 1 << ", ";
                    std::cout << ") and #conflicts=" << curr->num_of_conflicts << std::endl;
                    printConflicts(*curr);
                }
            }
		}

		prev_f = curr->f_val;  // Refresh it after computing the heuristic and not re-inserting

		if (curr->conflict == nullptr) // Failed to find a conflict => no conflicts
		{  // found a solution (and finish the while loop)
			solution_found = true;
			solution_cost = curr->g_val;
			solution_node = curr;
			break;
		}

		// Expand the node
		HL_num_expanded++;
		curr->time_expanded = HL_num_expanded;

        if (spdlog::default_logger()->level() <= spdlog::level::info) {
            ostringstream msg;
			msg << "****** Expanding #" << curr->time_generated
				<< " with f= " << curr->g_val << "+" << curr->h_val << " (";
			for (int i = 0; i < num_of_agents; i++)
				msg << paths[i]->size() - 1 << ", ";
			msg << ")";
            spdlog::info(msg.str());
            spdlog::info("Chosen conflict {}", *curr->conflict);
		}

		if (split == split_strategy::DISJOINT3)
		{
//			ICBSNode* n1 = new ICBSNode(curr);
//			ICBSNode* n2 = new ICBSNode(curr);
//			ICBSNode* n3 = new ICBSNode(curr);
//			const auto& [agent1_id, agent2_id, location1, location2, timestep] = *curr->conflict;
//			n1->agent_id = agent1_id;
//			n2->agent_id = agent2_id;
//			n3->agent_id = (1 + agent1_id) * num_of_agents + agent2_id;
//			if (location2 < 0) // vertex conflict
//			{
//				LL_num_generated += n1->add_constraint(make_tuple(location1, -1, timestep, true));
//				LL_num_generated += n2->add_constraint(make_tuple(location1, -1, timestep, true));
//				LL_num_generated += n3->add_constraint(make_tuple(location1, -1, timestep, false));
//			}
//			else // edge conflict
//			{
//				LL_num_generated += n1->add_constraint(make_tuple(location1, location2, timestep, true));
//				LL_num_generated += n2->add_constraint(make_tuple(location2, location1, timestep, true));
//				LL_num_generated += n3->add_constraint(make_tuple(location1, location2, timestep, false));
//			}
//			vector<Path*> copy(paths);
//			bool Sol1 = generateChild(n1);
//			paths = copy;
//			bool Sol2 = generateChild(n2);
//			paths = copy;
//			bool Sol3 = generateChild(n3);
		}
		else
		{
			ICBSNode* n1 = new ICBSNode(curr, true);
			ICBSNode* n2 = new ICBSNode(curr, false);

#ifndef CBS_LCA_JUMPING
			branch(curr, n1, n2); // add constraints to child nodes

			vector<Path*> temp(paths);
			n1->all_paths = &paths;
			auto [child1, continue_to_next_child1] = generateChild(n1); // plan paths for n1
            if (child1 != nullptr) {
                spdlog::info("Generated left child #{} with f= {}+{} and {} conflicts",
                             n1->time_generated, n1->g_val, n1->h_val, n1->num_of_conflicts);
            } else if (continue_to_next_child1) {
                spdlog::info("No feasible solution for left child!");
            }
            if (!continue_to_next_child1) {
                spdlog::info("Left child found a bypass - adopting its new paths without splitting");
            }
			if (child1 != nullptr && continue_to_next_child1)  // Might have been deleted. FIXME: bad design.
			    n1->all_paths = nullptr;

            if (!continue_to_next_child1) {
                continue_working_on_last_node = true;
            }
            else {
                paths = temp;
                n2->all_paths = &paths;
                auto [child2, continue_to_next_child2] = generateChild(n2); // plan paths for n2

                if (child2 != nullptr) {
                    spdlog::info("Generated right child #{} with f= {}+{} and {} conflicts",
                                 n2->time_generated, n2->g_val, n2->h_val, n2->num_of_conflicts);
                } else if (continue_to_next_child2) {
                    spdlog::info("No feasible solution for right child!");
                }
                if (!continue_to_next_child2) {
                    spdlog::info("Right child found a bypass - adopting its new paths without splitting");
                }
                if (child2 != nullptr && continue_to_next_child2)
                    n2->all_paths = nullptr;


                if (!continue_to_next_child2)
                    continue_working_on_last_node = true;

                if (continue_to_next_child1 && continue_to_next_child2) {
                    // No bypass found. Insert children in OPEN and FOCAL
                    if (child1 != nullptr)
                        push_child_into_lists(child1);
                    if (child2 != nullptr)
                        push_child_into_lists(child2);
                }
            }
#else
            continue_working_on_last_node = branch_and_generate_with_up_and_down(curr, n1, n2, cat); // add constraints to child nodes
#endif
		}
		if (!continue_working_on_last_node) {
            curr->clear();
            if (open_list.empty()) {  // This check is needed before we look at open_list.top() below
                solution_found = false;
                break;
            }
            ICBSNode *open_head = open_list.top();
            if (open_head->f_val > min_f_val) {
                min_f_val = open_head->f_val;
                double focal_list_threshold_candidate = min_f_val * focal_w;
                if (focal_list_threshold_candidate > focal_list_threshold) {
                    auto old_focal_list_size = focal_list.size();
                    auto old_open_list_size = open_list.size();
                    updateFocalList(focal_list_threshold, focal_list_threshold_candidate, focal_w);
                    focal_list_threshold = focal_list_threshold_candidate;
                    spdlog::info("FOCAL update from |FOCAL|={} with |OPEN|={} to |FOCAL|={}",
                                  old_focal_list_size, old_open_list_size, focal_list.size());
                }
            }
            spdlog::info("New FOCAL threshold = {}", focal_list_threshold);

            curr->all_paths = nullptr;
        }
		else {
		    curr->conflict = nullptr;  // It was resolved. Choose a new one.
		}
    }  // end of while loop

	runtime = std::clock() - start;
	wall_runtime = std::chrono::system_clock::now() - wall_start;
	highLevelTime = runtime - lowLevelTime - hl_node_verification_runtime - hlHeuristicTime - up_and_down_runtime
                    - cat_runtime - highLevelMddBuildingTime;
	wall_highLevelTime = wall_runtime - wall_lowLevelTime - wall_hl_node_verification_runtime - wall_hlHeuristicTime
                         - wall_up_and_down_runtime - wall_cat_runtime - wall_mddTime;
	if (spdlog::default_logger()->level() <= spdlog::level::info)
    {
	    ostringstream msg;
        printPaths(msg, paths);
        spdlog::info(msg.str());
    }
	return solution_found;
}

void ICBSSearch::update_cat_and_lpas(ICBSNode *prev_node, ICBSNode *curr,
									 vector<Path*>& paths, ConflictAvoidanceTable *cat) {
#ifdef CBS_LCA_JUMPING
    if (prev_node != nullptr) {
		auto up_and_down_start = std::clock();
		auto wall_up_and_down_start = std::chrono::system_clock::now();

		vector<ICBSNode *> steps_up_from_prev_node_to_lowest_common_ancestor;
		vector<ICBSNode *> steps_down_from_lowest_common_ancestor_to_curr_node;
		findShortestPathFromPrevNodeToCurr(curr, prev_node,
										   steps_up_from_prev_node_to_lowest_common_ancestor,
										   steps_down_from_lowest_common_ancestor_to_curr_node);  // TODO: Optimize using the bitfields!
        spdlog::info("Moving from #{} at depth {} to #{} at depth {} in {} steps up and {} steps down:",
		        prev_node->time_generated, prev_node->depth, curr->time_generated, curr->depth,
		        steps_up_from_prev_node_to_lowest_common_ancestor.size(),
		        steps_down_from_lowest_common_ancestor_to_curr_node.size());

		lca_jumping_runtime += std::clock() - up_and_down_start;
		wall_lca_jumping_runtime += std::chrono::system_clock::now() - wall_up_and_down_start;  // Don't over-count - the CAT time is counted separately
		// Prepare the CAT

		// for every step up from prev_node to curr (might be empty):
		//   for every path in new_paths that wasn't already deleted from the CAT:
		//	  removePathFromConflictAvoidance(cat, path);
		// for every step down from prev_node to curr (can't be empty):
		//   for every path in new_paths that wasn't already deleted from the CAT:
		//	  removePathFromConflictAvoidance(cat, prev_path);
		// for every path we removed:
		//   addPathToConflictAvoidance(cat, curr node's path for the same agent)

		// Remove paths from the CAT
		std::set<int> ids_of_agents_whose_paths_we_removed;
		auto end_of_ids_of_agents_whose_paths_we_removed = ids_of_agents_whose_paths_we_removed.end();
		for (auto node : steps_up_from_prev_node_to_lowest_common_ancestor) {
			for (auto &agent_id_and_new_path : node->new_paths) {
				if (ids_of_agents_whose_paths_we_removed.find(agent_id_and_new_path.first) ==
					end_of_ids_of_agents_whose_paths_we_removed) {  // not already deleted
					removePathFromConflictAvoidanceTable(agent_id_and_new_path.second, *cat, agent_id_and_new_path.first);
					ids_of_agents_whose_paths_we_removed.insert(agent_id_and_new_path.first);
				}
			}
		}
		ICBSNode *lowestCommonAncestor = steps_down_from_lowest_common_ancestor_to_curr_node.front()->parent;
		vector<Path *> lca_paths(num_of_agents,
											  nullptr);  // The paths of the node we're currently working on.
		populatePaths(lowestCommonAncestor, lca_paths);
		for (auto node : steps_down_from_lowest_common_ancestor_to_curr_node) {
			for (const auto& agent_id_and_new_path : node->new_paths) {
				if (ids_of_agents_whose_paths_we_removed.find(agent_id_and_new_path.first) ==
					end_of_ids_of_agents_whose_paths_we_removed) {  // not already deleted
					removePathFromConflictAvoidanceTable(*lca_paths[agent_id_and_new_path.first], *cat, agent_id_and_new_path.first);
					ids_of_agents_whose_paths_we_removed.insert(agent_id_and_new_path.first);
				}
			}
		}
		// Add the current node's paths to the CAT for each path we removed
		for (auto agent_id : ids_of_agents_whose_paths_we_removed) {
			addPathToConflictAvoidanceTable(*paths[agent_id], *cat, agent_id);
		}
		// The CAT is finally ready, now update the LPA*s

		// for every step up from prev_node to curr (might be empty):
		//   for every new constraint:
		//	  root_node->lpas[constrained_agent]->remove_constraint(constraint, cat)
		//   for every new constraint:
		//	  root_node->lpas[constrained_agent]->add_constraint(constraint, cat)
		// for every step down from prev_node to curr (can't be empty):

#ifndef LPA
#else
        up_and_down_start = std::clock();
        wall_up_and_down_start = std::chrono::system_clock::now();

		for (auto node : steps_up_from_prev_node_to_lowest_common_ancestor) {
			for (int j = 0; j < node->negative_constraints.size(); ++j) {
				for (const auto& constraint : node->negative_constraints[j]) {
					spdlog::debug("Removing constraint {} from agent {}", constraint, j);
                    LL_num_generated += node->pop_lpa_constraint(j, constraint, cat);
				}
			}
			// TODO: Handle positive constraints
		}
		for (auto node : steps_down_from_lowest_common_ancestor_to_curr_node) {
			for (int j = 0; j < node->negative_constraints.size(); ++j) {
				for (const auto& constraint : node->negative_constraints[j]) {
					spdlog::debug("Adding constraint {} to agent {}", constraint, j);
                    LL_num_generated += node->add_lpa_constraint(j, constraint, cat, true);
				}
			}
			// TODO: Handle positive constraints
		}

        lca_jumping_runtime += std::clock() - up_and_down_start;
        wall_lca_jumping_runtime += std::chrono::system_clock::now() - wall_up_and_down_start;
#endif

		// all that work just to get to the node's starting state

		// and when expanding, remove the constraint on agent1 after generating the left child and restore
		// the agent's path before _adding the constraint for agent2_ and generating the right child
		// and remove the constraint on agent2 in the end before finishing expansion. and fiddle with the CAT too.
    }
#endif
}

// RUN ID-CBS/LPA*
bool ICBSSearch::runIterativeDeepeningICBSSearch()
{
	if (HL_heuristic != highlevel_heuristic::NONE)
#ifndef LPA
        spdlog::info("ID-CBSH({}):", focal_w);
#else
		spdlog::info("ID-CBSH/LPA*:");
#endif
	else
#ifndef LPA
        spdlog::info("ID-ICBS({}):", focal_w);
#else
        spdlog::info("ID-ICBS/LPA*:");
#endif
	// set timer
	start = std::clock() - runtime;  // Count the time spent in the constructor
	auto wall_start = std::chrono::system_clock::now() - wall_runtime;  // Count the time spent in the constructor

	populatePaths(root_node, paths);
	root_node->all_paths = &paths;

	// Add the path of the last agent to the running CAT
	addPathToConflictAvoidanceTable(*(*root_node->all_paths)[num_of_agents-1], root_cat, num_of_agents-1);

	classifyConflicts(*root_node, &root_cat);  // classify conflicts
	root_node->conflict = getHighestPriorityConflict(*root_node);  // choose one to work on

	// Compute the root node's h value, to be used for setting the first iteration's threshold:
	if (HL_heuristic != highlevel_heuristic::NONE) {
		root_node->h_val = computeHeuristic(*root_node);
        if (spdlog::default_logger()->level() <= spdlog::level::info) {
            ostringstream msg;
			msg << "****** Computed h for #" << root_node->time_generated <<
				   " with f= " << root_node->g_val << "+" << root_node->h_val << " (";
			for (int i = 0; i < num_of_agents; i++)
				msg << (*root_node->all_paths)[i]->size() - 1 << ", ";
			msg << ") and #conflicts = " << root_node->num_of_conflicts;
			spdlog::info(msg.str());
		}
	}
	else
		root_node->h_val = 0;
    if (spdlog::default_logger()->level() <= spdlog::level::info) {
        ostringstream msg;
		printConflicts(msg, *root_node);
        spdlog::info(msg.str());
	}
	root_node->f_val = root_node->g_val + root_node->h_val;

	// Set the first threshold
	int threshold = root_node->f_val;

	auto logger = spdlog::get("ID");

	while (true)
	{
		if (std::clock() - start > time_limit * CLOCKS_PER_SEC)  // timeout
		{
            spdlog::warn("Timed out in IDCBS high level");
			break;
		}

        logger->info("IDCBS Threshold: {}", threshold);
		root_node->time_generated = 1;
		HL_num_expanded_before_this_iteration = HL_num_expanded;
		HL_num_generated_before_this_iteration = HL_num_generated;
		HL_num_generated_before_this_iteration--;  // Simulate that the root node was generated for this iteration
		auto [solved, next_threshold] = do_idcbsh_iteration(root_node, root_cat,
															threshold, std::numeric_limits<int>::max(),
															start + time_limit * CLOCKS_PER_SEC);
		if (solved) {
			break;
		}
        logger->info("Finished IDCBS threshold {}. Expanded {} nodes and generated {} nodes in {} seconds.",
					  threshold, HL_num_expanded - HL_num_expanded_before_this_iteration,
                     HL_num_generated - HL_num_generated_before_this_iteration,
					  ((float) (std::clock() - start)) / CLOCKS_PER_SEC);
		if (threshold == next_threshold) { // Unsolvable instance?
            logger->warn("Next threshold not found! Unsolvable??");
			break;
		}

        if (spdlog::default_logger()->level() < spdlog::level::warn) {
			for (const auto &agent_neg_constraints: root_node->negative_constraints) {
				if (agent_neg_constraints.size() != 0) {
					spdlog::critical("IDCBS root has negative constraints at end of threshold {}", threshold);
					std::abort();
				}
			}
            for (const auto &agent_pos_constraints: root_node->positive_constraints) {
                if (agent_pos_constraints.size() != 0) {
                    spdlog::critical("IDCBS root has negative constraints at end of threshold {}", threshold);
                    std::abort();
                }
            }
            for (auto lpa: root_node->lpas) {
                for (const auto &dyn_constraints_for_timestep : lpa->dcm.dyn_constraints_) {
                    if (dyn_constraints_for_timestep.size() != 0) {
                        spdlog::critical("LPA* for agent {} has constraints at end of threshold {}",
                                         lpa->agent_id, threshold);
                        std::abort();
                    }
                }
            }
        }

		threshold = next_threshold;
	}  // end of while loop

	runtime = std::clock() - start; //  get time
	wall_runtime = std::chrono::system_clock::now() - wall_start;
	highLevelTime = runtime - lowLevelTime - hl_node_verification_runtime - hlHeuristicTime - up_and_down_runtime - cat_runtime - highLevelMddBuildingTime;
	wall_highLevelTime = wall_runtime - wall_lowLevelTime - wall_hl_node_verification_runtime - wall_hlHeuristicTime - wall_up_and_down_runtime - wall_cat_runtime - wall_mddTime;
    if (spdlog::default_logger()->level() <= spdlog::level::info)
    {
        ostringstream msg;
        printPaths(msg, paths);
        spdlog::info(msg.str());
    }
	return solution_found;
}

// Returns <success, next_threshold>
std::tuple<bool, int> ICBSSearch::do_idcbsh_iteration(ICBSNode *curr,
													  ConflictAvoidanceTable &cat,
													  int threshold, int next_threshold, clock_t end_by, bool after_bypass /*= false*/) {
	if (std::clock() > end_by) {  // timeout (no need to unconstrain)
		spdlog::warn("Timed out in idcbsh iteration!");
		return make_tuple(false, next_threshold);
	}

	vector<Path *> &paths = *curr->all_paths;

	if (HL_heuristic == highlevel_heuristic::NONE) // Then conflicts are not yet classified
	{
		classifyConflicts(*curr, &cat);  // classify conflicts
		curr->conflict = getHighestPriorityConflict(*curr);  // choose one to work on

        if (spdlog::default_logger()->level() < spdlog::level::warn)
        {
            ostringstream msg;
            printConflicts(msg, *curr);
            spdlog::info(msg.str());
        }
	} else if (curr->conflict == nullptr) // CBSH, and h value has not been computed yet
	{
		classifyConflicts(*curr, &cat);  // classify conflicts
		curr->conflict = getHighestPriorityConflict(*curr);  // choose one to work on

		int orig_h = curr->h_val;
		int orig_f = curr->f_val;
		if (!after_bypass)
			curr->h_val = computeHeuristic(*curr);
		curr->f_val = curr->g_val + curr->h_val;
		int64_t delta_h = curr->h_val - orig_h;
        if (delta_h == 0)
            ++num_delta_h_0;
        else if (delta_h == 1)
            ++num_delta_h_1;
        else if (delta_h == -1)
            ++num_delta_h_minus_1;
        else if (delta_h < -1) {
            sum_delta_h_minus_2_or_more += delta_h;
            ++num_delta_h_minus_2_or_more;
        }
        else {
            sum_delta_h_2_or_more += delta_h;
            ++num_delta_h_2_or_more;
        }
        uint64_t delta_f = curr->f_val - orig_f;
        if (delta_f == 0) {
            ++num_delta_f_0;
            // delta_h == 1 is impossible because the cost can't decrease
            if (delta_h == 0)
                ++num_delta_f_0_delta_g_0;  // We resolved a non-cardinal conflict (or semi-cardinal from the
                // non-cardinal side) and didn't get a new cardinal conflict in the
                // new path (more likely)
            else if (delta_h == -1)
                ++num_delta_f_0_delta_g_1;  // We resolved a cardinal conflict from a side that was in the MVC
        }
        else if (delta_f == 1) {
            ++num_delta_f_1;
            if (delta_h == 1)
                ++num_delta_f_1_delta_g_0;  // We resolved a non-cardinal (or semi-cardinal from the
                // non-cardinal side) conflict and got a new cardinal conflict
                // (less likely)
            else if (delta_h == 0)
                ++num_delta_f_1_delta_g_1;  // We resolved a semi-cardinal conflict from the cardinal side
                // or a cardinal conflict from a side that wasn't in the MVC
            else if (delta_h == -1)
                ++num_delta_f_1_delta_g_2;  // We resolved a cardinal goal conflict from the goal side and the
            // goal side was in the MVC or
            // resolved a cardinal conflict in a disjoint way and got lucky
        }
        else {
            sum_delta_f_2_or_more += delta_f;
            ++num_delta_f_2_or_more;
        }

        if (spdlog::default_logger()->level() <= spdlog::level::info) {
            ostringstream msg;
			if (!after_bypass) {
			    msg << "****** Computed h for #" << curr->time_generated <<
			           " with f= " << curr->g_val << "+" << curr->h_val << " (";
			}
			else {
			    msg << std::endl << "****** Same node #" << curr->time_generated
			              << " with f= " << curr->g_val << "+" << curr->h_val << " (";
			}
			for (int i = 0; i < num_of_agents; i++)
			    msg << paths[i]->size() - 1 << ", ";
			msg << ") and #conflicts = " << curr->num_of_conflicts << std::endl;
            printConflicts(msg, *curr);
            spdlog::info(msg.str());
		}

		if (curr->f_val > threshold * focal_w) {
			spdlog::info("F value {} > {} threshold * {} focal_w. Backtracking.", curr->f_val, threshold, focal_w);

			next_threshold = min(next_threshold, curr->f_val);
			return make_tuple(false, next_threshold);  // The parent will unconstrain
		}
	}

	if (curr->conflict == nullptr) // Failed to find a conflict => no conflicts => found a solution
	{
		solution_found = true;
		solution_cost = curr->g_val;
		solution_node = curr;  // Actually also the only node - the root
		min_f_val = curr->f_val;  // Just to shut up a focal list feasibility test up
		return make_tuple(true, next_threshold);
	}

	// Expand the node
	HL_num_expanded++;
	curr->time_expanded = HL_num_expanded - HL_num_expanded_before_this_iteration;

    if (spdlog::default_logger()->level() <= spdlog::level::info)
	{
        ostringstream msg;
		msg << std::endl << "****** Expanding #" << curr->time_generated
				  << " with f= " << curr->g_val <<	 "+" << curr->h_val << " (";
		for (int i = 0; i < num_of_agents; i++)
			msg << paths[i]->size() - 1 << ", ";
		msg << ")";
		spdlog::info(msg.str());
        spdlog::info("Chosen conflict: {}", *curr->conflict);
	}

	const auto& [agent1_id, agent2_id, location1, location2, timestep] = *curr->conflict;

	std::shared_ptr<Conflict> orig_conflict = curr->conflict;  // TODO: Consider not storing the chosen conflict as a state of the node
	int orig_agent_id = curr->agent_id;
	int orig_makespan = curr->makespan;
	int orig_g_val = curr->g_val;
	int orig_h_val = curr->h_val;
	int orig_num_conflicts = curr->num_of_conflicts;
	int orig_time_generated = curr->time_generated;
	int orig_time_expanded = curr->time_expanded;
	ICBSNode::WillCostIncrease orig_left_cost_will_increase = curr->left_cost_will_increase;
	ICBSNode::WillCostIncrease orig_right_cost_will_increase = curr->right_cost_will_increase;
	ICBSNode::WillCostIncrease orig_left_f_will_increase = curr->left_f_will_increase;
	ICBSNode::WillCostIncrease orig_right_f_will_increase = curr->right_f_will_increase;
	Path path_backup = *paths[agent1_id];

	curr->agent_id = agent1_id;
	auto [replan1_success, constraint1_added] = idcbsh_add_constraint_and_replan(
			curr, cat, next_threshold - curr->g_val - 1);  // If the cost will be larger than that, don't bother generating the node
															           // - it won't even update next_threshold
	int delta_g = curr->g_val - orig_g_val;
    if (delta_g == 0)
        ++num_delta_g_0;
    else if (delta_g == 1)
        ++num_delta_g_1;
    else {
        sum_delta_g_2_or_more += delta_g;
        ++num_delta_g_2_or_more;
    }
	// Subtract the delta_g from h, just to be nice:
	if (curr->h_val >= delta_g)
		curr->h_val -= delta_g;
	else
		curr->h_val = 0;
	curr->f_val = curr->g_val + curr->h_val;

	if (replan1_success) {
		HL_num_generated++;
        if (spdlog::default_logger()->level() < spdlog::level::warn) {
			// check the solution
			arePathsConsistentWithConstraints(paths, curr);
		}
	}

	if (replan1_success && curr->f_val <= threshold * focal_w)
	{
		curr->time_generated = HL_num_generated - HL_num_generated_before_this_iteration;
		// Find the left child's conflicts:
		clearConflictsOfAgent(*curr, curr->agent_id);
		auto hl_node_verification_start = std::clock();
		auto wall_hl_node_verification_start = std::chrono::system_clock::now();
		for (int a2 = 0; a2 < num_of_agents; a2++) {
		    if (a2 != curr->agent_id) {
		        findConflicts(*curr, curr->agent_id, a2);
		    }
		}
		hl_node_verification_runtime += std::clock() - hl_node_verification_start;
		wall_hl_node_verification_runtime += std::chrono::system_clock::now() - wall_hl_node_verification_start;
		curr->count_conflicts();
		sum_num_conflicts += curr->num_of_conflicts;
		++num_num_conflicts;

		spdlog::info("Generated left child #{} with f= {}+{} and {} conflicts {}",
		             curr->time_generated, curr->g_val, curr->h_val, curr->num_of_conflicts);

		bool bypass = false;
		if ((curr->g_val == orig_g_val) && (curr->num_of_conflicts < orig_num_conflicts)) {
            if (screen) {
                std::cout << "Left child found a bypass - adopting its new paths without splitting." << std::endl;
            }
            idcbsh_unconstrain(curr, cat, path_backup, orig_conflict, orig_makespan, orig_g_val, orig_h_val, orig_left_cost_will_increase, orig_right_cost_will_increase, true);
            bypass = true;
            curr->time_generated = orig_time_generated;  // Simulate adopting the child's path and continuing with the parent
		}

		// Recurse!
		curr->conflict = nullptr;  // Trigger computation of h in the recursive call
		auto [success, lowest_avoided_f_val] = do_idcbsh_iteration(curr, cat, threshold, next_threshold, end_by, bypass);
		next_threshold = min(next_threshold, lowest_avoided_f_val);  // lowest_avoided_f_val might actually be just the
																	 // next_threshold we provided

		if (std::clock() > end_by) {  // timeout (no need to unconstrain)
			spdlog::warn("Timed out in idcbsh iteration!");
			return make_tuple(false, next_threshold);
		}

		curr->time_generated = orig_time_generated;

		if (success) {
			curr->agent_id = orig_agent_id;  // Just to be clean
			// Notice we're keeping the paths and the constraints of the solution
			return make_tuple(true, next_threshold);
		} else {
			if (bypass) {  // Don't generate the right child, and don't unconstrain (we already did)
        	    return make_tuple(false, next_threshold);
        	}
			curr->agent_id = agent1_id;  // The recursive call may have changed it, and it needs to be restored before unconstrain is called
		}
	} else {
		// A path was not found for the constrained agent in the left child, or the overall g was higher than the threshold
		if (curr->f_val > threshold * focal_w)  // The overall f was higher than the threshold
			next_threshold = min(next_threshold, curr->f_val);

        if (!constraint1_added)
            spdlog::info("The left child's cost would have been larger than the NEXT threshold {}", next_threshold);
        else if (curr->f_val <= threshold * focal_w)
            spdlog::info("No solution for the left child!");
        else
            spdlog::info("The left child's F value {} > {} threshold * {} focal_w.", curr->f_val, threshold, focal_w);
	}

	// A goal was not found with a cost below the threshold in the left child
	if (constraint1_added)
		idcbsh_unconstrain(curr, cat, path_backup, orig_conflict, orig_makespan, orig_g_val, orig_h_val, orig_left_cost_will_increase, orig_right_cost_will_increase);

	spdlog::info("Moving to the right child of #{}", curr->time_generated);

	curr->agent_id = agent2_id;
	path_backup = *paths[agent2_id];
	auto [replan2_success, constraint2_added] = idcbsh_add_constraint_and_replan(
			curr, cat, next_threshold - curr->g_val - 1);  // If the cost will be larger than that, don't bother generating the node
															              // - it won't even update next_threshold
	delta_g = curr->g_val - orig_g_val;
	// Subtract the delta_g from h, just to be nice:
	if (curr->h_val >= delta_g)
		curr->h_val -= delta_g;
	else
		curr->h_val = 0;
	curr->f_val = curr->g_val + curr->h_val;

	if (replan2_success) {
		HL_num_generated++;
        if (spdlog::default_logger()->level() < spdlog::level::warn) {
			// check the solution
			arePathsConsistentWithConstraints(paths, curr);
		}
	}

	if (replan2_success && curr->f_val <= threshold * focal_w)
	{
		curr->time_generated = HL_num_generated - HL_num_generated_before_this_iteration;
		// Find the node's conflicts:
		clearConflictsOfAgent(*curr, curr->agent_id);
		for (int a2 = 0; a2 < num_of_agents; a2++) {
			if (a2 != curr->agent_id) {
                auto hl_node_verification_start = std::clock();
                auto wall_hl_node_verification_start = std::chrono::system_clock::now();
                findConflicts(*curr, curr->agent_id, a2);
                hl_node_verification_runtime += std::clock() - hl_node_verification_start;
                wall_hl_node_verification_runtime += std::chrono::system_clock::now() - wall_hl_node_verification_start;
            }
		}
		curr->count_conflicts();
        sum_num_conflicts += curr->num_of_conflicts;
        ++num_num_conflicts;

		spdlog::info("Generated right child #{} with f= {}+{} and {} conflicts", curr->time_generated, curr->g_val,
		              curr->h_val, curr->num_of_conflicts);

		// No need to check for a bypass here - we already went down the left child, removing the last constraint won't help

		curr->conflict = nullptr;  // Trigger computation of h in the recursive call
		// Recurse!
		auto [success, lowest_avoided_f_val] = do_idcbsh_iteration(curr, cat, threshold, next_threshold, end_by);
		next_threshold = min(next_threshold, lowest_avoided_f_val);

		curr->time_generated = orig_time_generated;

		if (success) {
			curr->agent_id = orig_agent_id;  // Just to be clean
			// Notice we're keeping the paths and the constraints of the solution
			return make_tuple(true, next_threshold);
		} else {
		}
	} else {
		// A path was not found for the constrained agent in the left child, or the overall g was higher than the threshold
        if (!constraint2_added)
            spdlog::info("The right child's cost would have been larger than the NEXT threshold {}", next_threshold);
        else if (curr->f_val <= threshold * focal_w)
            spdlog::info("No solution for the right child!");
        else
            spdlog::info("The right child's F value {} > {} threshold * {} focal_w", curr->f_val, threshold, focal_w);
	}
	curr->agent_id = agent2_id;  // The recursive call may have changed it, and it needs to be restored before unconstrain is called
	if (constraint2_added)
		idcbsh_unconstrain(curr, cat, path_backup, orig_conflict, orig_makespan, orig_g_val, orig_h_val, orig_left_cost_will_increase, orig_right_cost_will_increase);
	curr->agent_id = orig_agent_id;  // Just to be clean
	spdlog::info("Finished exploring the subtree under #{} with f<={} * {} without finding a goal node",
	             curr->time_generated, threshold, focal_w);
	return make_tuple(false, next_threshold);
}

// Returns (replan_success, constraint_added)
tuple<bool, bool> ICBSSearch::idcbsh_add_constraint_and_replan(ICBSNode *node,
															   ConflictAvoidanceTable &cat,
															   int allowed_cost_increase)
{
	vector<Path *> &paths = *node->all_paths;
	const auto& [agent1_id, agent2_id, location1, location2, timestep] = *node->conflict;
	int oldG = node->g_val;
	ICBSNode::WillCostIncrease costMayIncrease = node->agent_id == agent1_id ? node->left_cost_will_increase : node->right_cost_will_increase;

	int minNewCost;
	if (timestep >= (int)paths[node->agent_id]->size())  // Conflict happens after the agent reaches its goal.
		// Since there can only be vertex conflicts when the agent is WAITing
		// at its goal, the goal would only be reachable after the time of the new constraint
		minNewCost = timestep + 1;
	else if ((*paths[node->agent_id])[timestep].builtMDD == false)  // Can't check if we built an MDD for the agent
		// we only know the cost won't decrease
		minNewCost = (int)paths[node->agent_id]->size() - 1;
	else if ((*paths[node->agent_id])[timestep].single == false) {  // Not a singleton in the agent's MDD -
		// we only know the cost won't decrease
		minNewCost = (int) paths[node->agent_id]->size() - 1;
		// TODO: Consider setting maxNewCost = minNewCost and passing it to the low level
	}
	else if (location1 >= 0 && location2 < 0)  // It's a vertex constraint on a singleton in the agent's MDD -
											   // cardinal vertex - cost will increase by at least 1
		minNewCost = (int)paths[node->agent_id]->size();
	else if (paths[node->agent_id]->at(timestep - 1).builtMDD && paths[node->agent_id]->at(timestep - 1).single) // Edge constraint on an edge between two singletons in the agent's MDD
		minNewCost = (int)paths[node->agent_id]->size();
	else // Not a cardinal edge
		minNewCost = (int)paths[node->agent_id]->size() - 1;

	if (minNewCost - ((int)paths[node->agent_id]->size() - 1) > allowed_cost_increase)  // This check is instead of partial expansion
		return make_tuple(false, false);

	// The conflict-avoidance table already has the paths of all the agents
	removePathFromConflictAvoidanceTable(*paths[node->agent_id], cat, node->agent_id);

	// Constrain and replan
	if (location2 < 0 || node->agent_id == agent1_id)
		LL_num_generated += node->add_constraint(make_tuple(location1, location2, timestep, false), &cat, true, posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem);
	else
		LL_num_generated += node->add_constraint(make_tuple(location2, location1, timestep, false), &cat, true, posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem);

	bool replan_success = findPathForSingleAgent(node, &cat, timestep, minNewCost, node->agent_id, true);

	// Restore the conflict-avoidance table to have the paths of all the agents
	addPathToConflictAvoidanceTable(*paths[node->agent_id], cat, node->agent_id);

	if (replan_success && (costMayIncrease == ICBSNode::WillCostIncrease::NO) && (node->g_val > oldG))
	{
		spdlog::error("Cost increased for non-cardinal agent {}!?!?!?!", node->agent_id);
		std::abort();
	}
	if (replan_success && (costMayIncrease == ICBSNode::WillCostIncrease::YES) && (node->g_val == oldG))
	{
		spdlog::error("Cost did not increase for cardinal agent {}!?!?!?!", node->agent_id);
		std::abort();
	}

	return make_tuple(replan_success, true);
}

void ICBSSearch::idcbsh_unconstrain(ICBSNode *node, ConflictAvoidanceTable &cat,
									Path &path_backup, shared_ptr<Conflict> &conflict_backup,
									int makespan_backup, int g_val_backup, int h_val_backup,
                                    ICBSNode::WillCostIncrease left_cost_will_increase_backup,
                                    ICBSNode::WillCostIncrease right_cost_will_increase_backup,
									bool just_unconstrain)
{
	vector<Path *> &paths = *node->all_paths;
	// Remove the last constraint on the agent
	int agent_id = node->agent_id;
	ConflictAvoidanceTable* catp = nullptr;
#ifndef LPA
#else
	// The conflict-avoidance table already has the paths of all the agents
	removePathFromConflictAvoidanceTable(*paths[agent_id], cat, agent_id);
	catp = &cat;
#endif
	LL_num_generated += node->pop_constraint(catp);
#ifndef LPA
#else
	// Restore the conflict-avoidance table to have the paths of all the agents
	// Note we haven't asked LPA* to re-find the path so we're restoring the path from backup
	if (!just_unconstrain)
	    addPathToConflictAvoidanceTable(path_backup, cat, agent_id);
	else
	    addPathToConflictAvoidanceTable(*paths[agent_id], cat, agent_id);  // Keep the new path
#endif

	if (just_unconstrain)
	    return;

	// No need to ask LPA* to find the path again just to restore it, we saved a backup!
	paths[agent_id]->resize(path_backup.size());
	for (int i = 0 ; i < path_backup.size() ; ++i)
	{
		(*paths[agent_id])[i] = path_backup[i];
	}

	// Find the conflicts again, we didn't save a backup of them.
	// TODO: Consider doing that too.
	clearConflictsOfAgent(*node, agent_id);
	auto hl_node_verification_start = std::clock();
	auto wall_hl_node_verification_start = std::chrono::system_clock::now();
	for (int a2 = 0; a2 < num_of_agents; a2++) {
	    if (a2 != agent_id) {
	        findConflicts(*node, agent_id, a2);
	    }
	}
	hl_node_verification_runtime += std::clock() - hl_node_verification_start;
	wall_hl_node_verification_runtime += std::chrono::system_clock::now() - wall_hl_node_verification_start;
	node->count_conflicts();
    // No need to update the num_conflicts statistics
	classifyConflicts(*node); // classify and choose conflicts
								 // (the choice isn't guaranteed to be stable nor even deterministic,
								 // so I'm not assigning the result, I'm using a backup:

	// Restore the node's original values:
	node->conflict = conflict_backup;
	node->makespan = makespan_backup;
	node->g_val = g_val_backup;
	node->h_val = h_val_backup;
	node->left_cost_will_increase = left_cost_will_increase_backup;
	node->right_cost_will_increase = right_cost_will_increase_backup;
	node->f_val = node->g_val + node->h_val;
	// Note it's safe not to do that after a bypass is found: the g, h, makespan and f did not change,
	// the conflict is going to be set to nullptr anyway before the recursive call, and left_cost_will_increase and
	// right_cost_will_increase will be overridden in the recursive call when the next conflict is chosen (or
	// the goal will is found)
}

ICBSSearch::ICBSSearch(const MapLoader &ml, const AgentsLoader &al, double focal_w, split_strategy p,
                       highlevel_heuristic HL_h,
                       int cutoffTime, int child_pref_budget, int max_child_pref_options,
                       bool propagatePositiveCons, bool preferFCardinals, bool preferGoalConflicts) :
	focal_w(focal_w), split(p), HL_heuristic(HL_h), solution_node(nullptr),
	num_of_agents(al.num_of_agents),
	time_limit(cutoffTime),
	num_map_cols(ml.cols),
	map_size(ml.rows * ml.cols),
	moves_offset(ml.moves_offset),
	search_engines(num_of_agents, nullptr),
	paths(num_of_agents, nullptr),
	child_pref_budget(child_pref_budget), max_child_pref_options(max_child_pref_options),
	root_cat(ml.moves_offset, ml.map_size()),
	posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem(propagatePositiveCons),
	preferFCardinals(preferFCardinals),
	preferGoalConflicts(preferGoalConflicts)
#ifdef USE_GUROBI
    , mvc_model(gurobi_env)
#endif
{
	// set timer
	std::clock_t prep_start = std::clock();
	auto wall_prep_start = std::chrono::system_clock::now();

#ifdef USE_GUROBI
    try {
        gurobi_env.set(GRB_DoubleParam_TimeLimit,
                       ((double) cutoffTime) / 2);  // Set each optimize call's timeout to half the global timeout
        gurobi_env.set(GRB_IntParam_OutputFlag, 0);  // Turn off logging
        gurobi_env.set(GRB_IntParam_Threads,
                       1);  // Use a single thread - we have generally simple models, and there's an overhead to running extra threads
        // It's also a more fair comparison with older algorithms
        //gurobi_env.set(GRB_IntParam_MIPFocus, 0);

        //TEMP!@#
        mvc_model.set(GRB_DoubleParam_TimeLimit,
                      ((double) cutoffTime) / 2);  // Set each optimize call's timeout to half the global timeout
        mvc_model.set(GRB_IntParam_OutputFlag, 0);  // Turn off logging
        mvc_model.set(GRB_IntParam_Threads,
                      1);  // Use a single thread - we have generally simple models, and there's an overhead to running extra threads
                               // It's also a more fair comparison with older algorithms
//    mvc_model.set(GRB_DoubleParam_Heuristics, 0);  // Multiple grbtune results on 100s of saved models found that this is a good setting usually.
//    mvc_model.read("tune3.prm");
        vector<double> lbs(num_of_agents, 0.0);
        vector<double> obj_coeffs(num_of_agents,
                                  1);  // The default is that the model minimizes the objective function, as we need
        double *ubs_data = nullptr;
#ifndef GUROBI_WITH_GOAL_CONSTRAINTS
        vector<char> types(num_of_agents, GRB_BINARY);
        vector<double> ubs(num_of_agents, 1.0);
        ubs_data = ubs.data();
#else
        vector<char> types(num_of_agents, GRB_INTEGER);
#endif
        mvc_vars = mvc_model.addVars(lbs.data(), ubs_data, obj_coeffs.data(), types.data(), nullptr, num_of_agents);
    }
    catch (const GRBException& exception) {
        spdlog::critical(exception.getMessage());
        throw;
    }
#endif

#ifndef LPA
#else
	vector<LPAStar*> lpas(num_of_agents);
#endif

	// initialize single agent search solver and heuristics for the low-level search
	for (int i = 0; i < num_of_agents; i++)
	{
		int init_loc = ml.linearize_coordinate((al.initial_locations[i]).first, (al.initial_locations[i]).second);
		int goal_loc = ml.linearize_coordinate((al.goal_locations[i]).first, (al.goal_locations[i]).second);
		ostringstream htable_filename_stream;
		htable_filename_stream << ml.name << "_" << goal_loc << ".htable";
		string htable_filename = htable_filename_stream.str();
		auto htable = new int[ml.rows * ml.cols];
		if (std::filesystem::exists(htable_filename)) {
            FILE* htable_file = fopen(htable_filename.c_str(), "rb");
            int ret = fread(htable, sizeof(htable[0]), ml.rows * ml.cols, htable_file);
            fclose(htable_file);
            if (ret != ml.rows * ml.cols) {
                spdlog::critical("{}'s contents are too short!", htable_filename);
                abort();
            }
		}
		else {
            HeuristicCalculator heuristicCalculator(init_loc, goal_loc, ml.my_map, ml.rows, ml.cols, ml.moves_offset);
            heuristicCalculator.computeHVals(htable);
            auto temp_filename = new char[htable_filename.size() + 7];
            memcpy(temp_filename, htable_filename.c_str(), htable_filename.size());
            memcpy(temp_filename + htable_filename.size(), "XXXXXX", 7);
            int temp_fd = mkstemp(temp_filename);
            FILE* temp_file = fdopen(temp_fd, "w");
            fwrite(htable, sizeof(htable[0]), ml.rows * ml.cols, temp_file);
            fclose(temp_file);
            close(temp_fd);
            std::filesystem::rename(temp_filename, htable_filename);
            delete [] temp_filename;
		}
		search_engines[i] = new ICBSSingleAgentLLSearch(init_loc, goal_loc, ml.my_map, ml.rows, ml.cols, ml.moves_offset, htable);

#ifndef LPA
#else
		float* my_heuristic = new float[ml.rows * ml.cols];
		for (int j = 0; j < ml.rows * ml.cols; ++j) {
			my_heuristic[j] = search_engines[i]->my_heuristic[j];
		}
		lpas[i] = new LPAStar(init_loc, goal_loc, my_heuristic, &ml, i);
#endif
	}
	if (split != split_strategy::NON_DISJOINT) // Disjoint splitting uses differential heuristics
											   // for the low-level search in addition to the perfect heuristic
	{
		for (int i = 0; i < num_of_agents; i++)
		{
			search_engines[i]->differential_h.resize(num_of_agents);
			for (int j = 0; j < num_of_agents; j++)
				search_engines[i]->differential_h[j] = search_engines[j]->my_heuristic;
		}
	}

	prepTime = std::clock() - prep_start;
	wall_prepTime = std::chrono::system_clock::now() - wall_prep_start;

	start = std::clock();
	auto wall_start = std::chrono::system_clock::now();

	// initialize paths_found_initially
#ifndef LPA
	root_node = new ICBSNode(num_of_agents);
#else
	root_node = new ICBSNode(lpas);
#endif
	paths_found_initially.resize(num_of_agents);
	vector<Path*> paths(num_of_agents, nullptr);
	populatePaths(root_node, paths);
	root_node->all_paths = &paths;

	std::vector < std::unordered_map<int, ConstraintState > > cons_table;
	for (int i = 0; i < num_of_agents; i++)
	{
		if (i > 0)
			addPathToConflictAvoidanceTable(*(*root_node->all_paths)[i-1], root_cat, i-1);

		clock_t ll_start = std::clock();
		auto wall_ll_start = std::chrono::system_clock::now();
#ifndef LPA
		pair<int, int> start_location_and_time(search_engines[i]->start_location, 0);
		pair<int, int> goal_location_and_time(search_engines[i]->goal_location, numeric_limits<int>::max());
		bool success = search_engines[i]->findShortestPath(paths_found_initially[i], cons_table, root_cat, start_location_and_time, goal_location_and_time, 0, -1);
		LL_num_expanded += search_engines[i]->num_expanded;
		LL_num_generated += search_engines[i]->num_generated;
#else
		spdlog::debug("Calling LPA* for the first time for agent {}", i);
		auto generated_before = root_node->lpas[i]->allNodes_table.size();
		bool success = root_node->lpas[i]->findPath(root_cat, -1, -1, start + time_limit * CLOCKS_PER_SEC);
		LL_num_expanded += root_node->lpas[i]->num_expanded;
		//LL_num_expanded += node->lpas[ag]->num_expandeds[node->lpas[ag]->paths.size() - 1];
		LL_num_generated += root_node->lpas[i]->allNodes_table.size() - generated_before;
#endif
		lowLevelTime += std::clock() - ll_start;
		wall_lowLevelTime += std::chrono::system_clock::now() - wall_ll_start;
		if (!success)
		{
			spdlog::error("NO solution exists for agent {} of the root node", i);
			delete root_node;
			exit(-1);
		}
#ifndef LPA
#else
		const vector<int>* primitive_path = root_node->lpas[i]->getPath();  // 1 - first iteration path (0 is an empty path)
		paths_found_initially[i].resize(primitive_path->size());
		for (int j = 0; j < primitive_path->size(); ++j) {
			paths_found_initially[i][j].location = (*primitive_path)[j];
			paths_found_initially[i][j].builtMDD = false;
		}
		paths_found_initially[i][0].builtMDD = true;
		paths_found_initially[i][0].single = true;
		paths_found_initially[i][0].numMDDNodes = 1;
		paths_found_initially[i][primitive_path->size()-1].builtMDD = true;
		paths_found_initially[i][primitive_path->size()-1].single = true;
		paths_found_initially[i][primitive_path->size()-1].numMDDNodes = 1;
#endif
		root_node->makespan = max(root_node->makespan, (int)paths_found_initially[i].size() - 1);
	}

	// generate the root node and update data structures
	root_node->agent_id = -1;
	root_node->g_val = 0;
	for (int i = 0; i < num_of_agents; i++)
		root_node->g_val += (int) paths_found_initially[i].size() - 1;
	root_node->h_val = 0;
	root_node->f_val = root_node->g_val;
	root_node->depth = 0;
	root_node->open_handle = open_list.push(root_node);
	root_node->in_open = true;
	root_node->focal_handle = focal_list.push(root_node);
	root_node->in_focal = true;
	HL_num_generated++;
	root_node->time_generated = HL_num_generated;
	allNodes_table.push_back(root_node);
	findConflicts(*root_node);
	root_node->count_conflicts();
    sum_num_conflicts += root_node->num_of_conflicts;
    ++num_num_conflicts;
	min_f_val = root_node->f_val;
	focal_list_threshold = min_f_val * focal_w;

	root_node->all_paths = nullptr;

	runtime = std::clock() - start;
	wall_runtime = std::chrono::system_clock::now() - wall_start;
}

ICBSSearch::~ICBSSearch()
{
	delete [] mvc_vars;

	for (auto& search_engine : search_engines)
		delete search_engine;

	for (auto& node : allNodes_table) {
		// TODO: free all the lpastar instances from all the nodes, but carefully, since they're shared
		delete node;
	}
}
