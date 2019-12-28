#include "ICBSSearch.h"
#include <filesystem>  // For exists

//////////////////// HIGH LEVEL HEURISTICS ///////////////////////////
// compute heuristics for the high-level search
int ICBSSearch::computeHeuristics(const ICBSNode& curr)
{
	// build conflict graph
	vector<vector<bool>> CG(num_of_agents);
	int num_of_CGnodes = 0, num_of_CGedges = 0;
	for (int i = 0; i < num_of_agents; i++)
		CG[i].resize(num_of_agents, false);
	for (list<std::shared_ptr<Conflict>>::const_iterator it = curr.cardinalConf.begin(); it != curr.cardinalConf.end(); ++it)
	{
		auto [agent1, agent2, loc1, loc2, timestep] = **it;
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
		for (auto con : curr->positive_constraints[curr->agent_id])
		{
			constraints.push_back(make_pair(curr->agent_id, con));
		}
		for (auto con : curr->negative_constraints[curr->agent_id])
		{
			constraints.push_back(make_pair(curr->agent_id, con));
		}
		curr = curr->parent;
	}
	// TODO: Use this function.
}

// build the constraint table for replanning agent <agent_id>,
// and find the two closest landmarks for agent <agent_id> that have the new constraint at time step <timestep> between them.
// TODO: Consider splitting into two functions
// update cons_table: cons_table[time_step][location].vertex or .edge = true iff they're a negative constraint for the agent
// update start and goal: the two landmarks for the agent such that its path between them violates the newly posted constraint (at timestep)
// return last goal constraint timestep - the time of the last constraint on an agent's goal
int ICBSSearch::buildConstraintTable(ICBSNode* curr, int agent_id, int timestep,
	std::vector < std::unordered_map<int, ConstraintState > >& cons_table, 
	pair<int,int>& start, pair<int,int>& goal)
{
	int lastGoalConsTimestep = -1;

	// extract all constraints on agent_id
	list < Constraint > constraints_positive;  
	list < Constraint > constraints_negative;
	while (curr != nullptr)
	{
		for (auto con: curr->negative_constraints[agent_id]) {
			auto [loc1, loc2, constraint_timestep, positive_constraint] = con;
			constraints_negative.push_back(con);
			if (loc1 == goal.first && loc2 < 0 && lastGoalConsTimestep < constraint_timestep)
				lastGoalConsTimestep = constraint_timestep;
		}

		for (int j = 0; j < num_of_agents ; ++j) {
			if (curr->agent_id == agent_id) // for the constrained agent, those are landmarks
			{
				for (auto con: curr->positive_constraints[agent_id]) {
					auto[loc1, loc2, constraint_timestep, positive_constraint] = con;

					if (loc2 < 0) // vertex constraint
					{
						if (start.second < constraint_timestep && constraint_timestep < timestep)  // This landmark is between (start.second, timestep)
						{ // update start
							start.first = loc1;
							start.second = constraint_timestep;
						} else if (timestep <= constraint_timestep && constraint_timestep < goal.second)  // the landmark is between [timestep, goal.second)
						{ // update goal
							goal.first = loc1;
							goal.second = constraint_timestep;
							// If the landmark is at the same timestep as the new constraint, planning the new path will surely fail
						}
					} else // edge constraint, viewed as two landmarks on the from-vertex and the to-vertex
					{
						if (start.second < constraint_timestep && constraint_timestep < timestep)  // the second landmark is between (start.second, timestep)
						{ // update start
							start.first = loc2;
							start.second = constraint_timestep;
						} else if (timestep <= constraint_timestep - 1 && constraint_timestep - 1 < goal.second)  // the first landmark is between [timestep, goal.second)
						{ // update goal
							goal.first = loc1;
							goal.second = constraint_timestep - 1;
						}
					}
					constraints_positive.push_back(con);
				}
			}
			else {  // for the other agents, it is equivalent to a negative constraint
				for (auto con: curr->positive_constraints[agent_id]) {
					auto [loc1, loc2, constraint_timestep, positive_constraint] = con;
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
	for (list< Constraint >::iterator it = constraints_negative.begin(); it != constraints_negative.end(); it++) 
	{
		auto [loc1, loc2, constraint_timestep, positive_constraint] = *it;
		if (!positive_constraint)
		{
			if (loc2 < 0) // vertex constraint
				cons_table[constraint_timestep][loc1].vertex = true;
			else // edge constraint
			{
				for(int i = 0; i < MapLoader::valid_moves_t::WAIT_MOVE; i++)
				{
					if (loc2 - loc1 == moves_offset[i])
					{
						cons_table[constraint_timestep][loc2].edge[i] = true;
					}
				}
			}
		}
		else if (loc2 < 0)  // positive vertex constraint for other agent
			cons_table[constraint_timestep][loc1].vertex = true;
		else  // positive edge constraint for other agent
		{
			cons_table[constraint_timestep - 1][loc1].vertex = true;
			cons_table[constraint_timestep][loc2].vertex = true;
			for (int i = 0; i < MapLoader::valid_moves_t::WAIT_MOVE; i++)
			{
				if (loc1 - loc2 == moves_offset[i])
					cons_table[constraint_timestep][loc1].edge[i] = true;
			}
		}
			
	}
	return lastGoalConsTimestep;
}

// build conflict avoidance table
// update cat: Set cat[time_step][location].vertex or .edge[direction] to the number of other agents that plan to use it
void ICBSSearch::buildConflictAvoidanceTable(vector<vector<PathEntry> *> &the_paths, int exclude_agent, const ICBSNode &node,
                                             std::vector<std::unordered_map<int, AvoidanceState> > &cat)
{
	if (node.makespan == 0)
		return;
	// TODO: Consider removing this optimisation and the node parameter
	for (int ag = 0; ag < num_of_agents; ag++)
	{
		if (ag != exclude_agent &&
		    the_paths[ag] != NULL  // Happens when computing the initial paths for the root node
		    )
		{
			addPathToConflictAvoidanceTable(the_paths[ag], cat);
		}
	}
}

// add a path to cat
void ICBSSearch::addPathToConflictAvoidanceTable(vector<PathEntry> *path,
                                                 std::vector<std::unordered_map<int, AvoidanceState> > &cat)
{
	//if (cat[0][path->at(0).location].vertex < 255)
	//	cat[0][path->at(0).location].vertex++;
	// Don't bother with timestep 0 - no conflicts will occur there

	for (size_t timestep = 1; timestep < cat.size(); timestep++)
	{
		if (timestep >= path->size()) {
			AvoidanceState& entry = cat[timestep][path->back().location];
			if (entry.vertex < 255)
				entry.vertex++;
		}
		else
		{
			int to = path->at(timestep).location;
			int from = path->at(timestep - 1).location;
			AvoidanceState& to_entry = cat[timestep][to];
			AvoidanceState& from_entry = cat[timestep][from];
			if (to_entry.vertex < 255)
				to_entry.vertex++;
			for (int i = 0; i < MapLoader::valid_moves_t::WAIT_MOVE; i++)
			{
				if (from - to == moves_offset[i]) {
					if (from_entry.edge[i] < 255)
						from_entry.edge[i]++;
					break;
				}
			}
			// TODO: Have a small table mapping from move offsets to their index and use it instead of iterating
		}
	}
}

// add a path to cat
void ICBSSearch::removePathFromConflictAvoidanceTable(vector<PathEntry> *path,
                                                      std::vector<std::unordered_map<int, AvoidanceState> > &cat)
{
    //if (cat[0][path->at(0).location].vertex < 255)
    //	cat[0][path->at(0).location].vertex++;
    // Don't bother with timestep 0 - no conflicts will occur there

    for (size_t timestep = 1; timestep < cat.size(); timestep++)
    {
        if (timestep >= path->size()) {
            AvoidanceState& entry = cat[timestep][path->back().location];
            if (entry.vertex > 0)
                entry.vertex--;
        }
        else
        {
            int to = path->at(timestep).location;
            int from = path->at(timestep - 1).location;
            AvoidanceState& to_entry = cat[timestep][to];
            AvoidanceState& from_entry = cat[timestep][from];
            if (to_entry.vertex > 0)
                to_entry.vertex--;
            for (int i = 0; i < MapLoader::valid_moves_t::WAIT_MOVE; i++)
            {
                if (from - to == moves_offset[i]) {
                    if (from_entry.edge[i] > 0)
                        from_entry.edge[i]--;
                    break;
                }
            }
            // TODO: Have a small table mapping from move offsets to their index and use it instead of iterating
        }
    }
}

//////////////////// CONFLICTS ///////////////////////////
// copy conflicts from the parent node if the paths of both agents in the conflict remain unchanged from the parent
void ICBSSearch::copyConflictsFromParent(ICBSNode& curr)
{
	// Mark which paths remain unchanged from the parent node
	vector<bool> unchanged(num_of_agents, true);
	for (list<pair<int, vector<PathEntry>>>::const_iterator it = curr.new_paths.begin(); it != curr.new_paths.end(); it++)
		unchanged[it->first] = false;
	// Copy conflicts of agents both whose paths remain unchanged
	copyConflicts(unchanged, curr.parent->cardinalConf, curr.cardinalConf);
	copyConflicts(unchanged, curr.parent->semiConf, curr.semiConf);
	copyConflicts(unchanged, curr.parent->nonConf, curr.nonConf);
	copyConflicts(unchanged, curr.parent->unknownConf, curr.unknownConf);
}

void ICBSSearch::clearConflictsOfAgent(ICBSNode &curr, int ag)
{
	// Mark which paths remain unchanged from the parent node
	bool unchanged[num_of_agents];
	for (int j = 0; j < num_of_agents; ++j) {
		unchanged[j] = true;
	}
	unchanged[ag] = false;

	// Keep conflicts of agents both whose paths remain unchanged
	clearConflictsOfAffectedAgents(unchanged, curr.cardinalConf);
	clearConflictsOfAffectedAgents(unchanged, curr.semiConf);
	clearConflictsOfAffectedAgents(unchanged, curr.nonConf);
	clearConflictsOfAffectedAgents(unchanged, curr.unknownConf);
}

// copy conflicts from "from" to "to" if the paths of both agents remain unchanged
// according to the given array
void ICBSSearch::copyConflicts(const vector<bool>& unchanged, 
	const list<std::shared_ptr<Conflict>>& from, list<std::shared_ptr<Conflict>>& to)
{
	for (const auto& conflict : from)
	{
		auto [agent1, agent2, loc1, loc2, timestep] = *conflict;
		if (unchanged[agent1] && unchanged[agent2])
			to.push_back(conflict);
	}
}

// Of changed agents
void ICBSSearch::clearConflictsOfAffectedAgents(bool *unchanged,
                                                list<std::shared_ptr<Conflict>> &lst)
{
	auto it = lst.begin();
	while (it != lst.end()) {
		auto [agent1, agent2, loc1, loc2, timestep] = **it;
		if (!unchanged[agent1] || !unchanged[agent2])
			it = lst.erase(it);
		else
			++it;

	}
}

// find all conflict in the current solution
void ICBSSearch::findConflicts(ICBSNode& curr)
{
	if (curr.parent != nullptr) // not root node
	{
		// detect conflicts that occur on the new planned paths
		vector<bool> detected(num_of_agents, false);
		for (list<pair<int, vector<PathEntry>>>::const_iterator it = curr.new_paths.begin(); it != curr.new_paths.end(); it++)
		{
			int a1 = it->first;
			detected[a1] = true;
			for (int a2 = 0; a2 < num_of_agents; a2++)
			{
				if (detected[a2])
					continue;
				findConflicts(paths, a1, a2, curr);
			}
		}
	}
	else // root node
	{ // detect conflicts among all paths
		for(int a1 = 0; a1 < num_of_agents ; a1++)
		{
			for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
			{
				findConflicts(paths, a1, a2, curr);
			}
		}
	}
}

// find conflicts between paths of agents a1 and a2
void ICBSSearch::findConflicts(vector<vector<PathEntry> *> &the_paths, int a1, int a2, ICBSNode &curr)
{
	size_t min_path_length = the_paths[a1]->size() < the_paths[a2]->size() ? the_paths[a1]->size() : the_paths[a2]->size();
	for (int timestep = 0; timestep < min_path_length; timestep++)
	{
		int loc1 = the_paths[a1]->at(timestep).location;
		int loc2 = the_paths[a2]->at(timestep).location;
		if (loc1 == loc2)
		{
			curr.unknownConf.push_back(std::shared_ptr<Conflict>(new Conflict(a1, a2, loc1, -1, timestep)));
		}
		else if (timestep < min_path_length - 1
			&& loc1 == the_paths[a2]->at(timestep + 1).location
			&& loc2 == the_paths[a1]->at(timestep + 1).location)
		{
			curr.unknownConf.push_back(std::shared_ptr<Conflict>(new Conflict(a1, a2, loc1, loc2, timestep + 1))); // edge conflict
		}
	}
	if (the_paths[a1]->size() != the_paths[a2]->size()) // check whether there are conflicts that occur after one agent reaches its goal
	{
		int a1_ = the_paths[a1]->size() < the_paths[a2]->size() ? a1 : a2;
		int a2_ = the_paths[a1]->size() < the_paths[a2]->size() ? a2 : a1;
		int loc1 = the_paths[a1_]->back().location;
		for (int timestep = (int)min_path_length; timestep < the_paths[a2_]->size(); timestep++)
		{
			int loc2 = the_paths[a2_]->at(timestep).location;
			if (loc1 == loc2)
			{ // It's at least a semi cardinal conflict
				curr.unknownConf.push_front(std::shared_ptr<Conflict>(new Conflict(a1_, a2_, loc1, -1, timestep))); 
			}
		}
	}
}

// Classify conflicts into cardinal, semi-cardinal and non-cardinal and populate the node's conflict lists with them
// Returns the highest priority conflict.
// If a CBS heuristic isn't used, return the first cardinal conflict that was found immediately, without finishing
// the classification of conflicts.
std::shared_ptr<Conflict> ICBSSearch::classifyConflicts(ICBSNode &node, vector<vector<PathEntry> *> &the_paths)
{
	if (node.cardinalConf.empty() && node.semiConf.empty() && node.nonConf.empty() && node.unknownConf.empty())
		return NULL; // No conflict

	// Classify all conflicts in unknownConf
	while (!node.unknownConf.empty())
	{
		std::shared_ptr<Conflict> con = node.unknownConf.front();
		auto [agent1, agent2, loc1, loc2, timestep] = *con;
		node.unknownConf.pop_front();

		bool cardinal1, cardinal2;
		if (loc2 >= 0) // Edge conflict
		{
			bool success = buildMDD(node, agent1, timestep, 0, cat);  // Build an MDD for the agent's current cost, unless we already know if it's narrow at <timestep>
			if (!success)
			    break;
			success = buildMDD(node, agent2, timestep, 0, cat);
            if (!success)
                break;
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
			if (timestep >= the_paths[agent1]->size())
				cardinal1 = true;
			else
			{
				bool success = buildMDD(node, agent1, timestep, 0, cat);
                if (!success)
                    break;
				cardinal1 = a1_path[timestep].single;
			}
			if (timestep >= the_paths[agent2]->size())
				cardinal2 = true;
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
			if (!HL_heuristic)  // Found a cardinal conflict and they're not used to complete heuristics. Return it immediately.
			{
				conflictType = conflict_type::CARDINAL;
				return con;
			}
			node.cardinalConf.push_back(con);
		}
		else if (cardinal1 || cardinal2)
		{
			node.semiConf.push_back(con);
		}
//		else if (cardinal1)
//		{
//			node.semiConf.push_back(reversed_conflict);  // Make sure the non-cardinal agent is first (remember to change the above elif to be only for cardinal2 !@#
//		}
		else
		{
			node.nonConf.push_back(con);
		}
	}

	return getHighestPriorityConflict(node, the_paths);
}

// Primary priority - cardinal conflicts, then semi-cardinal and non-cardinal conflicts
std::shared_ptr<Conflict> ICBSSearch::getHighestPriorityConflict(ICBSNode &node, vector<vector<PathEntry> *> &the_paths)
{
	vector<int> metric(num_of_agents, 0);
	vector<double> widthMDD(num_of_agents, 0);
	if (split == split_strategy::SINGLETONS)
	{
		for (int i = 0; i < num_of_agents; i++)
		{
			for (int j = 0; j< the_paths[i]->size(); j++)
				widthMDD[i] += the_paths[i]->at(j).numMDDNodes;
			widthMDD[i] /= the_paths[i]->size();
		}
	}
	if (!node.cardinalConf.empty())
	{
		conflictType = conflict_type::CARDINAL;
		return getHighestPriorityConflict(node.cardinalConf, the_paths, widthMDD, metric);
	}
	else if (!node.semiConf.empty())
	{
		conflictType = conflict_type::SEMICARDINAL;
		return getHighestPriorityConflict(node.semiConf, the_paths, widthMDD, metric);
	}
	else
	{
		conflictType = conflict_type::NONCARDINAL;
		return getHighestPriorityConflict(node.nonConf, the_paths, widthMDD, metric);
	}

}

// Secondary priority within a primary priority group
std::shared_ptr<Conflict> ICBSSearch::getHighestPriorityConflict(const list<std::shared_ptr<Conflict>> &confs,
                                                                 vector<vector<PathEntry> *> &the_paths,
                                                                 const vector<double> &widthMDD,
                                                                 const vector<int> &metric)
{
	std::shared_ptr<Conflict> choice = confs.front();

	if (split == split_strategy::SINGLETONS)
	{
		for (const auto& conf : confs)
		{
			auto [agent1, agent2, loc1, loc2, timestep] = *conf;
			auto [choice_agent1, choice_agent2, choice_loc1, choice_loc2, choice_timestep] = *choice;
			int metric1 = 0;
			int maxSingles = 0;
			double minWidth = 0;
			if (timestep < (int)the_paths[agent1]->size())
				for (int j = 0; j < timestep; j++)
					if (the_paths[agent1]->at(j).builtMDD && the_paths[agent1]->at(j).single)
						metric1++;
			int metric2 = 0;
			if (timestep < (int)the_paths[agent2]->size())
				for (int j = 0; j < timestep; j++)
					if (the_paths[agent2]->at(j).builtMDD && the_paths[agent2]->at(j).single)
						metric2++;
			if (max(metric1, metric2) > metric[0])
			{
				choice = conf;
				maxSingles = max(metric1, metric2);
				minWidth = min(widthMDD[agent1], widthMDD[agent2]);
			}
			else if (max(metric1, metric2) == maxSingles &&  min(widthMDD[agent1], widthMDD[agent2]) < minWidth)
			{
				choice = conf;
				minWidth = min(widthMDD[agent1], widthMDD[agent2]);
			}
		}
	}
	else if (split == split_strategy::WIDTH)
	{
		for (const auto& conf : confs)
		{
			auto [agent1, agent2, loc1, loc2, timestep] = *conf;
			auto [choice_agent1, choice_agent2, choice_loc1, choice_loc2, choice_timestep] = *choice;
			int w1, w2, w3, w4;
			if (the_paths[agent1]->size() <= timestep)
				w1 = 1;
			else
				w1 = the_paths[agent1]->at(timestep).numMDDNodes;
			if (the_paths[agent2]->size() <= timestep)
				w2 = 1;
			else
				w2 = the_paths[agent2]->at(timestep).numMDDNodes;
			if (the_paths[choice_agent1]->size() <= choice_timestep)
				w3 = 1;
			else
				w3 = the_paths[choice_agent1]->at(choice_timestep).numMDDNodes;
			if (the_paths[choice_agent2]->size() <= choice_timestep)
				w4 = 1;
			else
				w4 = the_paths[choice_agent2]->at(choice_timestep).numMDDNodes;
			if (w1 * w2 < w3 * w4)
				choice = conf;
			else if (w1 * w2 == w3 * w4)
			{
				int single1 = 0;
				int single2 = 0;
				int single3 = 0;
				int single4 = 0;
				if (timestep < (int)the_paths[agent1]->size())
					for (int j = 0; j < timestep; j++)
						if (the_paths[agent1]->at(j).builtMDD && the_paths[agent1]->at(j).single)
							single1++;
				if (timestep < (int)the_paths[agent2]->size())
					for (int j = 0; j < timestep; j++)
						if (the_paths[agent2]->at(j).builtMDD && the_paths[agent2]->at(j).single)
							single2++;
				if (choice_timestep < (int)the_paths[choice_agent1]->size())
					for (int j = 0; j < choice_timestep; j++)
						if (the_paths[choice_agent1]->at(j).builtMDD && the_paths[choice_agent1]->at(j).single)
							single1++;
				if (choice_timestep < (int)the_paths[choice_agent2]->size())
					for (int j = 0; j < choice_timestep; j++)
						if (the_paths[choice_agent2]->at(j).builtMDD && the_paths[choice_agent2]->at(j).single)
							single2++;
				if (max(single1, single2) > max(single3, single4))
					choice = conf;
			}
		}
	}
#if !defined(LPA) && !defined(NOLPA_LATEST_CONFLICT_WITHIN_CLASS)
	else // uniformly at random
	{
		int id = rand() % confs.size();
		int i = 0;
		for (list<std::shared_ptr<Conflict>>::const_iterator it = confs.begin(); it != confs.end(); ++it)
		{
			if (i == id)
				return *it;
			else
				i++;
		}
	}
#else
	else // Closer to the goal - better for LPA*
	     // Can't just choose the conflict with the latest timestep. A later conflict in a very very long path
	     //       might not be better than an earlier conflict in a short path.
	{
		int min_steps_from_goal = std::numeric_limits<int>::max();
		for (const auto& conf : confs)
		{
			auto [agent1, agent2, loc1, loc2, timestep] = *conf;
			int steps_from_goal = the_paths[agent1]->size() - 1 - timestep + the_paths[agent2]->size() - 1 - timestep;
			if (steps_from_goal < min_steps_from_goal) {
				choice = conf;
				min_steps_from_goal = steps_from_goal;
			}
		}
	}
#endif
	return choice;

	

}

// build MDDs if needed
// For every step of the path of the given agent, save whether the MDD at that step has a single node
void ICBSSearch::buildMDD(ICBSNode &curr, vector<vector<PathEntry> *> &the_paths, int ag, int timestep, int lookahead)
{
	if (the_paths[ag]->at(timestep).builtMDD)
		return;

	// Find the last node on this branch that computed a new path for this agent
	ICBSNode* node = &curr; // Back to where we get the path
	bool found = false;
	while (node->parent != NULL)
	{
		for (const auto& newPath : node->new_paths)
		{
			if (newPath.first == ag)
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

	// Build a constraint table with entries for each timestep up to the makespan at the last node that added a
	// constraint for the agent. There can't be constraints that occur later than that because each agent must plan a
	// path that at least lets every constraint have the opportunity to affect it.
	std::vector < std::unordered_map<int, ConstraintState > > cons_table(node->makespan + 1);
	pair<int, int> start = make_pair(search_engines[ag]->start_location, 0);
	pair<int, int> goal = make_pair(search_engines[ag]->goal_location, (int)the_paths[ag]->size() - 1);
	buildConstraintTable(node, ag, timestep, cons_table, start, goal);

	std::clock_t mdd_building_start = std::clock();
	auto wall_mddStart = std::chrono::system_clock::now();
	MDD mdd;
	bool success = mdd.buildMDD(cons_table, start_location_and_time, goal_location_and_time, lookahead, *search_engines[ag], start + time_limit * CLOCKS_PER_SEC);
	if (!success) {
        highLevelMddBuildingTime += std::clock() - mdd_building_start;
        wall_mddTime += std::chrono::system_clock::now() - wall_mddStart;
        return false;
    }

	for (int i = 0; i < mdd.levels.size(); i++)
	{
		the_paths[ag]->at(i + start.second).single = mdd.levels[i].size() == 1;
		the_paths[ag]->at(i + start.second).numMDDNodes = (int)mdd.levels[i].size();
		the_paths[ag]->at(i + start.second).builtMDD = true;
	}
    highLevelMddBuildingTime += std::clock() - mdd_building_start;
    wall_mddTime += std::chrono::system_clock::now() - wall_mddStart;

    return true;
}

int ICBSSearch::countMddSingletons(int agent_id, int conflict_timestep)
{
	int num_singletons = 0;
	if (conflict_timestep < (int)paths[agent_id]->size())
		for (int j = 0; j < conflict_timestep; j++)
		{
			if (paths[agent_id]->at(j).builtMDD && paths[agent_id]->at(j).single)
				num_singletons++;
		}
	return num_singletons;
}

int ICBSSearch::getTotalMddWidth(int agent_id)
{
	int width = 0;
	for (int j = 0; j < paths[agent_id]->size(); j++)
	{
		width += paths[agent_id]->at(j).numMDDNodes;
	}
	return width;
}

void ICBSSearch::addPositiveConstraintsOnNarrowLevelsLeadingToPositiveConstraint(int agent_id, int timestep,
		ICBSNode* n1, ICBSNode* n2, const std::vector < std::unordered_map<int, AvoidanceState > >* catp)
{
	if (posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem)
	// The MDD levels up to the positive constraint are a superset of the MDD levels of the MDD for reaching
	// the location of the positive constraint at the time of the positive constraint, so any 1-width level
	// among the levels up to that of the positive constraint is also a 1-width level in the MDD for the agent
	// to reach the positive constraint (we know the positive constraint is reachable so it can't be a
	// 0-width level in the smaller MDD). Every 1-width level's node can also be added as a positive constraint
	// - we must pass through it to reach the positive constraint we added.
	{
		for (int i = timestep; i > 0; i--)
		{
			if (!paths[agent_id]->at(i).builtMDD)  // No MDD for this level. When does this happen?
				break;
			else if (!paths[agent_id]->at(i).single)  // Not a 1-width MDD level.
				continue;
			else if (paths[agent_id]->at(i - 1).builtMDD && paths[agent_id]->at(i - 1).single) {  // Safe because i > 0
				// This level is narrow and the previous one too - add a positive edge constraint between their
				// nodes. It's preferable over two positive constraints for some reason.
				LL_num_generated += n1->add_constraint(
						make_tuple(paths[agent_id]->at(i - 1).location, paths[agent_id]->at(i).location, i, true), catp);
			}
			else if (i < timestep && !paths[agent_id]->at(i + 1).single) {
				// This level is narrow, and the *next* one isn't (so we didn't already add a positive edge
				// constraint between them) - add a positive vertex constraint for this level's node
				LL_num_generated += n1->add_constraint(
						make_tuple(paths[agent_id]->at(i).location, -1, i, true), catp);
			}
		}
	}
}


//////////////////// GENERATE A NODE ///////////////////////////
// add constraints to child nodes
void ICBSSearch::branch(ICBSNode* curr, ICBSNode* n1, ICBSNode* n2)
{
	auto [agent1_id, agent2_id, location1, location2, timestep] = *curr->conflict;


	if (split == split_strategy::RANDOM)  // A disjoint split that chooses the agent to work on randomly
	{
		int id;
		if (rand() % 2 == 0) {
			id = agent1_id;
		}
		else {
			id = agent2_id;
		}

		std::vector<std::unordered_map<int, AvoidanceState>>* catp = nullptr;
#ifndef LPA
#else
		// build a conflict-avoidance table for the agent we'll constrain
		std::vector < std::unordered_map<int, AvoidanceState > > cat(curr->makespan + 1);
		buildConflictAvoidanceTable(paths, id, *curr, cat);
		catp = &cat;
#endif

		n1->agent_id = id;
		n2->agent_id = id;
		if (location2 >= 0 && id == agent2_id) // Adding an edge constraint for the second agent - the constraint will
											   // be on traversing the edge in the opposite direction
		{
			LL_num_generated += n1->add_constraint(make_tuple(location2, location1, timestep, true), catp);
			LL_num_generated += n2->add_constraint(make_tuple(location2, location1, timestep, false), catp);
		}
		else
		{
			LL_num_generated += n1->add_constraint(make_tuple(location1, location2, timestep, true), catp);
			LL_num_generated += n2->add_constraint(make_tuple(location1, location2, timestep, false), catp);
		}

		addPositiveConstraintsOnNarrowLevelsLeadingToPositiveConstraint(id, timestep, n1, n2, catp);
	}
	else if (split == split_strategy::SINGLETONS)  // A disjoint split that chooses the agent to work on to be the one
												   // with the smaller number of 1-width levels in each agent's MDD,
												   // and if they're equal, the one with the smaller total width of all
												   // levels in its MDD, divided by the size of the MDD
	{
		int num_singletons_1 = countMddSingletons(agent1_id, timestep);
		int num_singletons_2 = countMddSingletons(agent2_id, timestep);
		int id;
		if (num_singletons_1 > num_singletons_2)
			id = agent1_id;
		else if (num_singletons_1 < num_singletons_2)
			id = agent2_id;
		else {
			double normalized_width1 = float(getTotalMddWidth(agent1_id)) / paths[agent1_id]->size();
			double normalized_width2 = float(getTotalMddWidth(agent2_id)) / paths[agent2_id]->size();
			if (normalized_width1 < normalized_width2)
				id = agent1_id;
			else
				id = agent2_id;
		}

		n1->agent_id = id;
		n2->agent_id = id;
		std::vector<std::unordered_map<int, AvoidanceState>>* catp = nullptr;
#ifndef LPA
#else
		// build a conflict-avoidance table for the agent we'll constrain
		std::vector < std::unordered_map<int, AvoidanceState > > cat(curr->makespan + 1);
		buildConflictAvoidanceTable(paths, id, *curr, cat);
		catp = &cat;
#endif

		if (location2 >= 0 && id == agent2_id) // Adding an edge constraint for the second agent - the constraint will
											   // be on traversing the edge in the opposite direction
		{
			LL_num_generated += n1->add_constraint(make_tuple(location2, location1, timestep, true), catp);
			LL_num_generated += n2->add_constraint(make_tuple(location2, location1, timestep, false), catp);
		}
		else
		{
			LL_num_generated += n1->add_constraint(make_tuple(location1, location2, timestep, true), catp);
			LL_num_generated += n2->add_constraint(make_tuple(location1, location2, timestep, false), catp);
		}

		if (max(num_singletons_1, num_singletons_2) > 1)  // There are narrow levels in the agent's MDD
			addPositiveConstraintsOnNarrowLevelsLeadingToPositiveConstraint(id, timestep, n1, n2, catp);
	}
	else if (split == split_strategy::WIDTH)
	{
		int id;
		if (timestep >= paths[agent1_id]->size())
			id = agent2_id;
		else if (timestep >= paths[agent2_id]->size())
			id = agent1_id;
		else if (paths[agent1_id]->at(timestep).numMDDNodes < paths[agent2_id]->at(timestep).numMDDNodes)
			id = agent1_id;
		else
			id = agent2_id;

		n1->agent_id = id;
		n2->agent_id = id;
		std::vector<std::unordered_map<int, AvoidanceState>>* catp = nullptr;
#ifndef LPA
#else
		// build a conflict-avoidance table for the agent we'll constrain
		std::vector < std::unordered_map<int, AvoidanceState > > cat(curr->makespan + 1);
		buildConflictAvoidanceTable(paths, id, *curr, cat);
		catp = &cat;
#endif

		if (location2 >= 0 && id == agent2_id) // Adding an edge constraint for the second agent - the constraint will
											   // be on traversing the edge in the opposite direction
		{
			LL_num_generated += n1->add_constraint(make_tuple(location2, location1, timestep, true), catp);
			LL_num_generated += n2->add_constraint(make_tuple(location2, location1, timestep, false), catp);
		}
		else
		{
			LL_num_generated += n1->add_constraint(make_tuple(location1, location2, timestep, true), catp);
			LL_num_generated += n2->add_constraint(make_tuple(location1, location2, timestep, false), catp);
		}

		addPositiveConstraintsOnNarrowLevelsLeadingToPositiveConstraint(id, timestep, n1, n2, catp);
	}
	else  // Do a non-disjoint split
	{
		n1->agent_id = agent1_id;
		n2->agent_id = agent2_id;

		std::vector<std::unordered_map<int, AvoidanceState>>* catp1 = nullptr;
		std::vector<std::unordered_map<int, AvoidanceState>>* catp2 = nullptr;
#ifndef LPA
#else
		// build conflict-avoidance tables for the agents we'll constrain
		std::vector < std::unordered_map<int, AvoidanceState > > cat1(curr->makespan + 1);
		std::vector < std::unordered_map<int, AvoidanceState > > cat2(curr->makespan + 1);
		buildConflictAvoidanceTable(paths, agent1_id, *curr, cat1);
		buildConflictAvoidanceTable(paths, agent2_id, *curr, cat2);
		catp1 = &cat1;
		catp2 = &cat2;
#endif

		LL_num_generated += n1->add_constraint(make_tuple(location1, location2, timestep, false), catp1);
		if (location2 >= 0) // Adding an edge constraint for the second agent - the constraint will
							// be on traversing the edge in the opposite direction
			LL_num_generated += n2->add_constraint(make_tuple(location2, location1, timestep, false), catp2);
		else
			LL_num_generated += n2->add_constraint(make_tuple(location1, location2, timestep, false), catp2);
	}
}

// Plan paths for a node. Returns whether a path was found.
// Assumes parent_paths was initialized with the paths of the node's parent.
bool ICBSSearch::generateChild(ICBSNode *node, vector<vector<PathEntry> *> &parent_paths)
{
	int h = 0;
	for (auto con : node->negative_constraints[node->agent_id])
	{
		auto [loc1, loc2, timestep, positive_constraint] = con;
		int a[2];
		if (split == split_strategy::DISJOINT3)
		{
			a[0] = node->agent_id / num_of_agents - 1;
			a[1] = node->agent_id % num_of_agents;
		}
		else
		{
			a[0] = node->agent_id;
			a[1] = -1;
		}
		for (int i = 0; i < 2 && a[i] >= 0; i++)
		{
			if (loc2 < 0 && timestep > parent_paths[a[i]]->size())  // The agent is forced out of its goal - the cost will surely increase
				// FIXME: Assumes the cost function is sum-of-costs
			{
				// Partial expansion - delay path finding:
				pair<int, vector<PathEntry>> newPath;
				newPath.first = a[i];
				newPath.second.resize(1);
				newPath.second.back().location = timestep;  // TODO: Why? Does it represent something?
				node->new_paths.push_back(newPath);
				h += timestep - (int)parent_paths[a[i]]->size();
				node->partialExpansion = true;
				continue;
			}
			int lowerbound;
			if (timestep >= (int)parent_paths[a[i]]->size()) // conflict happens after agent reaches its goal
				// (because constraints are on the same time as the conflict
				// or earlier due to propagation)
				lowerbound = timestep + 1;
			else if (!parent_paths[a[i]]->at(timestep).builtMDD) // unknown
				lowerbound = (int)parent_paths[a[i]]->size() - 1;
			else if (!parent_paths[a[i]]->at(timestep).single) // not cardinal
				lowerbound = (int)parent_paths[a[i]]->size() - 1;
			else if (loc1 >= 0 && loc2 < 0) // cardinal vertex
				lowerbound = (int)parent_paths[a[i]]->size();
			else if (parent_paths[a[i]]->at(timestep - 1).builtMDD && parent_paths[a[i]]->at(timestep - 1).single) // Cardinal edge
				lowerbound = (int)parent_paths[a[i]]->size();
			else // Not cardinal edge
				lowerbound = (int)parent_paths[a[i]]->size() - 1;

			if (!findPathForSingleAgent(node, parent_paths, nullptr, timestep, lowerbound, a[i]))
			{
				delete node;
				return false;
			}
		}
	}
	for (int ag = 0; ag < num_of_agents; ++ag) {
		if (ag == node->agent_id)
		{
			continue;  // Positive constraints affect all agents except the specified agent
		}

		for (auto con : node->positive_constraints[ag])
		{
			auto [loc1, loc2, timestep, positive_constraint] = con;
			if (loc2 < 0 &&  // vertex constraint
				getAgentLocation(parent_paths, timestep, ag) == loc1)  // The agent is in violation of the positive constraint
			{
				if (timestep > parent_paths[ag]->size())  // The agent is forced out of its goal - the cost will surely increase
					// FIXME: Assumes the cost function is sum-of-costs
				{
					// Partial expansion - postpone path finding:
					pair<int, vector<PathEntry>> newPath;
					newPath.first = ag;
					newPath.second.resize(1);
					newPath.second.back().location = timestep;
					node->new_paths.push_back(newPath);
					h += timestep - (int)parent_paths[ag]->size();
					node->partialExpansion = true;
				}
				else if (!findPathForSingleAgent(node, parent_paths, nullptr, timestep,
												 max((int) parent_paths[ag]->size() - 1, timestep), ag))
				{
					delete node;
					return false;
				}
			}
			else if (loc2 >= 0)  // edge constraint
			{
				if (getAgentLocation(parent_paths, timestep - 1, ag) == loc2 &&
					getAgentLocation(parent_paths, timestep, ag) == loc1) // move from "to" to "from"
				{
					if (!findPathForSingleAgent(node, parent_paths, nullptr, timestep,
												max((int) parent_paths[ag]->size() - 1, timestep), ag))
					{
						delete node;
						return false;
					}
				}
				else if (getAgentLocation(parent_paths, timestep - 1, ag) == loc1) // stay in location "from"
				{
					if (!findPathForSingleAgent(node, parent_paths, nullptr, timestep - 1,
												max((int) parent_paths[ag]->size() - 1, timestep), ag))
					{
						delete node;
						return false;
					}
				}
				else if (getAgentLocation(parent_paths, timestep, ag) == loc2) // stay in location "to"
				{
					if (!findPathForSingleAgent(node, parent_paths, nullptr, timestep,
												max((int) parent_paths[ag]->size() - 1, timestep), ag))
					{
						delete node;
						return false;
					}
				}
			}
		}
	}

	if (!node->partialExpansion) {
		// Check for partial expansion carried over from the parent or something
		// TODO: Can happen?
		for (const auto& path : node->new_paths) {
			if (path.second.size() <= 1) {
				node->partialExpansion = true;
				break;
			}
		}
	}

	// compute h value
	if (h > 0)  // from partial expansion
		node->h_val = h;
	else
	{
		node->h_val = max(node->parent->f_val - node->g_val, 0);

	}
	node->f_val = node->g_val + node->h_val;

	// Find the node's conflicts
	copyConflictsFromParent(*node);
	if (!node->partialExpansion)
	{
		findConflicts(*node);
	}

	node->num_of_conflicts = (int)node->unknownConf.size() + (int)node->cardinalConf.size() +
                             (int)node->semiConf.size() + (int)node->nonConf.size();

	// update handles
	node->open_handle = open_list.push(node);
	HL_num_generated++;
	node->time_generated = HL_num_generated;
	if (node->f_val <= focal_list_threshold)
		node->focal_handle = focal_list.push(node);
	allNodes_table.push_back(node);

	if (screen) // check the solution
		arePathsConsistentWithConstraints(paths, node);

	return true;
}

// plan a path for an agent in the node, also put the path in the_paths
bool ICBSSearch::findPathForSingleAgent(ICBSNode *node, vector<vector<PathEntry> *> &the_paths,
                                        vector<unordered_map<int, AvoidanceState >> *the_cat,
                                        int timestep, int earliestGoalTimestep, int ag, bool skipNewpaths)
{
	// extract all constraints on agent ag, and build constraint table
	ICBSNode* curr = node;
	pair<int,int> start(search_engines[ag]->start_location, 0), goal(search_engines[ag]->goal_location, INT_MAX);

	// build constraint table
	std::vector < std::unordered_map<int, ConstraintState > > cons_table(node->makespan + 1);
	int lastGoalConTimestep = buildConstraintTable(curr, ag, timestep, cons_table, start, goal);

	std::vector<std::unordered_map<int, AvoidanceState> > local_scope_cat(node->makespan + 1);
	if (the_cat == nullptr) {
	    // build conflict-avoidance table
	    buildConflictAvoidanceTable(the_paths, ag, *node, local_scope_cat);
	    the_cat = &local_scope_cat;
	}

	// Prepare the place we'll put the path
    vector<PathEntry>* newPath = nullptr;
    if (!skipNewpaths) {
        // Linear lookup:
        bool found = false;
        auto it = node->new_paths.begin();
        for ( ; it != node->new_paths.end(); ++it)
            // Check if the agent has an empty path in new_paths. If it does, update its path entry.
        {
            if (it->first == ag && it->second.size() <= 1)  // trivial entry - replace it
            {
                found = true;
                break;
            }
            if (it->first > ag)  // Passed it - insert here
                break;
        }
        if (!found)
            it = node->new_paths.emplace(it);  // Inserts before the iterator
        it->first = ag;
        newPath = &it->second;
    } else
        newPath = new vector<PathEntry>();

	// A path w.r.t cons_table (and prioritize by the conflict-avoidance table).
	bool foundSol;

	// TODO: Pass the timeout to the low level
	if (goal.second  >= the_paths[ag]->size())
	{
#ifndef LPA
		clock_t ll_start = std::clock();
		auto wall_ll_start = std::chrono::system_clock::now();
		foundSol = search_engines[ag]->findShortestPath(*newPath, cons_table, *the_cat,
		                                                start, goal, earliestGoalTimestep, lastGoalConTimestep);
		lowLevelTime += std::clock() - ll_start;
		wall_lowLevelTime += std::chrono::system_clock::now() - wall_ll_start;
		LL_num_expanded += search_engines[ag]->num_expanded;
		LL_num_generated += search_engines[ag]->num_generated;
#else
		if (start.second == 0 && goal.second == INT_MAX &&
				node->lpas[ag] != NULL
				) {
			if (screen)
				cout << "Calling LPA* again for agent " << ag << endl;
			int generated_before = node->lpas[ag]->allNodes_table.size();
			clock_t ll_start = std::clock();
			auto wall_ll_start = std::chrono::system_clock::now();
			foundSol = node->lpas[ag]->findPath(*the_cat, earliestGoalTimestep, lastGoalConTimestep);
			lowLevelTime += std::clock() - ll_start;
			wall_lowLevelTime += std::chrono::system_clock::now() - wall_ll_start;
			if (foundSol) {
				const vector<int> *primitive_path = node->lpas[ag]->getPath(node->lpas[ag]->paths.size() - 1);
                newPath->resize(primitive_path->size());
				for (int j = 0; j < primitive_path->size(); ++j) {
                    newPath->operator[](j).location = (*primitive_path)[j];
                    newPath->operator[](j).builtMDD = false;
				}
				// Mark the first and last locations as "cardinal locations" regardless of the MDD:
                newPath->operator[](0).builtMDD = true;
                newPath->operator[](0).single = true;
                newPath->operator[](0).numMDDNodes = 1;
                newPath->operator[](primitive_path->size() - 1).builtMDD = true;
                newPath->operator[](primitive_path->size() - 1).single = true;
                newPath->operator[](primitive_path->size() - 1).numMDDNodes = 1;
			}
			LL_num_expanded += node->lpas[ag]->num_expanded[node->lpas[ag]->paths.size() - 1];
			LL_num_generated += node->lpas[ag]->allNodes_table.size() - generated_before;
		}
		else
		{
			if (screen)
				cout << "Calling normal A* for agent " << ag << " instead of LPA* because LPA* can't handle a changed "
						"start, and can't recover if it was unable in the past" << endl;
			if (node->lpas[ag] != NULL) {
				delete node->lpas[ag];  // We've just created this copy - it's unused anywhere else
				node->lpas[ag] = NULL;
			}
			clock_t ll_start = std::clock();
			auto wall_ll_start = std::chrono::system_clock::now();
			foundSol = search_engines[ag]->findShortestPath(*newPath, cons_table, *the_cat, start, goal,
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
		foundSol = search_engines[ag]->findPath(*newPath, cons_table, *the_cat, start, goal, lowlevel_hval::DH);
		lowLevelTime += std::clock() - ll_start;
		wall_lowLevelTime += std::chrono::system_clock::now() - wall_ll_start;
		LL_num_expanded += search_engines[ag]->num_expanded;
		LL_num_generated += search_engines[ag]->num_generated;
	}

	if (!foundSol)
		return false;

    // update the_paths, g_val, and makespan
    node->g_val = node->g_val - (int) the_paths[ag]->size() + (int) newPath->size();
    if (skipNewpaths && the_paths[ag] != &paths_found_initially[ag])
        delete the_paths[ag];
    the_paths[ag] = newPath;
    node->makespan = max(node->makespan, (int)newPath->size() - 1);

	if (screen)
		this->printPaths(the_paths);

	return true;
}

// plan paths that are not planned yet due to partial expansion
bool ICBSSearch::finishPartialExpansion(ICBSNode *node, vector<vector<PathEntry> *> &the_paths)
{
	for(auto p: node->new_paths)
	{
		if (p.second.size() <= 1)
		{
			if (!findPathForSingleAgent(node, the_paths, nullptr, p.second.back().location, p.second.back().location, p.first))
			{
				delete node;
				return false;
			}
		}
	}
	node->h_val = max(node->parent->f_val - node->g_val, 0);
	node->f_val = node->g_val + node->h_val;
	findConflicts(*node);  // Conflicts involving agents whose paths were unchanged in this node were already copied from the parent
	node->num_of_conflicts = (int) node->unknownConf.size() + (int) node->cardinalConf.size() +
                             (int) node->semiConf.size() + (int) node->nonConf.size();

	HL_num_reexpanded++;
	node->partialExpansion = false;
	return true;
}


//////////////////// TOOLS ///////////////////////////
// check whether the new planned path obeys the constraints -- for debug
bool ICBSSearch::arePathsConsistentWithConstraints(vector<vector<PathEntry> *> &the_paths, ICBSNode *curr) const
{
	if (curr->partialExpansion)
		return true;
	while (curr != nullptr)
	{
		for (int agent = 0; agent < num_of_agents; ++agent) {
			for (auto con : curr->negative_constraints[agent]) {
				auto [loc1, loc2, timestep, positive_constraint] = con;
				if (loc2 < 0) // vertex constraint
				{
					if (the_paths[agent]->size() > timestep && the_paths[agent]->at(timestep).location == loc1)
					{
						std::cout << "Path " << agent << " violates constraint " << con << std::endl;
						exit(1);
					}
					else if (the_paths[agent]->size() <= timestep && the_paths[agent]->back().location == loc1)
					{
						std::cout << "Path " << agent << " violates constraint " << con << std::endl;
						exit(1);
					}
				}
				else // edge constraint
				{
					if (timestep < the_paths[agent]->size() &&  // Otherwise we're fine - can't violate an edge constraint by WAITing
						the_paths[agent]->at(timestep - 1).location == loc1 &&
						the_paths[agent]->at(timestep).location == loc2)
					{
						std::cout << "Path " << agent << " violates constraint " << con << std::endl;
						exit(1);
					}
				}
			}

			for (auto con : curr->positive_constraints[agent]) {
				auto [loc1, loc2, timestep, positive_constraint] = con;
				for (int other_agent = 0; other_agent < num_of_agents; other_agent++)
				{
					if (other_agent == agent)
					{
						if (loc2 < 0) // vertex constraint
						{
							if (timestep < the_paths[other_agent]->size() && the_paths[other_agent]->at(timestep).location != loc1)
							{
								std::cout << "Path " << other_agent << " violates constraint " << con << std::endl;
								exit(1);
							}
						}
						else // edge constraint
						{
							if (timestep < the_paths[other_agent]->size() &&
								(the_paths[other_agent]->at(timestep - 1).location != loc1 ||
								 the_paths[other_agent]->at(timestep).location != loc2))
							{
								std::cout << "Path " << other_agent << " violates constraint " << con << std::endl;
								exit(1);
							}
						}
					}
					else  // The positive constraint is on a different agent - an implicit negative constraint for this agent
					{
						if (timestep >= the_paths[other_agent]->size())
						{
							if (loc2 < 0 && the_paths[other_agent]->at(the_paths[other_agent]->size() - 1).location == loc1)
							{
								std::cout << "Path " << other_agent << " violates constraint " << con << std::endl;
								exit(1);
							}
						}
						else if (loc2 < 0) // vertex constraint
						{
							if (the_paths[other_agent]->at(timestep).location == loc1)
							{
								std::cout << "Path " << other_agent << " violates constraint " << con << std::endl;
								exit(1);
							}
						}
						else // edge constraint
						{
							if ((the_paths[other_agent]->at(timestep - 1).location == loc2 && the_paths[other_agent]->at(timestep).location == loc1) ||
								the_paths[other_agent]->at(timestep - 1).location == loc1 ||
								the_paths[other_agent]->at(timestep).location == loc2)
							{
								std::cout << "Path " << other_agent << " violates constraint " << con << std::endl;
								exit(1);
							}
						}
					}
				}
			}
		}

		curr = curr->parent;
	}
	return true;
}

// check whether the paths are feasible -- for debug
void ICBSSearch::isFeasible() const
{
	if (!solution_found)
		return;
	else if (solution_cost != min_f_val)
		cout << "Solution cost " << solution_cost << " != min f val " << min_f_val << endl;
	int sum = 0;
	for (int i = 0; i < num_of_agents; i++)
	{
		sum += (int)paths[i]->size() - 1;
		if (paths[i]->front().location != search_engines[i]->start_location)
			cout << "Start Wrong" << endl;
		if (paths[i]->back().location != search_engines[i]->goal_location)
			cout << "Goal Wrong" << endl;
		for (int t = 1; t < paths[i]->size(); t++)
		{
			if (search_engines[0]->my_map[paths[i]->at(t).location])
				cout << "Obstacle!" << endl;
			int move = paths[i]->at(t).location - paths[i]->at(t - 1).location;
			bool valid = false;
			for (int j = 0; j < 5; j++)
				if (move == search_engines[0]->moves_offset[j])
					valid = true;
			if (!valid)
				cout << "Invalid Move for agent " << i << " from " << paths[i]->at(t - 1).location << " to " << paths[i]->at(t).location << endl;
		}
		for (int j = i + 1; j < num_of_agents; j++)
		{
			if (paths[i]->at(0).location == paths[j]->at(0).location)
				cout << "Start position collision between agent " << i << " and " << j << "!" << endl;
			int t_min = (int)min(paths[i]->size(), paths[j]->size());
			if (t_min == 0)
				continue;
			for (int t = 1; t < t_min; t++)
			{
				if (paths[i]->at(t).location == paths[j]->at(t).location)
					cout << "Vertex collision between agent " << i << " and " << j << " at time " << t << "!" << endl;
				else if (paths[i]->at(t).location == paths[j]->at(t - 1).location &&
					paths[i]->at(t - 1).location == paths[j]->at(t).location)
					cout << "Edge collision between agent " << i << " and " << j << " at time " << t << "!" << endl;
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
				if (paths[a]->at(t_min - 1).location == paths[b]->at(t).location)
					cout << "Vertex collision between agent " << i << " and " << j << " at time " << t << "!" << endl;
			}
		}
	}
	if (sum != solution_cost)
		cout << "Solution cost wrong!" << endl;
}

// Returns the ID of the location the given agent is in at the given timestep according to the given paths
// of the current node we're working on
inline int ICBSSearch::getAgentLocation(vector<vector<PathEntry> *> &the_paths, size_t timestep, int agent_id)
{
	// if last timestep > plan length, agent remains in its last location
	if (timestep >= the_paths[agent_id]->size())
		return the_paths[agent_id]->at(the_paths[agent_id]->size() - 1).location;
	// otherwise, return its location for that timestep
	return the_paths[agent_id]->at(timestep).location;
}

// Populates the search's paths member by scanning from the given node towards the root node and
// taking the first constrained path encountered for each agent. Agents with no
// constrained path take the original path they got in the root node.
inline void ICBSSearch::populatePaths(ICBSNode *curr, vector<vector<PathEntry> *> &the_paths)
{
	vector<bool> path_already_found(num_of_agents, false);  // initialized with false for all agents
	while (curr->parent != NULL)
	{
		for (list<pair<int, vector<PathEntry>>>::iterator it = curr->new_paths.begin(); it != curr->new_paths.end(); it++)
		{
			if (!path_already_found[it->first] && it->second.size() > 1)
			{
				the_paths[it->first] = &(it->second);
				path_already_found[it->first] = true;
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
	for (ICBSNode* n : open_list) 
	{
		if (n->f_val > old_lower_bound && n->f_val <= new_lower_bound)
			n->focal_handle = focal_list.push(n);
	}
}

// Reinsert a node to OPEN (and FOCAL if needed) because of the lazy heuristics
bool ICBSSearch::reinsert(ICBSNode* curr)
{
	if (curr->f_val > focal_list_threshold)
	{
		curr->open_handle = open_list.push(curr);
		ICBSNode* open_head = open_list.top();
		if (open_head->f_val > min_f_val) 
		{
			min_f_val = open_head->f_val;
			double new_focal_list_threshold = min_f_val * focal_w;
			updateFocalList(focal_list_threshold, new_focal_list_threshold, focal_w);
			focal_list_threshold = new_focal_list_threshold;
		}
		return true;
	}
	else
		return false;
}


//////////////////// PRINT ///////////////////////////
void ICBSSearch::printPaths(vector<vector<PathEntry> *> &the_paths) const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		std::cout << "Agent " << i << " (" << paths_found_initially[i].size() - 1 << " -->" <<
			the_paths[i]->size() - 1 << "): ";
		for (int t = 0; t < the_paths[i]->size(); t++)
			std::cout << "(" << the_paths[i]->at(t).location / num_map_cols << "," << the_paths[i]->at(t).location % num_map_cols << ")->";
		std::cout << std::endl;
	}
}

void ICBSSearch::printConflicts(const ICBSNode &curr) const
{
	for (const auto& conflict: curr.cardinalConf)
	{
		std::cout << "Cardinal " << (*conflict) << std::endl;
	}
	for (const auto& conflict : curr.semiConf)
	{
		std::cout << "Semi-cardinal " << (*conflict) << std::endl;
	}
	for (const auto& conflict : curr.nonConf)
	{
		std::cout << "Non-cardinal " << (*conflict) << std::endl;
	}
	for (const auto& conflict : curr.unknownConf)
	{
		std::cout << "Unknown-cardinality " << (*conflict) << std::endl;
	}
}

void ICBSSearch::printConstraints(const ICBSNode* n) const
{
	const ICBSNode* curr = n;
	while (curr != nullptr)
	{
		for (int j = 0; j < num_of_agents; ++j) {
			for (auto con: curr->positive_constraints[j])
			{
				std::cout << curr->agent_id << ": " << con << std::endl;
			}
			for (auto con: curr->negative_constraints[j])
			{
				std::cout << curr->agent_id << ": " << con << std::endl;
			}
		}
		curr = curr->parent;
	}
}

void ICBSSearch::printResults() const
{
	std::cout << "Status,Cost,Focal Delta,Root Cost,Root f,Wall PrepTime,PrepTime,Wall MDD Time,MDD Time,"
				 "HL Expanded,HL Generated,LL Expanded,LL Generated,Wall HL runtime,HL runtime,"
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
		min_f_val - root_node->g_val << "," <<
		root_node->g_val << "," << root_node->f_val << "," <<
		((float) wall_prepTime.count()) / 1000000000 << "," <<
		((float) prepTime) / CLOCKS_PER_SEC << "," <<
		((float) wall_mddTime.count()) / 1000000000 << "," <<
		((float) highLevelMddBuildingTime) / CLOCKS_PER_SEC << "," <<
		HL_num_expanded << "," << HL_num_generated << "," <<
		LL_num_expanded << "," << LL_num_generated << "," <<
		((float) wall_highLevelTime.count()) / 1000000000 << "," <<
		((float) highLevelTime) / CLOCKS_PER_SEC << "," <<
		((float) wall_lowLevelTime.count()) / 1000000000 << "," <<
		((float) lowLevelTime) / CLOCKS_PER_SEC << "," <<
		((float) wall_runtime.count()) / 1000000000 << "," <<
		((float) runtime) / CLOCKS_PER_SEC << "," <<
		max_mem <<
		std::endl;
}

void ICBSSearch::saveResults(const string& outputFile, const string& agentFile, const string& solver) const
{
	ofstream stats;
	if (std::filesystem::exists(outputFile) == false)
	{
		stats.open(outputFile);
		stats << "Cost,Focal Delta,Root Cost,Root f,Wall PrepTime,PrepTime,Wall MDD Time,MDD Time,"
		          "HL Expanded,HL Generated,LL Expanded,LL Generated,Wall HL runtime,HL runtime,"
		          "Wall LL runtime,LL runtime,Wall Runtime,Runtime,Max Mem (kB),solver,instance" << std::endl;
	}
	else
		stats.open(outputFile, ios::app);
	stats << solution_cost << "," << 
		min_f_val - root_node->g_val << "," <<
		root_node->g_val << "," << root_node->f_val << "," <<
		((float) wall_prepTime.count()) / 1000000000 << "," <<
		((float) prepTime) / CLOCKS_PER_SEC << "," <<
        ((float) wall_mddTime.count()) / 1000000000 << "," <<
        ((float) highLevelMddBuildingTime) / CLOCKS_PER_SEC << "," <<
		HL_num_expanded << "," << HL_num_generated << "," <<
		LL_num_expanded << "," << LL_num_generated << "," <<
		((float) wall_highLevelTime.count()) / 1000000000 << "," <<
		((float) highLevelTime) / CLOCKS_PER_SEC << "," <<
		((float) wall_lowLevelTime.count()) / 1000000000 << "," <<
		((float) lowLevelTime) / CLOCKS_PER_SEC << "," <<
		((float) wall_runtime.count()) / 1000000000 << "," <<
		((float) runtime) / CLOCKS_PER_SEC << "," <<
		max_mem << "," <<
		solver << "," << agentFile << endl;
	stats.close();
}

//////////////////// MAIN FUNCTIONS ///////////////////////////
// RUN CBS
bool ICBSSearch::runICBSSearch() 
{
	if (HL_heuristic)
#ifndef LPA
		std::cout << "CBSH: " << std::endl;
#else
		std::cout << "CBSH/LPA*: " << std::endl;
#endif
	else
#ifndef LPA
		std::cout << "ICBS: " << std::endl;
#else
		std::cout << "ICBS/LPA*: " << std::endl;
#endif
	// set timer
	std::clock_t start = std::clock();
	auto wall_start = std::chrono::system_clock::now();

	while (!focal_list.empty()) 
	{
		runtime = std::clock() - start;
		wall_runtime = std::chrono::system_clock::now() - wall_start;
		if (runtime > time_limit * CLOCKS_PER_SEC)  // timeout
		{
			break;
		}

		// pop the best node from FOCAL
		ICBSNode* curr = focal_list.top();
		focal_list.pop();
		open_list.erase(curr->open_handle);

		// get current solutions
		populatePaths(curr, paths);

		if (curr->partialExpansion)
		{
			bool Sol = finishPartialExpansion(curr, paths);
			if (!Sol || reinsert(curr))
				continue;
		}
		if (!HL_heuristic) // No heuristics
		{
			curr->conflict = classifyConflicts(*curr, paths); // choose conflict
			
			if ( screen == 1)
				printConflicts(*curr);
		}
		else if (curr->conflict == NULL) //CBSH, and h value has not been computed yet
		{					
			curr->conflict = classifyConflicts(*curr, paths); // classify and choose conflicts

			curr->h_val = computeHeuristics(*curr);
			curr->f_val = curr->g_val + curr->h_val;

			if (screen == 1)
			{
				std::cout << std::endl << "****** Computed h for #" << curr->time_generated
						  << " with f= " << curr->g_val << "+" << curr->h_val << " (";
				for (int i = 0; i < num_of_agents; i++)
					std::cout << paths[i]->size() - 1 << ", ";
				std::cout << ") and #conflicts = " << curr->num_of_conflicts << std::endl;
				printConflicts(*curr);
			}

			if (reinsert(curr))
			{	
				continue;
			}
		}

		if (curr->conflict == NULL) // Failed to find a conflict => no conflicts
		{  // found a solution (and finish the while loop)
			solution_found = true;
			solution_cost = curr->g_val;
			break;
		}


		// Expand the node
		HL_num_expanded++;
		curr->time_expanded = HL_num_expanded;

		if (screen == 1)
		{
			std::cout << std::endl << "****** Expanding #" << curr->time_generated
				<< " with f= " << curr->g_val <<	 "+" << curr->h_val << " (";
			for (int i = 0; i < num_of_agents; i++)
				std::cout << paths[i]->size() - 1 << ", ";
			std::cout << ")" << std::endl;
			std::cout << "Chose conflict " << (*curr->conflict) << std::endl;
		}

		if (split == split_strategy::DISJOINT3)
		{
//			ICBSNode* n1 = new ICBSNode(curr);
//			ICBSNode* n2 = new ICBSNode(curr);
//			ICBSNode* n3 = new ICBSNode(curr);
//			auto [agent1_id, agent2_id, location1, location2, timestep] = *curr->conflict;
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
//			vector<vector<PathEntry>*> copy(paths);
//			bool Sol1 = generateChild(n1);
//			paths = copy;
//			bool Sol2 = generateChild(n2);
//			paths = copy;
//			bool Sol3 = generateChild(n3);
		}
		else
		{
			ICBSNode* n1 = new ICBSNode(curr);
			ICBSNode* n2 = new ICBSNode(curr);
		
			branch(curr, n1, n2); // add constraints to child nodes

			bool success1 = false, success2 = false;
			vector<vector<PathEntry>*> temp(paths);
			success1 = generateChild(n1, paths); // plan paths for n1
			if (screen == 1) {
				if (success1) {
					std::cout << "Generated left child #" << n1->time_generated
							  << " with cost " << n1->g_val
							  << " and " << n1->num_of_conflicts << " conflicts " << std::endl;
				} else {
					std::cout << "No feasible solution for left child! " << std::endl;
				}
			}
			paths = temp;
			success2 = generateChild(n2, paths); // plan paths for n2

			if (screen == 1)
			{
				if (success2)
				{
					std::cout << "Generated right child #" << n2->time_generated
							  << " with cost " << n2->g_val
							  << " and " << n2->num_of_conflicts << " conflicts " << std::endl;
				}
				else
				{
					std::cout << "No feasible solution for right child! " << std::endl;
				}
			}
		}
		curr->clear();
		if (open_list.size() == 0) 
		{
			solution_found = false;
			break;
		}
		ICBSNode* open_head = open_list.top();
		if (open_head->f_val > min_f_val) 
		{
			if (screen == 1)
				cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() 
					<< " with |OPEN|=" << open_list.size() << " to |FOCAL|=";

			min_f_val = open_head->f_val;
			double new_focal_list_threshold = min_f_val * focal_w;
			updateFocalList(focal_list_threshold, new_focal_list_threshold, focal_w);
			focal_list_threshold = new_focal_list_threshold;
			if (screen == 1)
				cout << focal_list.size() << endl;

		}
		if (screen == 1)
			cout << " ; (after) " << focal_list_threshold << endl << endl;
	}  // end of while loop

	runtime = std::clock() - start; //  get time
	wall_runtime = std::chrono::system_clock::now() - wall_start;
	highLevelTime = runtime - lowLevelTime;
	wall_highLevelTime = wall_runtime - wall_lowLevelTime;
	printPaths(paths);
	return solution_found;
}

// RUN ID-CBS/LPA*
bool ICBSSearch::runIterativeDeepeningICBSSearch()
{
	if (HL_heuristic)
#ifndef LPA
		std::cout << "CBSH: " << std::endl;
#else
		std::cout << "ID-CBSH/LPA*: " << std::endl;
#endif
	else
#ifndef LPA
		std::cout << "ICBS: " << std::endl;
#else
		std::cout << "ID-ICBS/LPA*: " << std::endl;
#endif
	// set timer
	std::clock_t start = std::clock();
	auto wall_start = std::chrono::system_clock::now();

	vector<vector<PathEntry>*> the_paths;
	the_paths.resize(num_of_agents, NULL);
	populatePaths(root_node, the_paths);

	// Add the path of the last agent to the running CAT
	if (root_node->makespan + 1 > root_cat.size())
	{
		root_cat.resize(root_node->makespan + 1);
		buildConflictAvoidanceTable(paths, num_of_agents, *root_node, root_cat);
	}
	else
		addPathToConflictAvoidanceTable(paths[num_of_agents-1], root_cat);

	root_node->conflict = classifyConflicts(*root_node, the_paths); // classify and choose conflicts

	// Compute the root node's h value, to be used for setting the first iteration's threshold:
	if (HL_heuristic) {
		root_node->h_val = computeHeuristics(*root_node);
		if (screen == 1) {
			std::cout << std::endl << "****** Computed h for #" << root_node->time_generated
					  << " with f= " << root_node->g_val << "+" << root_node->h_val << " (";
			for (int i = 0; i < num_of_agents; i++)
				std::cout << paths[i]->size() - 1 << ", ";
			std::cout << ") and #conflicts = " << root_node->num_of_conflicts << std::endl;
		}
	}
	else
		root_node->h_val = 0;
	if (screen == 1) {
		printConflicts(*root_node);
	}
	root_node->f_val = root_node->g_val + root_node->h_val;

	// Set the first threshold
	int threshold = root_node->f_val;

	while (true)
	{
		runtime = std::clock() - start;
		wall_runtime = std::chrono::system_clock::now() - wall_start;
		if (runtime > time_limit * CLOCKS_PER_SEC)  // timeout
		{
			break;
		}

		if (screen)  // TODO: Use a high glog instead
			std::cout << "Threshold: " << threshold << std::endl;
		int start_expanded = HL_num_expanded;
		root_node->time_generated = 1;
		HL_num_expanded_before_this_iteration = HL_num_expanded;
		HL_num_generated_before_this_iteration = HL_num_generated;
		HL_num_generated_before_this_iteration--;  // Simulate that the root node was generated for this iteration
		auto [solved, next_threshold] = do_idcbsh_iteration(root_node, the_paths, root_cat,
    	                                                    threshold, std::numeric_limits<int>::max(),
    	                                                    start + time_limit * CLOCKS_PER_SEC);
		if (solved)
			break;
		if (screen)  // TODO: Use a high glog instead
			std::cout << "Finished threshold " << threshold << ". Expanded " << HL_num_expanded - start_expanded <<
					  " nodes in " << std::clock() - start << " seconds." << std::endl;
		if (threshold == next_threshold) { // Unsolvable instance?
			std::cout << "Next threshold not found! Unsolvable??" << std::endl;
			break;
		}

		if (screen) {
            for (const auto &agent_neg_constraints: root_node->negative_constraints) {
                if (agent_neg_constraints.size() != 0) {
                    std::cout << "IDCBS root has constraints at end of threshold " << threshold << "!" << std::endl;
                    std::abort();
                }
            }
            for (auto lpa: root_node->lpas) {
                for (const auto &dyn_constraints_for_timestep : lpa->dcm.dyn_constraints_) {
                    if (dyn_constraints_for_timestep.size() != 0) {
                        std::cout << "LPA* for agent " << lpa->agent_id << " has constraints at end of threshold "
                                  << threshold << "!" << std::endl;
                        std::abort();
                    }
                }
            }
        }

		threshold = next_threshold;
	}  // end of while loop

	runtime = std::clock() - start; //  get time
	wall_runtime = std::chrono::system_clock::now() - wall_start;
	highLevelTime = runtime - lowLevelTime;
	wall_highLevelTime = wall_runtime - wall_lowLevelTime;
	paths = the_paths;  // For isFeasible
	printPaths(paths);
	return solution_found;
}

std::tuple<bool, int> ICBSSearch::do_idcbsh_iteration(ICBSNode *curr, vector<vector<PathEntry> *> &the_paths,
                                                      vector<unordered_map<int, AvoidanceState >> &the_cat,
                                                      int threshold, int next_threshold, clock_t end_by) {
	if (std::clock() > end_by)  // timeout (no need to unconstrain)
	{
		return make_tuple(false, next_threshold);
	}

	if (!HL_heuristic) // Then conflicts are not yet classified
	{
		curr->conflict = classifyConflicts(*curr, the_paths); // choose conflict

		if (screen == 1)
			printConflicts(*curr);
	} else if (curr->conflict == nullptr) //CBSH, and h value has not been computed yet
	{
		curr->conflict = classifyConflicts(*curr, the_paths); // classify and choose conflicts

		curr->h_val = computeHeuristics(*curr);
		curr->f_val = curr->g_val + curr->h_val;

		if (screen == 1) {
			std::cout << std::endl << "****** Computed h for #" << curr->time_generated
					  << " with f= " << curr->g_val << "+" << curr->h_val << " (";
			for (int i = 0; i < num_of_agents; i++)
				std::cout << the_paths[i]->size() - 1 << ", ";
			std::cout << ") and #conflicts = " << curr->num_of_conflicts << std::endl;
			printConflicts(*curr);
		}

		if (curr->f_val > threshold) {
			if (screen)
				std::cout << "F value " << curr->f_val << " < " << threshold << " threshold. Backtracking." << std::endl;

			next_threshold = min(next_threshold, curr->f_val);
			return make_tuple(false, next_threshold);  // The parent will unconstrain
		}
	}

	if (curr->conflict == NULL) // Failed to find a conflict => no conflicts => found a solution
	{
		solution_found = true;
		solution_cost = curr->g_val;
		min_f_val = curr->f_val;  // Just to shut a focal list feasibility test up
		return make_tuple(true, next_threshold);
	}

	// Expand the node
	HL_num_expanded++;
	curr->time_expanded = HL_num_expanded - HL_num_expanded_before_this_iteration;

	if (screen == 1)
	{
		std::cout << std::endl << "****** Expanding #" << curr->time_generated
				  << " with f= " << curr->g_val <<	 "+" << curr->h_val << " (";
		for (int i = 0; i < num_of_agents; i++)
			std::cout << the_paths[i]->size() - 1 << ", ";
		std::cout << ")" << std::endl;
		std::cout << "Chosen conflict: " << *curr->conflict << std::endl;
	}

	auto [agent1_id, agent2_id, location1, location2, timestep] = *curr->conflict;

	std::shared_ptr<Conflict> orig_conflict = curr->conflict;  // TODO: Consider not storing the conflict as a state of the node
	int orig_agent_id = curr->agent_id;
	vector<PathEntry> path_backup = *the_paths[agent1_id];
	int orig_makespan = curr->makespan;
	int orig_g_val = curr->g_val;
	int orig_h_val = curr->h_val;
	// TODO: node->new_paths is accumulating unused paths needlessly. Handle it sometime. Make sure buildMDD still works. It currently relies on new_paths. Might need a separate idcbsh function that just takes the current node's makespan.

	curr->agent_id = agent1_id;
	auto [replan1_success, constraint1_added] = idcbsh_add_constraint_and_replan(
			curr, the_paths, the_cat,
			next_threshold - curr->g_val - 1);  // If the cost will be larger than that, don't bother generating the node
														 // - it won't even update next_threshold
	int g_delta = curr->g_val - orig_g_val;
	// Subtract the g_delta from h, just to be nice:
	if (curr->h_val >= g_delta)
		curr->h_val -= g_delta;
	else
		curr->h_val = 0;
	curr->f_val = curr->g_val + curr->h_val;

	if (replan1_success) {
		HL_num_generated++;
		curr->time_generated = HL_num_generated - HL_num_generated_before_this_iteration;
		if (screen) {
			// check the solution
			arePathsConsistentWithConstraints(the_paths, curr);

			// Print
			std::cout << "Generated left child #" << curr->time_generated
					  << " with cost " << curr->g_val
					  << " and " << curr->num_of_conflicts << " conflicts " << std::endl;
		}
	}

	if (replan1_success && curr->f_val <= threshold)
	{
		// Find the node's conflicts:
		clearConflictsOfAgent(*curr, curr->agent_id);
		for (int a2 = 0; a2 < num_of_agents; a2++) {
			if (a2 != curr->agent_id)
				findConflicts(the_paths, curr->agent_id, a2, *curr);
		}
		curr->num_of_conflicts = (int) curr->unknownConf.size() + (int) curr->cardinalConf.size() +
								 (int) curr->semiConf.size() + (int) curr->nonConf.size();
		curr->conflict = nullptr;  // Trigger computation of h in the recursive call

		// Recurse!
		auto [success, lowest_avoided_f_val] = do_idcbsh_iteration(curr, the_paths, the_cat,
		                                                           threshold, next_threshold, end_by);
		next_threshold = min(next_threshold, lowest_avoided_f_val);  // lowest_avoided_f_val might actually be just the
																	 // next_threshold we provided

		if (success) {
			curr->agent_id = orig_agent_id;  // Just to be clean
			return make_tuple(true, next_threshold);
		} else {
			curr->agent_id = agent1_id;  // The recursive call may have changed it, and it needs to be restored before unconstrain is called
		}
	} else {
		// A path was not found for the constrained agent in the left child, or the overall g was higher than the threshold
		if (curr->f_val > threshold)  // The overall g was higher than the threshold
			next_threshold = min(next_threshold, curr->f_val);

		if (screen == 1) {
			if (!constraint1_added)
				std::cout << "The left child's cost would have been larger than the NEXT threshold " <<
						  next_threshold << "." << std::endl;
			else if (curr->f_val <= threshold)
				std::cout << "No solution for the left child! " << std::endl;
			else
				std::cout << "The left child's F value " << curr->f_val << " < " << threshold << " threshold." << std::endl;
		}
	}

	// A goal was not found with a cost below the threshold in the left child
	if (constraint1_added)
		idcbsh_unconstrain(curr, the_paths, the_cat, path_backup, orig_conflict, orig_makespan, orig_g_val, orig_h_val);

	curr->agent_id = agent2_id;

	path_backup = *the_paths[agent2_id];

	auto [replan2_success, constraint2_added] = idcbsh_add_constraint_and_replan(
			curr, the_paths, the_cat,
            next_threshold - curr->g_val - 1);  // If the cost will be larger than that, don't bother generating the node
														 // - it won't even update next_threshold
	g_delta = curr->g_val - orig_g_val;
	// Subtract the g_delta from h, just to be nice:
	if (curr->h_val >= g_delta)
		curr->h_val -= g_delta;
	else
		curr->h_val = 0;
	curr->f_val = curr->g_val + curr->h_val;

	if (replan2_success) {
		HL_num_generated++;
		curr->time_generated = HL_num_generated - HL_num_generated_before_this_iteration;
		if (screen) {
			// check the solution
			arePathsConsistentWithConstraints(the_paths, curr);

			// Print
			std::cout << "Generated right child #" << curr->time_generated
					  << " with cost " << curr->g_val
					  << " and " << curr->num_of_conflicts << " conflicts " << std::endl;
		}
	}

	if (replan2_success && curr->f_val <= threshold)
	{
		// Find the node's conflicts:
		clearConflictsOfAgent(*curr, curr->agent_id);
		for (int a2 = 0; a2 < num_of_agents; a2++) {
			if (a2 != curr->agent_id)
				findConflicts(the_paths, curr->agent_id, a2, *curr);
		}
		curr->num_of_conflicts = (int) curr->unknownConf.size() + (int) curr->cardinalConf.size() +
								 (int) curr->semiConf.size() + (int) curr->nonConf.size();
		curr->conflict = nullptr;  // Trigger computation of h in the recursive call

		// Recurse!
		auto [success, lowest_avoided_f_val] = do_idcbsh_iteration(curr, the_paths, the_cat,
                                                                   threshold, next_threshold, end_by);
		next_threshold = min(next_threshold, lowest_avoided_f_val);

		if (success) {
			curr->agent_id = orig_agent_id;  // Just to be clean
			return make_tuple(true, next_threshold);
		} else {
			curr->agent_id = agent2_id;  // The recursive call may have changed it, and it needs to be restored before unconstrain is called
			if (constraint2_added)
				idcbsh_unconstrain(curr, the_paths, the_cat, path_backup, orig_conflict, orig_makespan, orig_g_val, orig_h_val);
			curr->agent_id = orig_agent_id;  // Just to be clean
			return make_tuple(false, next_threshold);
		}
	} else {
		// A path was not found for the constrained agent in the left child, or the overall g was higher than the threshold
		if (screen == 1) {
			if (!constraint2_added)
				std::cout << "The right child's cost would have been larger than the NEXT threshold " <<
				next_threshold << "." << std::endl;
			else if (curr->f_val <= threshold)
				std::cout << "No solution for the right child! " << std::endl;
			else
				std::cout << "The right child's F value " << curr->f_val << " < " << threshold
						  << " threshold." << std::endl;
		}
	}
	curr->agent_id = agent2_id;  // The recursive call may have changed it, and it needs to be restored before unconstrain is called
	if (constraint2_added)
		idcbsh_unconstrain(curr, the_paths, the_cat, path_backup, orig_conflict, orig_makespan, orig_g_val, orig_h_val);
	curr->agent_id = orig_agent_id;  // Just to be clean
	return make_tuple(false, next_threshold);
}

// Returns (replan_success, constraint_added)
tuple<bool, bool> ICBSSearch::idcbsh_add_constraint_and_replan(ICBSNode *node, vector<vector<PathEntry> *> &the_paths,
                                                               vector<unordered_map<int, AvoidanceState >> &the_cat,
                                                               int allowed_cost_increase)
{
	auto [agent1_id, agent2_id, location1, location2, timestep] = *node->conflict;
	bool costMayIncrease = true;
	int oldG = node->g_val;

	int minNewCost;
	if (timestep >= (int)the_paths[node->agent_id]->size()) // Conflict happens after the agent reaches its goal.
		// Since there can only be vertex conflicts when the agent is WAITing
		// at its goal, the goal would only be reachable after the time of the new constraint
		minNewCost = timestep + 1;
	else if (the_paths[node->agent_id]->at(timestep).builtMDD == false) // Can't check if it's a singleton in the agent's MDD -
		// we only know the cost won't decrease
		minNewCost = (int)the_paths[node->agent_id]->size() - 1;
	else if (the_paths[node->agent_id]->at(timestep).single == false) {  // Not a singleton in the agent's MDD -
		// we only know the cost won't decrease
		minNewCost = (int) the_paths[node->agent_id]->size() - 1;
		// TODO: Consider setting maxNewCost = minNewCost and passing it to the low level
		costMayIncrease = false;
	}
	else if (location1 >= 0 && location2 < 0)  // It's a vertex constraint on a singleton in the agent's MDD -
											   // cardinal vertex - cost will increase by at least 1
		minNewCost = (int)the_paths[node->agent_id]->size();
	else if (the_paths[node->agent_id]->at(timestep - 1).builtMDD && the_paths[node->agent_id]->at(timestep - 1).single) // Edge constraint on an edge between two singletons in the agent's MDD
		minNewCost = (int)the_paths[node->agent_id]->size();
	else // Not a cardinal edge
		minNewCost = (int)the_paths[node->agent_id]->size() - 1;

	if (minNewCost - ((int)the_paths[node->agent_id]->size() - 1) > allowed_cost_increase)  // This check is instead of partial expansion
		return make_tuple(false, false);

	// The conflict-avoidance table already has the paths of all the agents
	removePathFromConflictAvoidanceTable(the_paths[node->agent_id], the_cat);

	// Constrain and replan
	if (location2 < 0 || node->agent_id == agent1_id)
		LL_num_generated += node->add_constraint(make_tuple(location1, location2, timestep, false), &the_cat, true);
	else
		LL_num_generated += node->add_constraint(make_tuple(location2, location1, timestep, false), &the_cat, true);

	bool replan_success = findPathForSingleAgent(node, the_paths, &the_cat, timestep, minNewCost, node->agent_id, true);

    // Restore the conflict-avoidance table to have the paths of all the agents
    addPathToConflictAvoidanceTable(the_paths[node->agent_id], the_cat);

	if (!costMayIncrease && node->g_val > oldG)
	{
		std::cout << "Cost increased for non-cardinal agent " << node->agent_id << "!?!?!?!" << std::endl;
		std::abort();
		return make_tuple(false, true);
	}

	return make_tuple(replan_success, true);
}

void ICBSSearch::idcbsh_unconstrain(ICBSNode *node, vector<vector<PathEntry> *> &the_paths,
                                    vector<unordered_map<int, AvoidanceState >> &the_cat,
                                    vector<PathEntry> &path_backup, shared_ptr<Conflict> &conflict_backup,
                                    int makespan_backup, int g_val_backup, int h_val_backup)
{
	// Remove the last constraint on the agent
	int agent_id = node->agent_id;
	std::vector<std::unordered_map<int, AvoidanceState>>* catp = nullptr;
#ifndef LPA
#else
    // The conflict-avoidance table already has the paths of all the agents
    removePathFromConflictAvoidanceTable(the_paths[agent_id], the_cat);
    catp = &the_cat;
#endif
	LL_num_generated += node->pop_constraint(catp);
#ifndef LPA
#else
    // Restore the conflict-avoidance table to have the paths of all the agents
    addPathToConflictAvoidanceTable(the_paths[agent_id], the_cat);
#endif

	// No need to ask LPA* to find the path again just to restore it, we saved a backup!
	the_paths[agent_id]->resize(path_backup.size());
	for (int i = 0 ; i < path_backup.size() ; ++i)
	{

		(*the_paths[agent_id])[i] = path_backup[i];
	}
	// Find the conflicts again, we didn't save a backup of them.
	// TODO: Consider doing that too.
	clearConflictsOfAgent(*node, agent_id);
	for (int a2 = 0; a2 < num_of_agents; a2++) {
		if (a2 != agent_id)
			findConflicts(the_paths, agent_id, a2, *node);
	}
	node->num_of_conflicts = (int) node->unknownConf.size() + (int) node->cardinalConf.size() +
							 (int) node->semiConf.size() + (int) node->nonConf.size();
	classifyConflicts(*node, the_paths); // classify and choose conflicts
										 // (the choice isn't guaranteed to be stable nor even deterministic,
										 // so I'm not assigning the result, I'm using a backup:
	node->conflict = conflict_backup;
	node->makespan = makespan_backup;
	node->g_val = g_val_backup;
	node->h_val = h_val_backup;
	node->f_val = node->g_val + node->h_val;
}

ICBSSearch::ICBSSearch(const MapLoader& ml, const AgentsLoader& al, double focal_w, split_strategy p, bool HL_h,
					   int cutoffTime, int screen):
	focal_w(focal_w), split(p), HL_heuristic(HL_h), screen(screen)
{
	// set timer
	std::clock_t start = std::clock();
	auto wall_start = std::chrono::system_clock::now();

	time_limit = cutoffTime;
	this->num_map_cols = ml.cols;
	num_of_agents = al.num_of_agents;
	map_size = ml.rows * ml.cols;
	moves_offset = ml.moves_offset;
	search_engines = vector < ICBSSingleAgentLLSearch* >(num_of_agents);

#ifndef LPA
#else
	vector<LPAStar*> lpas(num_of_agents);
#endif

	// initialize single agent search solver and heuristics for the low-level search
	for (int i = 0; i < num_of_agents; i++) 
	{
		int init_loc = ml.linearize_coordinate((al.initial_locations[i]).first, (al.initial_locations[i]).second);
		int goal_loc = ml.linearize_coordinate((al.goal_locations[i]).first, (al.goal_locations[i]).second);
		HeuristicCalculator heuristicCalculator(init_loc, goal_loc, ml.my_map, ml.rows, ml.cols, ml.moves_offset);
		search_engines[i] = new ICBSSingleAgentLLSearch(init_loc, goal_loc, ml.my_map, ml.rows, ml.cols, ml.moves_offset);
		heuristicCalculator.getHVals(search_engines[i]->my_heuristic);
#ifndef LPA
#else
		float* my_heuristic = new float[search_engines[i]->my_heuristic.size()];
		for (int j = 0; j < search_engines[i]->my_heuristic.size(); ++j) {
			my_heuristic[j] = search_engines[i]->my_heuristic[j];
		}
		lpas[i] = new LPAStar(init_loc, goal_loc, my_heuristic, &ml, i);
#endif
	}
	if (split != split_strategy::NON_DISJOINT) // Disjoint splitting uses differential heuristics (Manhattan Distance)
											   // for the low-level search in addition to the perfect heuristic
	{
		for (int i = 0; i < num_of_agents; i++)
		{
			search_engines[i]->differential_h.resize(num_of_agents);
			for (int j = 0; j < num_of_agents; j++)
				search_engines[i]->differential_h[j] = &search_engines[j]->my_heuristic;
		}
	}
	
	// initialize paths_found_initially
#ifndef LPA
	root_node = new ICBSNode(num_of_agents);
#else
	root_node = new ICBSNode(lpas);
#endif
	paths.resize(num_of_agents, NULL);
	paths_found_initially.resize(num_of_agents);
	std::vector < std::unordered_map<int, ConstraintState > > cons_table;
	for (int i = 0; i < num_of_agents; i++)
	{
		if (root_node->makespan + 1 > root_cat.size())
		{
			root_cat.resize(root_node->makespan + 1);
			buildConflictAvoidanceTable(paths, i, *root_node, root_cat);
		}
		else if (i > 0)
			addPathToConflictAvoidanceTable(paths[i-1], root_cat);

		clock_t ll_start = std::clock();
		auto wall_ll_start = std::chrono::system_clock::now();
		pair<int, int> start(search_engines[i]->start_location, 0);
		pair<int, int> goal(search_engines[i]->goal_location, INT_MAX);
#ifndef LPA
		bool success = search_engines[i]->findShortestPath(paths_found_initially[i], cons_table, root_cat, start, goal, 0, -1);
#else
		if (screen)
			cout << "Calling LPA* for the first time for agent " << i << endl;
		bool success = root_node->lpas[i]->findPath(root_cat, -1, -1);
#endif
		lowLevelTime += std::clock() - ll_start;
		wall_lowLevelTime += std::chrono::system_clock::now() - wall_ll_start;
		if (!success)
		{
			cout << "NO SOLUTION EXISTS FOR AGENT " << i;
			delete root_node;
			exit(-1);
		}
#ifndef LPA
#else
		const vector<int>* primitive_path = root_node->lpas[i]->getPath(1);  // 1 - first iteration path (0 is an empty path)
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
		paths[i] = &paths_found_initially[i];
		root_node->makespan = max(root_node->makespan, (int)paths_found_initially[i].size() - 1);
		LL_num_expanded += search_engines[i]->num_expanded;
		LL_num_generated += search_engines[i]->num_generated;
	}
	if (screen)
		this->printPaths(paths);

	// generate the root node and update data structures
	root_node->agent_id = -1;
	root_node->g_val = 0;
	for (int i = 0; i < num_of_agents; i++)
		root_node->g_val += (int) paths[i]->size() - 1;
	root_node->h_val = 0;
	root_node->f_val = root_node->g_val;
	root_node->depth = 0;
	root_node->open_handle = open_list.push(root_node);
	root_node->focal_handle = focal_list.push(root_node);
	HL_num_generated++;
	root_node->time_generated = HL_num_generated;
	allNodes_table.push_back(root_node);
	findConflicts(*root_node);
	root_node->num_of_conflicts = (int) root_node->unknownConf.size() + (int) root_node->cardinalConf.size() +
							 (int) root_node->semiConf.size() + (int) root_node->nonConf.size();
	min_f_val = root_node->f_val;
	focal_list_threshold = min_f_val * focal_w;

	prepTime = std::clock() - start;
	wall_prepTime = std::chrono::system_clock::now() - wall_start;
}

ICBSSearch::~ICBSSearch()
{
	for (size_t i = 0; i < search_engines.size(); i++)
		delete search_engines[i];

	for (list<ICBSNode*>::iterator it = allNodes_table.begin(); it != allNodes_table.end(); it++) {
		// TODO: free all the lpastar instances from all the nodes, but carefully, since they're shared
		delete *it;
	}
}
