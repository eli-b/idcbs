#include "ICBSSearch.h"

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
	while (curr != root_node)
	{
		for (auto con : curr->constraints)
		{
			constraints.push_back(make_pair(curr->agent_id, con));
		}
		curr = curr->parent;
	}
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
	while (curr != root_node)
	{
		for (auto con: curr->constraints)
		{
			auto [loc1, loc2, constraint_timestep, positive_constraint] = con;
			if (positive_constraint) // positive constraints affect all agents
			{
				if (curr->agent_id == agent_id) // for the constrained agent, it is a landmark
				{
					if (loc2 < 0) // vertex constraint
					{
						if (start.second < constraint_timestep && constraint_timestep < timestep)  // This landmark is between (start.second, timestep)
						{ // update start
							start.first = loc1;
							start.second = constraint_timestep;
						}
						else if (timestep <= constraint_timestep && constraint_timestep < goal.second)  // the landmark is between [timestep, goal.second)
						{ // update goal
							goal.first = loc1;
							goal.second = constraint_timestep;
							// If the landmark is at the same timestep as the new constraint, planning the new path will surely fail
						}
					}
					else // edge constraint, viewed as two landmarks on the from-vertex and the to-vertex
					{
						if (start.second < constraint_timestep && constraint_timestep < timestep)  // the second landmark is between (start.second, timestep)
						{ // update start
							start.first = loc2;
							start.second = constraint_timestep;
						}
						else if (timestep <= constraint_timestep - 1 && constraint_timestep - 1 < goal.second)  // the first landmark is between [timestep, goal.second)
						{ // update goal
							goal.first = loc1;
							goal.second = constraint_timestep - 1;
						}
					}
					constraints_positive.push_back(con);
				}
				else // for the other agents, it is equivalent to a negative constraint
				{
					if (loc1 == goal.first && loc2 < 0 && lastGoalConsTimestep < constraint_timestep) {
						lastGoalConsTimestep = constraint_timestep;
					}
					constraints_negative.push_back(con);
				}
			}
			else {  // Then it's a negative constraint
				if (curr->agent_id >= num_of_agents)  // This is used for Disjoint3. The negative constraint is imposed on both a1 and a2.
				{
					int a1 = curr->agent_id / num_of_agents - 1;
					int a2 = curr->agent_id % num_of_agents;
					// FIXME: This is a little hacky.
					if (a1 == agent_id) {
						constraints_negative.push_back(con);
						if (loc1 == goal.first && loc2 < 0 && lastGoalConsTimestep < constraint_timestep)
							lastGoalConsTimestep = constraint_timestep;
					} else if (a2 == agent_id) {
						if (loc2 >= 0) // edge constraint
						{ // need to swap locations
							constraints_negative.push_back(make_tuple(loc2, loc1, constraint_timestep, false));
							// FIXME: Don't we need to conditionally update lastGoalConsTimestep here too?
						} else {
							constraints_negative.push_back(con);
							if (loc1 == goal.first && loc2 < 0 && lastGoalConsTimestep < constraint_timestep)
								lastGoalConsTimestep = constraint_timestep;
						}

					}
				} else if (curr->agent_id == agent_id) // negative constraints only affect the current agent
				{
					constraints_negative.push_back(con);
					if (loc1 == goal.first && loc2 < 0 && lastGoalConsTimestep < constraint_timestep)
						lastGoalConsTimestep = constraint_timestep;
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
void ICBSSearch::buildConflictAvoidanceTable(std::vector < std::unordered_map<int, AvoidanceState > >& cat,
	int exclude_agent, const ICBSNode &node)
{
	if (node.makespan == 0)
		return;
	for (int ag = 0; ag < num_of_agents; ag++)
	{
		if (ag != exclude_agent && paths[ag] != NULL)
		{
			addPathToConflictAvoidanceTable(cat, ag);
		}
	}
}

// add a path to cat
void ICBSSearch::addPathToConflictAvoidanceTable(std::vector < std::unordered_map<int, AvoidanceState > >& cat, int ag)
{
	cat[0][paths[ag]->at(0).location].vertex = true;
	for (size_t timestep = 1; timestep < cat.size(); timestep++)
	{
		if (timestep >= paths[ag]->size() && cat[timestep][paths[ag]->back().location].vertex < 255)
			cat[timestep][paths[ag]->back().location].vertex++;
		else
		{
			int to = paths[ag]->at(timestep).location;
			int from = paths[ag]->at(timestep - 1).location;
			cat[timestep][to].vertex = true;
			for (int i = 0; i < MapLoader::valid_moves_t::WAIT_MOVE; i++)
			{
				if (from - to == moves_offset[i] && cat[timestep][from].edge[i] < 255)
					cat[timestep][from].edge[i]++;
			}
		}
	}
}



//////////////////// CONFLICTS ///////////////////////////
// copy conflicts from the parent node
void ICBSSearch::copyConflictsFromParent(ICBSNode& curr)
{
	// Copy from parent
	vector<bool> unchanged(num_of_agents, true);
	for (list<pair<int, vector<PathEntry>>>::const_iterator it = curr.new_paths.begin(); it != curr.new_paths.end(); it++)
		unchanged[it->first] = false;
	copyConflicts(unchanged, curr.parent->cardinalConf, curr.cardinalConf);
	copyConflicts(unchanged, curr.parent->semiConf, curr.semiConf);
	copyConflicts(unchanged, curr.parent->nonConf, curr.nonConf);
	copyConflicts(unchanged, curr.parent->unknownConf, curr.unknownConf);
}

// copy conflicts from "from" to "to" if the paths of both agents remain unchanged
void ICBSSearch::copyConflicts(const vector<bool>& unchanged, 
	const list<std::shared_ptr<Conflict>>& from, list<std::shared_ptr<Conflict>>& to)
{
	for (auto conflict : from)
	{
		auto [agent1, agent2, loc1, loc2, timestep] = *conflict;
		if (unchanged[agent1] && unchanged[agent2])
			to.push_back(conflict);
	}
}

// find all conflict in the current solution
void ICBSSearch::findConflicts(ICBSNode& curr)
{
	if (curr.parent != nullptr) // not root node
	{
		// detect conflicts that occur on the new planed paths
		vector<bool> detected(num_of_agents, false);
		for (list<pair<int, vector<PathEntry>>>::const_iterator it = curr.new_paths.begin(); it != curr.new_paths.end(); it++)
		{
			int a1 = it->first;
			detected[a1] = true;
			for (int a2 = 0; a2 < num_of_agents; a2++)
			{
				if (detected[a2])
					continue;
				findConflicts(curr, a1, a2);
			}
		}
	}
	else // root node
	{ // detect conflicts among all paths
		for(int a1 = 0; a1 < num_of_agents ; a1++)
		{
			for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
			{
				findConflicts(curr, a1, a2);
			}
		}
	}
}

// find conflicts between paths of agents a1 and a2
void ICBSSearch::findConflicts(ICBSNode& curr, int a1, int a2)
{
	size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
	for (int timestep = 0; timestep < min_path_length; timestep++)
	{
		int loc1 = paths[a1]->at(timestep).location;
		int loc2 = paths[a2]->at(timestep).location;
		if (loc1 == loc2)
		{
			curr.unknownConf.push_back(std::shared_ptr<Conflict>(new Conflict(a1, a2, loc1, -1, timestep)));
		}
		else if (timestep < min_path_length - 1
			&& loc1 == paths[a2]->at(timestep + 1).location
			&& loc2 == paths[a1]->at(timestep + 1).location)
		{
			curr.unknownConf.push_back(std::shared_ptr<Conflict>(new Conflict(a1, a2, loc1, loc2, timestep + 1))); // edge conflict
		}
	}
	if (paths[a1]->size() != paths[a2]->size()) // check whether there are conflicts that occur after one agent reaches its goal
	{
		int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
		int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
		int loc1 = paths[a1_]->back().location;
		for (int timestep = (int)min_path_length; timestep < paths[a2_]->size(); timestep++)
		{
			int loc2 = paths[a2_]->at(timestep).location;
			if (loc1 == loc2)
			{ // It's at least a semi cardinal conflict
				curr.unknownConf.push_front(std::shared_ptr<Conflict>(new Conflict(a1_, a2_, loc1, -1, timestep))); 
			}
		}
	}
}

// classify conflicts into cardinal, semi-cardinal and non-cardinal
std::shared_ptr<Conflict> ICBSSearch::classifyConflicts(ICBSNode &parent)
{
	if (parent.cardinalConf.empty() && parent.semiConf.empty() && parent.nonConf.empty() && parent.unknownConf.empty())
		return NULL; // No conflict

	// Classify all conflicts in unknownConf
	while (!parent.unknownConf.empty())
	{
		std::shared_ptr<Conflict> con = parent.unknownConf.front();
		auto [agent1, agent2, loc1, loc2, timestep] = *con;
		parent.unknownConf.pop_front();

		bool cardinal1, cardinal2;
		if (loc2 >= 0) // Edge conflict
		{
			buildMDD(parent, agent1, timestep);
			buildMDD(parent, agent2, timestep);
			buildMDD(parent, agent1, timestep - 1);
			buildMDD(parent, agent2, timestep - 1);
			cardinal1 = paths[agent1]->at(timestep).single &&
				paths[agent1]->at(timestep - 1).single;
			cardinal2 = paths[agent2]->at(timestep).single &&
				paths[agent2]->at(timestep - 1).single;
		}
		else // vertex conflict
		{
			if (timestep >= paths[agent1]->size())
				cardinal1 = true;
			else
			{
				buildMDD(parent, agent1, timestep);
				cardinal1 = paths[agent1]->at(timestep).single;
			}
			if (timestep >= paths[agent2]->size())
				cardinal2 = true;
			else
			{
				buildMDD(parent, agent2, timestep);
				cardinal2 = paths[agent2]->at(timestep).single;
			}
		}
		if (cardinal1 && cardinal2)
		{
			if (!HL_heuristic)// found a cardinal conflict, return it immediately
			{
				conflictType = conflict_type::CARDINAL;
				return con;
			}
			parent.cardinalConf.push_back(con);
		}
		else if (cardinal1 || cardinal2)
		{
			parent.semiConf.push_back(con);
		}
		else
		{
			parent.nonConf.push_back(con);
		}
	}

	return getHighestPriorityConflict(parent);
}

// Primary priority - cardinal, semi-cardinal and non-cardinal
std::shared_ptr<Conflict> ICBSSearch::getHighestPriorityConflict(ICBSNode &node)
{
	vector<int> metric(num_of_agents, 0);
	vector<double> widthMDD(num_of_agents, 0);
	if (split == split_strategy::SINGLETONS)
	{
		for (int i = 0; i < num_of_agents; i++)
		{
			for (int j = 0; j< paths[i]->size(); j++)
				widthMDD[i] += paths[i]->at(j).numMDDNodes;
			widthMDD[i] /= paths[i]->size();
		}
	}
	if (!node.cardinalConf.empty())
	{
		conflictType = conflict_type::CARDINAL;
		return getHighestPriorityConflict(node.cardinalConf, metric, widthMDD);
	}
	else if (!node.semiConf.empty())
	{
		conflictType = conflict_type::SEMICARDINAL;
		return getHighestPriorityConflict(node.semiConf, metric, widthMDD);
	}
	else
	{
		conflictType = conflict_type::NONCARDINAL;
		return getHighestPriorityConflict(node.nonConf, metric, widthMDD);
	}

}

// Secondary priority 
std::shared_ptr<Conflict> ICBSSearch::getHighestPriorityConflict(const list<std::shared_ptr<Conflict>>& confs, 
	const vector<int>& metric, const vector<double>& widthMDD)
{
	std::shared_ptr<Conflict> choice = confs.front();
	
	if (split == split_strategy::SINGLETONS)
	{
		for (auto conf : confs)
		{
			auto [agent1, agent2, loc1, loc2, timestep] = *conf;
			auto [choice_agent1, choice_agent2, choice_loc1, choice_loc2, choice_timestep] = *choice;
			int metric1 = 0;
			int maxSingles = 0;
			double minWidth = 0;
			if (timestep < (int)paths[agent1]->size())
				for (int j = 0; j < timestep; j++)
					if (paths[agent1]->at(j).buildMDD && paths[agent1]->at(j).single)
						metric1++;
			int metric2 = 0;
			if (timestep < (int)paths[agent2]->size())
				for (int j = 0; j < timestep; j++)
					if (paths[agent2]->at(j).buildMDD && paths[agent2]->at(j).single)
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
		for (auto conf : confs)
		{
			auto [agent1, agent2, loc1, loc2, timestep] = *conf;
			auto [choice_agent1, choice_agent2, choice_loc1, choice_loc2, choice_timestep] = *choice;
			int w1, w2, w3, w4;
			if (paths[agent1]->size() <= timestep)
				w1 = 1;
			else
				w1 = paths[agent1]->at(timestep).numMDDNodes;
			if (paths[agent2]->size() <= timestep)
				w2 = 1;
			else
				w2 = paths[agent2]->at(timestep).numMDDNodes;
			if (paths[choice_agent1]->size() <= choice_timestep)
				w3 = 1;
			else
				w3 = paths[choice_agent1]->at(choice_timestep).numMDDNodes;
			if (paths[choice_agent2]->size() <= choice_timestep)
				w4 = 1;
			else
				w4 = paths[choice_agent2]->at(choice_timestep).numMDDNodes;
			if (w1 * w2 < w3 * w4)
				choice = conf;
			else if (w1 * w2 == w3 * w4)
			{
				int single1 = 0;
				int single2 = 0;
				int single3 = 0;
				int single4 = 0;
				if (timestep < (int)paths[agent1]->size())
					for (int j = 0; j < timestep; j++)
						if (paths[agent1]->at(j).buildMDD && paths[agent1]->at(j).single)
							single1++;
				if (timestep < (int)paths[agent2]->size())
					for (int j = 0; j < timestep; j++)
						if (paths[agent2]->at(j).buildMDD && paths[agent2]->at(j).single)
							single2++;
				if (choice_timestep < (int)paths[choice_agent1]->size())
					for (int j = 0; j < choice_timestep; j++)
						if (paths[choice_agent1]->at(j).buildMDD && paths[choice_agent1]->at(j).single)
							single1++;
				if (choice_timestep < (int)paths[choice_agent2]->size())
					for (int j = 0; j < choice_timestep; j++)
						if (paths[choice_agent2]->at(j).buildMDD && paths[choice_agent2]->at(j).single)
							single2++;
				if (max(single1, single2) > max(single3, single4))
					choice = conf;
			}
		}
	}
#ifndef LPA
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
	else // Latest - better for LPA*
	{
		int latest = -1;
		int i = 0;
		for (auto conf : confs)
		{
			auto [agent1, agent2, loc1, loc2, timestep] = *conf;
			if (timestep > latest) {
				choice = conf;
				latest = timestep;
			}
		}
	}
#endif
	return choice;

	

}

// build MDDs if needed
void ICBSSearch::buildMDD(ICBSNode& curr, int ag, int timestep, int lookahead)
{
	if (paths[ag]->at(timestep).buildMDD)
		return;

	MDD  mdd;
	pair<int, int> start = make_pair(search_engines[ag]->start_location, 0);
	pair<int, int> goal = make_pair(search_engines[ag]->goal_location, (int)paths[ag]->size() - 1);
	ICBSNode* it = &curr; // Back to where we get the path
	bool found = false;
	while (it->parent != NULL)
	{
		for (auto newPath : it->new_paths)
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
			it = it->parent;
	}
	std::vector < std::unordered_map<int, ConstraintState > > cons_table(it->makespan + 1);

	buildConstraintTable(it, ag, timestep, cons_table, start, goal);

	mdd.buildMDD(cons_table, start, goal, lookahead, *search_engines[ag]);

	for (int i = 0; i < mdd.levels.size(); i++)
	{
		paths[ag]->at(i + start.second).single = mdd.levels[i].size() == 1;
		paths[ag]->at(i + start.second).numMDDNodes = (int)mdd.levels[i].size();
		paths[ag]->at(i + start.second).buildMDD = true;
	}
}

int ICBSSearch::countMddSingletons(int agent_id, int conflict_timestep)
{
	int num_singletons = 0;
	if (conflict_timestep < (int)paths[agent_id]->size())
		for (int j = 0; j < conflict_timestep; j++)
		{
			if (paths[agent_id]->at(j).buildMDD && paths[agent_id]->at(j).single)
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
			if (!paths[agent_id]->at(i).buildMDD)  // No MDD for this level. When does this happen?
				break;
			else if (!paths[agent_id]->at(i).single)  // Not a 1-width MDD level.
				continue;
			else if (paths[agent_id]->at(i - 1).buildMDD && paths[agent_id]->at(i - 1).single) {  // Safe because i > 0
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
		buildConflictAvoidanceTable(cat, id, *curr);
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
		buildConflictAvoidanceTable(cat, id, *curr);
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
		buildConflictAvoidanceTable(cat, id, *curr);
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
		buildConflictAvoidanceTable(cat1, agent1_id, *curr);
		buildConflictAvoidanceTable(cat2, agent2_id, *curr);
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

// plan paths for a child node, returns whether a path was found
bool ICBSSearch::generateChild(ICBSNode*  node)
{
	int h = 0;
	for (auto con : node->constraints)
	{
		auto [loc1, loc2, timestep, positive_constraint] = con;
		if (positive_constraint)
		{
			for (int ag = 0; ag < num_of_agents; ag++)
			{
				if (ag == node->agent_id)
				{
					continue;
				}
				else if (loc2 < 0 && // vertex constraint
					getAgentLocation(ag, timestep) == loc1)
				{
					if (timestep > paths[ag]->size()) // partial expansion - postpone the path finding
					{
						pair<int, vector<PathEntry>> newPath;
						newPath.first = ag;
						newPath.second.resize(1);
						newPath.second.back().location = timestep;
						node->new_paths.push_back(newPath);
						h += timestep - (int)paths[ag]->size();
					}
					else if (!findPathForSingleAgent(node, ag, timestep, max((int)paths[ag]->size() - 1, timestep)))
					{
						delete (node);
						return false;
					}
				}
				else if (loc2 >= 0) //edge constraint
				{
					if (getAgentLocation(ag, timestep - 1) == loc2 && getAgentLocation(ag, timestep) == loc1) // move from "to" to "from"
					{
						if (!findPathForSingleAgent(node, ag, timestep, max((int)paths[ag]->size() - 1, timestep)))
						{
							delete (node);
							return false;
						}
					}
					else if (getAgentLocation(ag, timestep - 1) == loc1) // stay in location "from"
					{
						if (!findPathForSingleAgent(node, ag, timestep - 1, max((int)paths[ag]->size() - 1, timestep)))
						{
							delete (node);
							return false;
						}
					}
					else if (getAgentLocation(ag, timestep) == loc2) // stay in lcoation "to"
					{
						if (!findPathForSingleAgent(node, ag, timestep, max((int)paths[ag]->size() - 1, timestep)))
						{
							delete (node);
							return false;
						}
					}
				}
			}
		}
		else // negative constraint
		{
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
				if (loc2 < 0 && timestep > paths[a[i]]->size()) // delay the path finding
				{
					pair<int, vector<PathEntry>> newPath;
					newPath.first = a[i];
					newPath.second.resize(1);
					newPath.second.back().location = timestep;
					node->new_paths.push_back(newPath);
					h += timestep - (int)paths[a[i]]->size();
					continue;
				}
				int lowerbound;
				if (timestep >= (int)paths[a[i]]->size()) // conflict happens after agent reaches its goal (because constraints are on the same time as the conflict or ealier due to propagation)
					lowerbound = timestep + 1;
				else if (!paths[a[i]]->at(timestep).buildMDD) // unknown
					lowerbound = (int)paths[a[i]]->size() - 1;
				else if (!paths[a[i]]->at(timestep).single) // not cardinal
					lowerbound = (int)paths[a[i]]->size() - 1;
				else if (loc1 >= 0 && loc2 < 0) // cardinal vertex
					lowerbound = (int)paths[a[i]]->size();
				else if (paths[a[i]]->at(timestep - 1).buildMDD && paths[a[i]]->at(timestep - 1).single) // Cardinal edge
					lowerbound = (int)paths[a[i]]->size();
				else // Not cardinal edge
					lowerbound = (int)paths[a[i]]->size() - 1;

				if (!findPathForSingleAgent(node, a[i], timestep, lowerbound))
				{
					delete (node);
					return false;
				}
			}
		}
	}

	//mark partialExpansion
	for (auto path : node->new_paths)
	{
		if (path.second.size() <= 1)
		{
			node->partialExpansion = true;
			break;
		}
	}

	//Estimate h value
	if (h > 0)
		node->h_val = h;
	else
	{
		node->h_val = max(node->parent->f_val - node->g_val, 0);
		node->f_val = node->g_val + node->h_val;
	}
	node->f_val = node->g_val + node->h_val;


	copyConflictsFromParent(*node);
	if (!node->partialExpansion)
	{
		findConflicts(*node);
	}

	node->num_of_collisions = (int)node->unknownConf.size() + (int)node->cardinalConf.size() +
		(int)node->semiConf.size() + (int)node->nonConf.size();

	// update handles
	node->open_handle = open_list.push(node);
	HL_num_generated++;
	node->time_generated = HL_num_generated;
	if (node->f_val <= focal_list_threshold)
		node->focal_handle = focal_list.push(node);
	allNodes_table.push_back(node);

	if (screen) // check the solution
		isPathsConsistentWithConstraints(node);

	return true;
}

// plan a path for an agent in a child node
bool ICBSSearch::findPathForSingleAgent(ICBSNode* node, int ag, int timestep, int lowerbound)
{
	// extract all constraints on agent ag, and build constraint table
	ICBSNode* curr = node;
	pair<int,int> start(search_engines[ag]->start_location, 0), goal(search_engines[ag]->goal_location, INT_MAX);

	// build constraint table
	std::vector < std::unordered_map<int, ConstraintState > > cons_table(node->makespan + 1);
	int lastGoalConTimestep = buildConstraintTable(curr, ag, timestep, cons_table, start, goal);
	
	// build reservation table
	std::vector < std::unordered_map<int, AvoidanceState > > cat(node->makespan + 1);
	buildConflictAvoidanceTable(cat, ag, *node);
	
	// find a path w.r.t cons_vec (and prioritize by res_table).
	pair<int,vector<PathEntry>> newPath;
	newPath.first = ag;
	newPath.second = *paths[ag];
	bool foundSol;

	// TODO: Pass the timeout to the low level
	if (goal.second  >= paths[ag]->size())
	{
#ifndef LPA
		foundSol = search_engines[ag]->findShortestPath(newPath.second, cons_table, cat, start, goal, lowerbound, lastGoalConTimestep);
		LL_num_expanded += search_engines[ag]->num_expanded;
		LL_num_generated += search_engines[ag]->num_generated;
#else
		if (start.second == 0 && goal.second == INT_MAX &&
				node->lpas[ag] != NULL &&
				node->lpas[ag]->dcm.dyn_constraints_.size() <= newPath.second.size() + 1) {  // +1 instead of -1 on the unsigned other side. -1 because the vector holds entries also for +1 of every conflict's timestep
			if (screen)
				cout << "Calling LPA* again for agent " << ag << endl;
			int generated_before = node->lpas[ag]->allNodes_table.size();
			foundSol = node->lpas[ag]->findPath(cat, lowerbound, lastGoalConTimestep);
			if (foundSol) {
				const vector<int> *primitive_path = node->lpas[ag]->getPath(node->lpas[ag]->paths.size() - 1);
				newPath.second.resize(primitive_path->size());
				for (int j = 0; j < primitive_path->size(); ++j) {
					newPath.second[j].location = (*primitive_path)[j];
					newPath.second[j].buildMDD = false;
				}
				newPath.second[0].buildMDD = true;
				newPath.second[0].single = true;
				newPath.second[0].numMDDNodes = 1;
				newPath.second[primitive_path->size() - 1].buildMDD = true;
				newPath.second[primitive_path->size() - 1].single = true;
				newPath.second[primitive_path->size() - 1].numMDDNodes = 1;
			}
			LL_num_expanded += node->lpas[ag]->num_expanded[node->lpas[ag]->paths.size() - 1];
			LL_num_generated += node->lpas[ag]->allNodes_table.size() - generated_before;
		}
		else
		{
			if (screen)
				cout << "Calling normal A* for agent " << ag << " instead of LPA* because LPA* can't handle a changed "
						"start or a constraint after reaching the goal, and can't recover if it was unable in the past" << endl;
			if (node->lpas[ag] != NULL) {
				delete node->lpas[ag];  // We've just created this copy - it's unused anywhere else
				node->lpas[ag] = NULL;
			}
			foundSol = search_engines[ag]->findShortestPath(newPath.second, cons_table, cat, start, goal, lowerbound, lastGoalConTimestep);
			LL_num_expanded += search_engines[ag]->num_expanded;
			LL_num_generated += search_engines[ag]->num_generated;
		}
#endif
	}
	else
	{
		foundSol = search_engines[ag]->findPath(newPath.second, cons_table, cat, start, goal, lowlevel_hval::DH);
		LL_num_expanded += search_engines[ag]->num_expanded;
		LL_num_generated += search_engines[ag]->num_generated;
	}

	if (!foundSol)
		return false;
	
	// update new_paths, paths and g_val
	if (node->new_paths.empty())
	{
		node->new_paths.push_back(newPath);
		node->g_val = node->g_val - (int)paths[ag]->size() + (int)newPath.second.size();
		paths[ag] = &node->new_paths.back().second;
	}
	else
	{
		bool insert = false;
		for (list<pair<int, vector<PathEntry>>>::iterator it = node->new_paths.begin(); it != node->new_paths.end(); ++it)
		{
			if (it->first == newPath.first && it->second.size() <= 1)
			{
				it->second = newPath.second;
				node->g_val = node->g_val - (int)paths[ag]->size() + (int)newPath.second.size();
				paths[ag] = &it->second;
				insert = true;
				break;
			}
		}
		if (!insert)
		{
			node->new_paths.push_back(newPath);
			node->g_val = node->g_val - (int)paths[ag]->size() + (int)newPath.second.size();
			paths[ag] = &node->new_paths.back().second;
		}
	}

	node->makespan = max(node->makespan, (int)newPath.second.size() - 1);
	if (screen)
		this->printPaths();
	return true;
}

// plan paths that are not planed yet due to partial expansion
bool ICBSSearch::findPaths(ICBSNode*  node)
{
	for(auto p: node->new_paths)
	{
		if (p.second.size() <= 1)
		{
			if (!findPathForSingleAgent(node, p.first, p.second.back().location, p.second.back().location))
			{
				delete (node);
				return false;
			}
		}
	}
	node->h_val = max(node->parent->f_val - node->g_val, 0);
	node->f_val = node->g_val + node->h_val;
	findConflicts(*node);
	node->num_of_collisions = (int) node->unknownConf.size() + (int) node->cardinalConf.size() +
		(int) node->semiConf.size() + (int) node->nonConf.size();

	HL_num_reexpanded++;
	node->partialExpansion = false;
	return true;
}


//////////////////// TOOLS ///////////////////////////
// check whether the new planed path obeys the constraints -- for debug
bool ICBSSearch::isPathsConsistentWithConstraints(ICBSNode* curr) const
{
	if (curr->partialExpansion)
		return true;
	while (curr != NULL)
	{
		for (auto con : curr->constraints)
		{
			auto [loc1, loc2, timestep, positive_constraint] = con;
			if (positive_constraint)
			{
				for (int i = 0; i < num_of_agents; i++)
				{
					if (i == curr->agent_id)
					{
						if (loc2 < 0) // vertex constraint
						{
							if (timestep < paths[i]->size() && paths[i]->at(timestep).location != loc1)
							{
								std::cout << "Path " << i << " violates constraint " << con << std::endl;
								exit(1);
							}
						}
						else // edge constraint
						{
							if (timestep < paths[i]->size() &&
								(paths[i]->at(timestep - 1).location != loc1 ||
								 paths[i]->at(timestep).location != loc2))
							{
								std::cout << "Path " << i << " violates constraint " << con << std::endl;
								exit(1);
							}
						}
					}
					else  // The positive constraint is on a different agent - an implicit negative constraint for this agent
					{
						if (timestep >= paths[i]->size())
						{
							// Nothing to do
						}
						else if (loc2 < 0) // vertex constraint
						{
							if (paths[i]->at(timestep).location == loc1)
							{
								std::cout << "Path " << i << " violates constraint " << con << std::endl;
								exit(1);
							}
						}
						else // edge constraint
						{
							if ((paths[i]->at(timestep - 1).location == loc2 && paths[i]->at(timestep).location == loc1) ||
								 paths[i]->at(timestep - 1).location == loc1 ||
								 paths[i]->at(timestep).location == loc2)
							{
								std::cout << "Path " << i << " violates constraint " << con << std::endl;
								exit(1);
							}
						}
					}
				}
			}
			else // negative constraint
			{
				if (loc2 < 0) // vertex constraint
				{
					if (paths[curr->agent_id]->size() > timestep && paths[curr->agent_id]->at(timestep).location == loc1)
					{
						std::cout << "Path " << curr->agent_id << " violates constraint " << con << std::endl;
						exit(1);
					}
					else if (paths[curr->agent_id]->size() <= timestep && paths[curr->agent_id]->back().location == loc1)
					{
						std::cout << "Path " << curr->agent_id << " violates constraint " << con << std::endl;
						exit(1);
					}
				}
				else // edge constraint
				{
					if (paths[curr->agent_id]->at(timestep - 1).location == loc1 && paths[curr->agent_id]->at(timestep).location == loc2)
					{
						std::cout << "Path " << curr->agent_id << " violates constraint " << con << std::endl;
						exit(1);
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
				cout << "Collision!" << endl;
			int t_min = (int)min(paths[i]->size(), paths[j]->size());
			if (t_min == 0)
				continue;
			for (int t = 1; t < t_min; t++)
			{
				if (paths[i]->at(t).location == paths[j]->at(t).location)
					cout << "Collision!" << endl;
				else if (paths[i]->at(t).location == paths[j]->at(t - 1).location &&
					paths[i]->at(t - 1).location == paths[j]->at(t).location)
					cout << "Collision!" << endl;
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
					cout << "Collision!" << endl;
			}
		}
	}
	if (sum != solution_cost)
		cout << "Solution cost wrong!" << endl;
}

inline int ICBSSearch::getAgentLocation(int agent_id, size_t timestep) 
{
	// if last timestep > plan length, agent remains in its last location
	if (timestep >= paths[agent_id]->size())
		return paths[agent_id]->at(paths[agent_id]->size() - 1).location;
	// otherwise, return its location for that timestep
	return paths[agent_id]->at(timestep).location;
}

// takes the paths_found_initially and UPDATES all (constrained) paths found for agents from curr to start
inline void ICBSSearch::updatePaths(ICBSNode* curr)
{
	ICBSNode* orig = curr;
	for (int i = 0; i < num_of_agents; i++)
		paths[i] = &paths_found_initially[i];
	vector<bool> updated(num_of_agents, false);  // initialized for false
	while (curr->parent != NULL)
	{
		for (list<pair<int, vector<PathEntry>>>::iterator it = curr->new_paths.begin(); it != curr->new_paths.end(); it++)
		{
			if (!updated[it->first] && it->second.size() > 1)
			{
				paths[it->first] = &(it->second);
				updated[it->first] = true;
			}
		}
		curr = curr->parent;
	}
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
void ICBSSearch::printPaths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		std::cout << "Agent " << i << " (" << paths_found_initially[i].size() - 1 << " -->" <<
			paths[i]->size() - 1 << "): ";
		for (int t = 0; t < paths[i]->size(); t++)
			std::cout << "(" << paths[i]->at(t).location / num_col << "," << paths[i]->at(t).location % num_col << ")->";
		std::cout << std::endl;
	}
}

void ICBSSearch::printConflicts(const ICBSNode &curr) const
{
	for (auto conflict: curr.cardinalConf)
	{
		std::cout << "Cardinal " << (*conflict) << std::endl;
	}
	for (auto conflict : curr.semiConf)
	{
		std::cout << "Semi " << (*conflict) << std::endl;
	}
	for (auto conflict : curr.nonConf)
	{
		std::cout << "Non " << (*conflict) << std::endl;
	}
	for (auto conflict : curr.unknownConf)
	{
		std::cout << "Unknown " << (*conflict) << std::endl;
	}
}

void ICBSSearch::printConstraints(const ICBSNode* n) const
{
	const ICBSNode* curr = n;
	while (curr != root_node)
	{
		for(auto con: curr->constraints)
		{
			std::cout << curr->agent_id << ": " << con << std::endl;
		}
		curr = curr->parent;
	}
}

void ICBSSearch::printResults() const
{
	if (runtime > time_limit * CLOCKS_PER_SEC)  // timeout
	{
		std::cout << "        Timeout  ; ";
	}
	else if (focal_list.empty() && solution_cost < 0)
	{
		std::cout << "No solutions  ; ";
	}
	else
	{
		std::cout << "         Optimal ; ";
	}
	std::cout << solution_cost << "," <<
		min_f_val - root_node->g_val << "," <<
		root_node->g_val << "," << root_node->f_val << "," <<
		((float) prepTime) / CLOCKS_PER_SEC << "," <<
		HL_num_expanded << "," << HL_num_generated << "," <<
		LL_num_expanded << "," << LL_num_generated << "," <<
		((float) runtime) / CLOCKS_PER_SEC << "," <<  std::endl;
}

void ICBSSearch::saveResults(const string& outputFile, const string& agentFile, const string& solver) const
{
	ofstream stats;
	stats.open(outputFile, ios::app);
	stats << solution_cost << "," << 
		min_f_val - root_node->g_val << "," <<
		root_node->g_val << "," << root_node->f_val << "," <<
		((float) prepTime) / CLOCKS_PER_SEC << "," <<
		HL_num_expanded << "," << HL_num_generated << "," <<
		LL_num_expanded << "," << LL_num_generated << "," <<
		((float) runtime) / CLOCKS_PER_SEC << "," <<
		solver << "," << agentFile << "," << endl;
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
	std::clock_t start;
	start = std::clock();

	while (!focal_list.empty()) 
	{
		runtime = std::clock() - start;
		if (runtime > time_limit * CLOCKS_PER_SEC)  // timeout
		{
			break;
		}

		// pop the best node from FOCAL
		ICBSNode* curr = focal_list.top();
		focal_list.pop();
		open_list.erase(curr->open_handle);

		// get current solutions
		updatePaths(curr);

		if (curr->partialExpansion)
		{
			bool Sol = findPaths(curr);
			if (!Sol || reinsert(curr))
				continue;
		}
		if (!HL_heuristic) // No heuristics
		{
			curr->conflict = classifyConflicts(*curr); // choose conflict
			
			if ( screen == 1)
				printConflicts(*curr);
		}
		else if (curr->conflict == NULL) //CBSH, and h value has not been computed yet
		{					
			curr->conflict = classifyConflicts(*curr); // classify and choose conflicts

			if (screen == 1)
			{
				std::cout << std::endl << "****** Compute h for #" << curr->time_generated
					<< " with f= " << curr->g_val << "+" << curr->h_val << " (";
				for (int i = 0; i < num_of_agents; i++)
					std::cout << paths[i]->size() - 1 << ", ";
				std::cout << ") and #conflicts = " << curr->num_of_collisions << std::endl;
				printConflicts(*curr);
			}
				
			curr->h_val = computeHeuristics(*curr);
			curr->f_val = curr->g_val + curr->h_val;

			if (reinsert(curr))
			{	
				continue;
			}
		}

		if (curr->conflict == NULL) // Failed to find a conflict => no conflicts
		{  // found a solution (and finish the while look)
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
			success1 = generateChild(n1); // plan paths for n1
			paths = temp;
			success2 = generateChild(n2); // plan paths for n2

			if (screen == 1)
			{
				if (success1)
				{
					std::cout << "Generated #" << n1->time_generated
						<< " with cost " << n1->g_val
						<< " and " << n1->num_of_collisions << " conflicts " << std::endl;
				}
				else
				{
					std::cout << "No feasible solution for left child! " << std::endl;
				}
				if (success2)
				{
					std::cout << "Generated #" << n2->time_generated
						<< " with cost " << n2->g_val
						<< " and " << n2->num_of_collisions << " conflicts " << std::endl;
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
	printResults();
	return solution_found;
}

ICBSSearch::ICBSSearch(const MapLoader& ml, const AgentsLoader& al, double focal_w, split_strategy p, bool HL_h, int cutoffTime):
	focal_w(focal_w), split(p), HL_heuristic(HL_h)
{
	// set timer
	std::clock_t start;
	start = std::clock();

	time_limit = cutoffTime;
	this->num_col = ml.cols;
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
		lpas[i] = new LPAStar(init_loc, goal_loc, my_heuristic, &ml);
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
	root_node = new ICBSNode();
#else
	root_node = new ICBSNode(lpas);
#endif
	paths.resize(num_of_agents, NULL);
	paths_found_initially.resize(num_of_agents);
	std::vector < std::unordered_map<int, ConstraintState > > cons_table;
	std::vector < std::unordered_map<int, AvoidanceState > > cat(root_node->makespan + 1);
	for (int i = 0; i < num_of_agents; i++) 
	{
		pair<int, int> start(search_engines[i]->start_location, 0);
		pair<int, int> goal(search_engines[i]->goal_location, INT_MAX);
		if (root_node->makespan + 1 > cat.size())
		{
			cat.resize(root_node->makespan + 1);
			buildConflictAvoidanceTable(cat, i, *root_node);
		}
		else if (i > 0)
			addPathToConflictAvoidanceTable(cat, i - 1);

#ifndef LPA
		if (search_engines[i]->findShortestPath(paths_found_initially[i], cons_table, cat, start, goal, 0, -1) == false)
#else
		if (screen)
			cout << "Calling LPA* for the first time for agent " << i << endl;
		if (root_node->lpas[i]->findPath(cat, -1, -1) == false)
#endif
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
			paths_found_initially[i][j].buildMDD = false;
		}
		paths_found_initially[i][0].buildMDD = true;
		paths_found_initially[i][0].single = true;
		paths_found_initially[i][0].numMDDNodes = 1;
		paths_found_initially[i][primitive_path->size()-1].buildMDD = true;
		paths_found_initially[i][primitive_path->size()-1].single = true;
		paths_found_initially[i][primitive_path->size()-1].numMDDNodes = 1;
#endif
		paths[i] = &paths_found_initially[i];
		root_node->makespan = max(root_node->makespan, (int)paths_found_initially[i].size() - 1);
		LL_num_expanded += search_engines[i]->num_expanded;
		LL_num_generated += search_engines[i]->num_generated;
	}
	if (screen)
		this->printPaths();

	// generate dummy start and update data structures
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
	min_f_val = root_node->f_val;
	focal_list_threshold = min_f_val * focal_w;

	prepTime = std::clock() - start;
}

ICBSSearch::~ICBSSearch()
{
	for (size_t i = 0; i < search_engines.size(); i++)
		delete (search_engines[i]);

	for (list<ICBSNode*>::iterator it = allNodes_table.begin(); it != allNodes_table.end(); it++) {
		// TODO: free all the lpastar instances from all the nodes, but carefully, since they're shared
		delete *it;
	}
}
