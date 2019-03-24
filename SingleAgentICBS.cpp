#include "SingleAgentICBS.h"


// Updates the path.
void SingleAgentICBS::updatePath(const LLNode* goal, vector<PathEntry> &path)
{
	const LLNode* curr = goal;
	int old_length = (int)path.size();
	if(goal->timestep >= old_length)
		path.resize(goal->timestep + 1);
	for(int t = goal->timestep;curr != NULL; t--)
	{
		path[t].location = curr->loc;
		if (t == goal->timestep || curr->parent == NULL) // The start and goal are singletons.
		{
			path[t].buildMDD = true;
			path[t].single = true;
			path[t].numMDDNodes = 1;
		}
		else
			path[t].buildMDD = false;
		curr = curr->parent;
	}
	if (path.size() > old_length)
	{
		int i = old_length;
		while (path[i].location == -1)
		{
			path[i] = path[old_length - 1];
			i++;
		}
	}
}

int SingleAgentICBS::getDifferentialHeuristic(int loc1, int loc2) const
{
	if (loc2 == goal_location)
		return my_heuristic[loc1];
	int h = 0, sub;
	for (int i = 0; i < differential_h.size(); i++)
	{
		sub = abs(differential_h[i]->at(loc1) - differential_h[i]->at(loc2));
		if(sub > h)
			h = sub;
	}
	return h;
}

// iterate over the constraints ( cons[t] is a list of all constraints for timestep t) and return the latest
// timestep which has a constraint involving the goal location
// which is the minimal plan length for the agent
int SingleAgentICBS::extractLastGoalTimestep(int goal_location, const vector< list< tuple<int, int, bool> > >* cons) 
{
	if (cons != NULL) 
	{
		for (int t = static_cast<int>(cons->size()) - 1; t > 0; t--) 
		{
			for (list< tuple<int, int, bool> >::const_iterator it = cons->at(t).begin(); it != cons->at(t).end(); ++it) 
			{
				if (get<0>(*it) == goal_location && get<1>(*it) < 0 && !get<2>(*it)) {
					return (t);
				}
			}
		}
	}
	return -1;
}


// Checks if a vaild path found (wrt my_map and constraints)
// input: curr_id (location at time next_timestep-1) ; next_id (location at time next_timestep); next_timestep
// cons[timestep] is a list of <loc1,loc2, bool> of (vertex/edge) constraints for that timestep. (loc2=-1 for vertex constraint).
bool SingleAgentICBS::isConstrained(int direction, int next_id, int next_timestep,
	const std::vector < std::unordered_map<int, ConstraintState > >& cons_table)  const 
{
	if (my_map[next_id]) // obstacles
		return true;
	else if(next_timestep >= cons_table.size())
		return false;
	auto it = cons_table[next_timestep].find(next_id);
	if (it == cons_table[next_timestep].end())
		return false;
	else if(it->second.vertex || it->second.edge[direction])
		return true;
	else 
		return false;
}

// Return the number of conflicts between the known_paths' (by looking at the reservation table) for the move [curr_id,next_id].
// Returns 0 if no conflict, 1 for vertex or edge conflict, 2 for both.
int SingleAgentICBS::numOfConflictsForStep(int curr_id, int next_id, int next_timestep, const std::vector < std::unordered_map<int, ConstraintState > >& cat )
{
	int retVal = 0;
	if (next_timestep >= cat.size()) 
	{
		// check vertex constraints (being at an agent's goal when he stays there because he is done planning)
		auto it = cat.back().find(next_id);
		if (it != cat.back().end() && it->second.vertex)
			retVal++;
		// Note -- there cannot be edge conflicts when other agents are done moving
	}
	else 
	{
		// check vertex constraints (being in next_id at next_timestep is disallowed)
		auto it = cat[next_timestep].find(next_id);
		if (it != cat[next_timestep].end())
		{
			if(it->second.vertex)
				retVal++;
			// check edge constraints (the move from curr_id to next_id at next_timestep-1 is disallowed)
			for (int i = 0; i < MapLoader::WAIT_MOVE; i++) // i=0 is the wait action that cannot lead to edge conflict
			{
				if (next_id - curr_id == moves_offset[i] && it->second.edge[i])
					retVal++;
			}
		}
	}
	return retVal;
}



// find a path from lcoation start.first at timestep start.second to location goal.first at timestep goal.second
// that satisfies all constraints in cons_table
// while minimizing conflicts with paths in cat
// return true if a path found (and updates path) or false if no path exists
// Since we don't have to find the SHORTEST path, we only use FOCAL (without OPEN) here and prune all nodes 
// that cannot reach the goal at the given timestep
bool SingleAgentICBS::findPath(vector<PathEntry> &path, 
	const std::vector < std::unordered_map<int, ConstraintState > >& cons_table,
	const std::vector < std::unordered_map<int, ConstraintState > >& cat,
	const pair<int, int> &start, const pair<int, int>&goal, lowlevel_hval h_type)
{
	num_expanded = 0;
	num_generated = 0;
	hashtable_t::iterator it;  // will be used for find()

	 // generate root and add it to the FOCAL list
	LLNode* root = new LLNode(start.first, 0, 
		getDifferentialHeuristic(start.first, goal.first), NULL, start.second, 0, false);
	num_generated++;
	root->focal_handle = focal_list.push(root);
	allNodes_table[root] = root;

	while (!focal_list.empty())
	{
		LLNode* curr = focal_list.top(); focal_list.pop();
		
		// check if the popped node is a goal
		if (curr->timestep == goal.second) 
		{
			if (curr->loc != goal.first)
				continue;
			updatePath(curr, path);
			releaseClosedListNodes(allNodes_table);
			focal_list.clear();
			allNodes_table.clear();
			return true;
		}
		
		num_expanded++;
		for (int i = 0; i < 5; i++)
		{
			int next_id = curr->loc + moves_offset[i];
			int next_timestep = curr->timestep + 1;

			if (0 <= next_id && next_id < map_size && 
				abs(next_id % num_col - curr->loc % num_col) < 2 && 
				!isConstrained(i, next_id, next_timestep, cons_table))
			{
				// compute cost to next_id via curr node
				int next_g_val = curr->g_val + 1;
				int next_h_val;
				if(h_type == lowlevel_hval::DEFAULT)
					next_h_val = abs(my_heuristic[goal.first] - my_heuristic[next_id]);
				else //h_type == lowlevel_hval::DH
					next_h_val = getDifferentialHeuristic(next_id, goal.first);
				if(next_timestep + next_h_val > goal.second) // the node cannot reach the goal node at time goal.second
					continue;
				int next_internal_conflicts = curr->num_internal_conf +
					numOfConflictsForStep(curr->loc, next_id, next_timestep, cat);

				// generate (maybe temporary) node
				LLNode* next = new LLNode(next_id, next_g_val, next_h_val,	curr, 
					next_timestep, next_internal_conflicts, false);		
				it = allNodes_table.find(next);// try to retrieve it from the hash table
				if (it == allNodes_table.end())
				{
					num_generated++;
					next->focal_handle = focal_list.push(next);
					allNodes_table[next] = next;
				}
				else 
				{  // update existing node's if needed
					delete(next);  // not needed anymore -- we already generated it before
					LLNode* existing_next = (*it).second;
					if (existing_next->num_internal_conf > next_internal_conflicts) // if there's fewer internal conflicts
					{
						// update existing node
						existing_next->parent = curr;
						existing_next->num_internal_conf = next_internal_conflicts;
					}
				} 
			}
		}  // end for loop that generates successors
	}  // end while loop
	 
	 // no path found
	releaseClosedListNodes(allNodes_table);
	focal_list.clear();
	allNodes_table.clear();
	return false;
}


// find the shortest path from lcoation start.first at timestep start.second to location goal.first
// (no earlier than timestep max{earliestGoalTimestep, lastGoalConsTime + 1} and no later than timestep goal.second)
// that satisfies all constraints in cons_table
// while minimizing conflicts with paths in cat
// return true if a path found (and updates path) or false if no path exists
bool SingleAgentICBS::findShortestPath(vector<PathEntry> &path,
	const std::vector < std::unordered_map<int, ConstraintState > >& cons_table,
	const std::vector < std::unordered_map<int, ConstraintState > >& cat,
	const pair<int, int> &start, const pair<int, int>&goal, int earliestGoalTimestep, int lastGoalConsTime)
{
	num_expanded = 0;
	num_generated = 0;
	hashtable_t::iterator it;  // will be used for find()

	 // generate start and add it to the OPEN list
	LLNode* root = new LLNode(start.first, 0, my_heuristic[start.first], NULL, start.second, 0, false);
	num_generated++;
	root->open_handle = open_list.push(root);
	root->focal_handle = focal_list.push(root);
	root->in_openlist = true;
	allNodes_table[root] = root;
	min_f_val = root->getFVal();
	lower_bound = max(min_f_val,
		(max(earliestGoalTimestep, lastGoalConsTime + 1) - start.second));


	while (!focal_list.empty()) 
	{
		LLNode* curr = focal_list.top(); focal_list.pop();
		open_list.erase(curr->open_handle);
		curr->in_openlist = false;	

		// check if the popped node is a goal
		if (curr->loc == goal.first && curr->timestep > lastGoalConsTime)
		{
			updatePath(curr, path);
			releaseClosedListNodes(allNodes_table);
			open_list.clear();
			focal_list.clear();
			allNodes_table.clear();
			return true;
		}
		else if(curr->timestep > goal.second) // did not reach the goal lcoation before the required timestep
			continue;
		num_expanded++;
		for (int i = 0; i < 5; i++)
		{
			int next_id = curr->loc + moves_offset[i];
			int next_timestep = curr->timestep + 1;
			if (0 <= next_id && next_id < map_size && 
				abs(next_id % num_col - curr->loc % num_col) < 2 &&
				!isConstrained(i, next_id, next_timestep, cons_table))
			{	
				// compute cost to next_id via curr node
				int next_g_val = curr->g_val + 1;
				int next_h_val = my_heuristic[next_id];
				int next_internal_conflicts = curr->num_internal_conf + 
					numOfConflictsForStep(curr->loc, next_id, next_timestep, cat);
				
				// generate (maybe temporary) node
				LLNode* next = new LLNode(next_id, next_g_val, next_h_val, curr, next_timestep, next_internal_conflicts, false);		
				it = allNodes_table.find(next); // try to retrieve it from the hash table
				if (it == allNodes_table.end()) 
				{
					next->open_handle = open_list.push(next);
					next->in_openlist = true;
					num_generated++;
					if (next->getFVal() <= lower_bound)
						next->focal_handle = focal_list.push(next);
					allNodes_table[next] = next;
				}
				else
				{  // update existing node's if needed (only in the open_list)
					delete(next);  // not needed anymore -- we already generated it before
					LLNode* existing_next = (*it).second;
					if (existing_next->in_openlist == true) // if its in the open list
					{ 
						if (existing_next->num_internal_conf > next_internal_conflicts)// if there's less internal conflicts
						{
							// update existing node
							existing_next->parent = curr;
							existing_next->num_internal_conf = next_internal_conflicts;
							if (existing_next->getFVal() <= lower_bound) // the existing node is in FOCAL
							{
								focal_list.update(existing_next->focal_handle); 
							}
						}
					}
				}  // end update an existing node
			}
		}  // end for loop that generates successors
		
		
		if (open_list.size() == 0)  // in case OPEN is empty, no path found...
			break;
		// update FOCAL if min f-val increased
		LLNode* open_head = open_list.top();
		if (open_head->getFVal() > min_f_val) 
		{
			int new_min_f_val = open_head->getFVal();
			int new_lower_bound = max(lower_bound,  new_min_f_val);
			for (LLNode* n : open_list) 
			{
				if (n->getFVal() > lower_bound && n->getFVal() <= new_lower_bound) 
				{
					n->focal_handle = focal_list.push(n);
				}
			}
			min_f_val = new_min_f_val;
			lower_bound = new_lower_bound;
		}
	}  // end while loop
	
	// no path found
	releaseClosedListNodes(allNodes_table);
	open_list.clear();
	focal_list.clear();
	allNodes_table.clear();
	return false;
}

inline void SingleAgentICBS::releaseClosedListNodes(hashtable_t& allNodes_table) 
{
	hashtable_t::iterator it;
	for (auto it: allNodes_table) 
	{
		delete it.second;
	}
}

SingleAgentICBS::SingleAgentICBS(int start_location, int goal_location,
	const bool* my_map, int num_row, int num_col, const int* moves_offset):
		my_map(my_map), moves_offset(moves_offset), start_location(start_location), goal_location(goal_location),
		num_col(num_col)
{
	this->map_size = num_row * num_col;

	// initialize allNodes_table (hash table)
	empty_node = new LLNode();
	empty_node->loc = -1;
	deleted_node = new LLNode();
	deleted_node->loc = -2;
	allNodes_table.set_empty_key(empty_node);
	allNodes_table.set_deleted_key(deleted_node);
}


SingleAgentICBS::~SingleAgentICBS()
{
	delete (empty_node);
	delete (deleted_node);
}
