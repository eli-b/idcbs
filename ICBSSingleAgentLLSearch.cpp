#include "ICBSSingleAgentLLSearch.h"
#include "conflict_avoidance_table.h"


// Put the path to the given goal node in the <path> vector
void ICBSSingleAgentLLSearch::updatePath(const ICBSSingleAgentLLNode* goal, Path &path)
{
	const ICBSSingleAgentLLNode* curr = goal;
	int old_length = (int)path.size();
	if (goal->timestep >= old_length)
		path.resize(goal->timestep + 1);

	// The start and goal are singletons:
	path[goal->timestep].builtMDD = true;
	path[goal->timestep].single = true;
	path[goal->timestep].numMDDNodes = 1;
	path[0].builtMDD = true;
	path[0].single = true;
	path[0].numMDDNodes = 1;

	for(int t = goal->timestep; curr != nullptr; t--)
	{
		path[t].location = curr->loc;
		path[t].builtMDD = false;
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

int ICBSSingleAgentLLSearch::getDifferentialHeuristic(int loc1, int loc2) const
{
	if (loc2 == goal_location)
		return my_heuristic[loc1];
	int h = 0, sub;
	for (int i = 0; i < differential_h.size(); i++)
	{
		sub = abs(differential_h[i][loc1] - differential_h[i][loc2]);
		if (sub > h)
			h = sub;
	}
	return h;
}

// iterate over the constraints ( cons[t] is a list of all constraints for timestep t) and return the latest
// timestep which has a constraint involving the goal location
// which is the minimal plan length for the agent
int ICBSSingleAgentLLSearch::extractLastGoalTimestep(int goal_location, const vector<list<ConstraintForKnownTimestep>>* cons)
{
	if (cons != NULL) 
	{
		for (int t = static_cast<int>(cons->size()) - 1; t > 0; t--) 
		{
			for (list<ConstraintForKnownTimestep>::const_iterator it = cons->at(t).begin(); it != cons->at(t).end(); ++it)
			{
			    auto [loc1, loc2, positive_constraint] = *it;
				if (loc1 == goal_location && loc2 < 0 && !positive_constraint) {
					return t;
				}
			}
		}
	}
	return -1;
}


// Checks if a move into next_id at next_timestepfrom direction is valid (wrt my_map and constraints)
// input: direction (into next_id) ; next_id (location at time next_timestep); next_timestep
// cons[timestep] is a list of <loc1,loc2, bool> of (vertex/edge) constraints for that timestep. (loc2=-1 for vertex constraint).
bool ICBSSingleAgentLLSearch::isConstrained(int direction, int next_id, int next_timestep,
                                            const XytHolder<ConstraintState>& cons_table)  const
{
	if (my_map[next_id]) // obstacles
		return true;
    int loc_id = next_id - moves_offset[direction];
    auto [found_curr, state_curr] = cons_table.get(loc_id, next_timestep - 1);
    if (found_curr && state_curr->vertex)
        return true;
    auto [found_next, state_next] = cons_table.get(next_id, next_timestep);
    if (!found_next)
        return false;
    // Check if there's a vertex constraint on next_id in the source and next timestep
    if (state_next->vertex)
        return true;

    // Check if there's an edge constraint on entering next_id in next_timestep
	return state_next->edge[direction];
}

// find a path from location start.first at timestep start.second to location goal.first at timestep goal.second
// that satisfies all constraints in cons_table
// while minimizing conflicts with paths in cat
// return true if a path found (and updates path) or false if no path exists
// Since we don't have to find the SHORTEST path, we only use FOCAL (without OPEN) here and prune all nodes 
// that cannot reach the goal at the given timestep
// This is used to re-plan a (sub-)path between two positive constraints.
bool ICBSSingleAgentLLSearch::findPath(Path &path,
                                       const XytHolder<ConstraintState>& cons_table,
                                       ConflictAvoidanceTable& cat,
                                       const pair<int, int> &start, const pair<int, int>&goal, lowlevel_heuristic h_type)
{
	num_expanded = 0;
	num_generated = 0;
	hashtable_t::iterator it;  // will be used for find()

	 // generate root and add it to the FOCAL list
	ICBSSingleAgentLLNode* root = new ICBSSingleAgentLLNode(start.first, 0, 
		getDifferentialHeuristic(start.first, goal.first), NULL, start.second, 0, false);
	num_generated++;
	root->focal_handle = focal_list.push(root);
	allNodes_table[root] = root;

	while (!focal_list.empty())
	{
		ICBSSingleAgentLLNode* curr = focal_list.top(); focal_list.pop();
		
		// check if the popped node is a goal
		if (curr->timestep == goal.second) 
		{
			if (curr->loc != goal.first)
				continue;
			updatePath(curr, path);
			focal_list.clear();
			releaseClosedListNodes(allNodes_table);
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
				if (h_type == lowlevel_heuristic::DEFAULT)
					next_h_val = abs(my_heuristic[goal.first] - my_heuristic[next_id]);
				else //h_type == lowlevel_heuristic::DH
					next_h_val = getDifferentialHeuristic(next_id, goal.first);
				if (next_timestep + next_h_val > goal.second) // the node cannot reach the goal node at time goal.second
					continue;
				int next_internal_conflicts = curr->num_internal_conf +
					cat.num_conflicts_for_step(curr->loc, next_id, next_timestep);

				// generate (maybe temporary) node
				auto next = new ICBSSingleAgentLLNode(next_id, next_g_val, next_h_val, curr,
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
					ICBSSingleAgentLLNode* existing_next = (*it).second;
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
	focal_list.clear();
	releaseClosedListNodes(allNodes_table);
	return false;
}


// find the shortest path from location start.first at timestep start.second to location goal.first
// (no earlier than timestep max{earliestGoalTimestep, lastGoalConsTime + 1} and no later than timestep goal.second)
// that satisfies all constraints in cons_table
// while minimizing conflicts with paths in cat and put it in the given <path> vector
// return true if a path was found (and update path) or false if no path exists
bool ICBSSingleAgentLLSearch::findShortestPath(Path &path,
                                               const XytHolder<ConstraintState>& cons_table,
                                               ConflictAvoidanceTable& cat,
                                               const pair<int, int> &start, const pair<int, int>&goal, int earliestGoalTimestep, int lastGoalConsTime)
{
	num_expanded = 0;
	num_generated = 0;
	hashtable_t::iterator it;  // will be used for find()

	 // generate start and add it to the OPEN list
	auto root = new ICBSSingleAgentLLNode(start.first, 0, my_heuristic[start.first], nullptr, start.second, 0, false);
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
		ICBSSingleAgentLLNode* curr = focal_list.top(); focal_list.pop();
		open_list.erase(curr->open_handle);
		curr->in_openlist = false;	

		// check if the popped node is a goal
		if (curr->loc == goal.first && curr->timestep > lastGoalConsTime)
		{
			updatePath(curr, path);
			open_list.clear();
			focal_list.clear();
			releaseClosedListNodes(allNodes_table);
			return true;
		}
		else if (curr->timestep > goal.second) // did not reach the goal location before the required timestep
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
					cat.num_conflicts_for_step(curr->loc, next_id, next_timestep);
				
				// generate (maybe temporary) node
				ICBSSingleAgentLLNode* next = new ICBSSingleAgentLLNode(next_id, next_g_val, next_h_val, curr, next_timestep, next_internal_conflicts, false);		
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
					delete next;  // not needed anymore -- we already generated it before
					ICBSSingleAgentLLNode* existing_next = (*it).second;
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
		
		
		if (open_list.empty())  // in case OPEN is empty, no path found...
			break;
		// update FOCAL if min f-val increased
		ICBSSingleAgentLLNode* open_head = open_list.top();
		if (open_head->getFVal() > min_f_val) 
		{
			int new_min_f_val = open_head->getFVal();
			int new_lower_bound = max(lower_bound,  new_min_f_val);
			for (ICBSSingleAgentLLNode* n : open_list) 
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
	open_list.clear();
	focal_list.clear();
	releaseClosedListNodes(allNodes_table);
	return false;
}

inline void ICBSSingleAgentLLSearch::releaseClosedListNodes(hashtable_t& allNodes_table)
{
	for (const auto& pair: allNodes_table)
	{
	    const auto& [node_k, node] = pair;
		delete node;
	}
    allNodes_table.clear();
}

ICBSSingleAgentLLSearch::ICBSSingleAgentLLSearch(int start_location, int goal_location,
	const bool* my_map, int num_row, int num_col, const int* moves_offset, int* my_heuristic):
		my_map(my_map), moves_offset(moves_offset), start_location(start_location), goal_location(goal_location),
		num_col(num_col), my_heuristic(my_heuristic)
{
	this->map_size = num_row * num_col;

	// initialize allNodes_table (hash table)
	empty_node = new ICBSSingleAgentLLNode();
	empty_node->loc = -1;
	deleted_node = new ICBSSingleAgentLLNode();
	deleted_node->loc = -2;
	allNodes_table.set_empty_key(empty_node);
	allNodes_table.set_deleted_key(deleted_node);
}


ICBSSingleAgentLLSearch::~ICBSSingleAgentLLSearch()
{
	delete empty_node;
	delete deleted_node;
	delete [] my_heuristic;
}
