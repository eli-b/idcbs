#pragma once
#include "ICBSSingleAgentLLNode.h"
#include "map_loader.h"
#include "conflict_avoidance_table.h"

class ICBSSingleAgentLLSearch
{
public:
	// define typedefs (will also be used in ecbs_search)
	// note -- handle typedefs is defined inside the class (hence, include node.h is not enough).
	typedef boost::heap::fibonacci_heap< ICBSSingleAgentLLNode*, boost::heap::compare<ICBSSingleAgentLLNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< ICBSSingleAgentLLNode*, boost::heap::compare<ICBSSingleAgentLLNode::secondary_compare_node> > heap_focal_t;
	typedef google::dense_hash_map<ICBSSingleAgentLLNode*, ICBSSingleAgentLLNode*, ICBSSingleAgentLLNode::NodeHasher, ICBSSingleAgentLLNode::eqnode> hashtable_t;
	heap_open_t open_list;
	heap_focal_t focal_list;
	hashtable_t allNodes_table;
	// used in hash table and would be deleted from the d'tor
	ICBSSingleAgentLLNode* empty_node;
	ICBSSingleAgentLLNode* deleted_node;

	int start_location;
	int goal_location;

	const bool* my_map;
	int map_size;
	const int* moves_offset;

	uint64_t num_expanded = 0;
	uint64_t num_generated = 0;

	int lower_bound;  // FOCAL's lower bound
	int min_f_val;  // min f-val seen so far
	int num_of_conf;
	int num_col;

	int* my_heuristic;  // this is the precomputed heuristic for this agent
	vector<int*> differential_h; // Used for differential heuristics

	 // path finding
	bool findPath(vector<PathEntry> &path,
		const std::vector < std::unordered_map<int, ConstraintState > >& cons_table, 
		ConflictAvoidanceTable& cat,
		const pair<int, int> &start, const pair<int, int>&goal, lowlevel_heuristic h_type);
	bool findShortestPath(vector<PathEntry> &path, 
		const std::vector < std::unordered_map<int, ConstraintState > >& cons_table,
		ConflictAvoidanceTable& cat,
		const pair<int, int> &start, const pair<int, int>&goal, int earliestGoalTimestep, int lastGoalConsTime);
	
	// tools
	int extractLastGoalTimestep(int goal_location, const vector< list< tuple<int, int, bool> > >* cons);
	bool isConstrained(int direction, int next_id, int next_timestep,
		const std::vector < std::unordered_map<int, ConstraintState > >& cons_table)  const;
	void updatePath(const ICBSSingleAgentLLNode* goal, vector<PathEntry> &path); 
	// A heuristic estimate of the difference between any two points,
	// computed as MIN(|H_i(loc1) - H_i(loc2)|) for every normal heuristic to goal i that we have.
	// We have one goal heuristic per agent.
	int getDifferentialHeuristic(int loc1, int loc2) const;
	inline void releaseClosedListNodes(hashtable_t& allNodes_table);


	ICBSSingleAgentLLSearch(int start_location, int goal_location, const bool* my_map, int map_row, int num_col, const int* moves_offset, int* my_heuristic);
	~ICBSSingleAgentLLSearch();

};
