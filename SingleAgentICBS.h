#pragma once
#include "LLNode.h"
#include "map_loader.h"

class SingleAgentICBS
{
public:
	// define typedefs (will also be used in ecbs_search)
	// note -- handle typedefs is defined inside the class (hence, include node.h is not enough).
	typedef boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::secondary_compare_node> > heap_focal_t;
	typedef google::dense_hash_map<LLNode*, LLNode*, LLNode::NodeHasher, LLNode::eqnode> hashtable_t;
	heap_open_t open_list;
	heap_focal_t focal_list;
	hashtable_t allNodes_table;
	// used in hash table and would be deleted from the d'tor
	LLNode* empty_node;
	LLNode* deleted_node;

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

	vector<int> my_heuristic;  // this is the precomputed heuristic for this agent
	vector<vector<int>*> differential_h; //Used for differential heuristics

	 // path finding
	bool findPath(vector<PathEntry> &path,
		const std::vector < std::unordered_map<int, ConstraintState > >& cons_table, 
		const std::vector < std::unordered_map<int, ConstraintState > >& cat,
		const pair<int, int> &start, const pair<int, int>&goal, lowlevel_hval h_type);
	bool findShortestPath(vector<PathEntry> &path, 
		const std::vector < std::unordered_map<int, ConstraintState > >& cons_table,
		const std::vector < std::unordered_map<int, ConstraintState > >& cat,
		const pair<int, int> &start, const pair<int, int>&goal, int earliestGoalTimestep, int lastGoalConsTime);
	
	// tools
	int extractLastGoalTimestep(int goal_location, const vector< list< tuple<int, int, bool> > >* cons);
	bool isConstrained(int direction, int next_id, int next_timestep,
		const std::vector < std::unordered_map<int, ConstraintState > >& cons_table)  const;
	void updatePath(const LLNode* goal, vector<PathEntry> &path); 
	int numOfConflictsForStep(int curr_id, int next_id, int next_timestep, 
		const std::vector < std::unordered_map<int, ConstraintState > >& cat);
	int getDifferentialHeuristic(int loc1, int loc2) const;
	inline void releaseClosedListNodes(hashtable_t& allNodes_table);


	SingleAgentICBS(int start_location, int goal_location, const bool* my_map, int map_row, int num_col, const int* moves_offset);
	~SingleAgentICBS();

};

