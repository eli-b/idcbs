// ICBS Search (High-level)
#pragma once

#include "ICBSNode.h"
#include "ICBSSingleAgentLLSearch.h"
#include "heuristic_calculator.h"
#include "agents_loader.h"

class ICBSSearch
{
public:
	//settings
	conflict_type conflictType;
	split_strategy split;
	bool posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem = false;
	int screen = 0;
	bool HL_heuristic;
	double focal_w = 1.0;
	int time_limit;

	// statistics of efficiency
	clock_t runtime = 0; // not include preprocessing
	clock_t prepTime = 0; // CPU time for preprocessing
	uint64_t HL_num_expanded = 0;
	uint64_t HL_num_generated = 0;
	uint64_t LL_num_expanded = 0;
	uint64_t LL_num_generated = 0;
	uint64_t HL_num_reexpanded = 0;

	// statistics of solution quality
	bool solution_found = false;
	int solution_cost = -1;
	int min_f_val;
	double focal_list_threshold;

	
	// print
	void printPaths() const;
	void printResults() const;
	void saveResults(const string& outputFile, const string& agentFile, const string& solver) const;

	void isFeasible()  const;

	bool runICBSSearch();
	ICBSSearch(const MapLoader& ml, const AgentsLoader& al, double focal_w, split_strategy c, bool HL_h, int cutoffTime);
	~ICBSSearch();


private:
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::secondary_compare_node> > heap_focal_t;
	//typedef boost::heap::fibonacci_heap< MDDNode*, boost::heap::compare<MDDNode::compare_node> > mdd_open_t;
	heap_open_t open_list;
	heap_focal_t focal_list;
	list<ICBSNode*> allNodes_table;

	// input
	int map_size;
	int num_of_agents;
	const int* moves_offset;
	int num_col;


	ICBSNode* root_node;
	vector < ICBSSingleAgentLLSearch* > search_engines;  // used to find (single) agents' paths and mdd
	vector<vector<PathEntry>*> paths;
	vector<vector<PathEntry>> paths_found_initially;  // contain initial paths found

	// print
	void printConflicts(const ICBSNode &n) const;
	void printConstraints(const ICBSNode* n) const;

	//conflicts
	void findConflicts(ICBSNode& curr);
	void findConflicts(ICBSNode& curr, int a1, int a2);
	std::shared_ptr<Conflict> classifyConflicts(ICBSNode &parent);
	std::shared_ptr<Conflict> getHighestPriorityConflict(ICBSNode &node);
	std::shared_ptr<Conflict> getHighestPriorityConflict(const list<std::shared_ptr<Conflict>>& confs, 
		const vector<int>& metric, const vector<double>& widthMDD);
	void copyConflictsFromParent(ICBSNode& curr);
	void copyConflicts(const vector<bool>& unchanged,
		const list<std::shared_ptr<Conflict>>& from, list<std::shared_ptr<Conflict>>& to);

	// branch
	void branch(ICBSNode* curr, ICBSNode* n1, ICBSNode*n2);
	bool findPathForSingleAgent(ICBSNode*  node, int ag, int timestep, int earliestGoalTimestep = 0);
	bool generateChild(ICBSNode* child);
	bool findPaths(ICBSNode*  node);
	void buildConflictAvoidanceTable(std::vector < std::unordered_map<int, AvoidanceState > >& res_table,
		int exclude_agent, const ICBSNode &node);
	int  buildConstraintTable(ICBSNode* curr, int agent_id, int timestep, 
		std::vector < std::unordered_map<int, ConstraintState > >& cons_table, pair<int, int>& start, pair<int, int>& goal);
	void addPathToConflictAvoidanceTable(std::vector < std::unordered_map<int, AvoidanceState > >& cat, int ag);
	int countMddSingletons(int agent_id, int conflict_timestep);
	int getTotalMddWidth(int agent_id);
	void addPositiveConstraintsOnNarrowLevelsLeadingToPositiveConstraint(int agent_id, int timestep, ICBSNode* n1, ICBSNode* n2, const std::vector < std::unordered_map<int, AvoidanceState > >* cat);


	// update
	inline void updatePaths(ICBSNode* curr);
	void collectConstraints(ICBSNode* curr, std::list<pair<int, tuple<int, int, int, bool>>> &constraints);
	bool reinsert(ICBSNode* curr);
	void updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight);


	//high-level heuristics
	int computeHeuristics(const ICBSNode& curr);
	bool KVertexCover(const vector<vector<bool>>& CG, int num_of_CGnodes, int num_of_CGedges, int k);
	
	// tools
	void buildMDD(ICBSNode& curr, int ag, int timestep, int lookahead = 0);
	bool isPathsConsistentWithConstraints(ICBSNode* curr) const;
	inline int getAgentLocation(int agent_id, size_t timestep);		
};

