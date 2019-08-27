// ICBS Search (High-level)
#pragma once

#include <chrono>

#include "ICBSNode.h"
#include "ICBSSingleAgentLLSearch.h"
#include "heuristic_calculator.h"
#include "agents_loader.h"

class ICBSSearch
{
public:
	//settings
	split_strategy split;
	bool posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem = false;
	int screen = 0;
	bool HL_heuristic;
	double focal_w = 1.0;
	int time_limit;

	// Used to ease tracking of the order of nodes in iterative deepening runs
	uint64_t HL_num_generated_before_this_iteration = 0;
	uint64_t HL_num_expanded_before_this_iteration = 0;

	// statistics of efficiency
	clock_t runtime = 0; // CPU time, excluding preprocessing
	clock_t prepTime = 0; // CPU time for preprocessing
	clock_t lowLevelTime = 0; // CPU time of running the low level
	clock_t highLevelTime = 0; // CPU time of running the high level
    clock_t highLevelMddBuildingTime = 0; // CPU time of building MDDs in the high level
	std::chrono::nanoseconds wall_runtime = std::chrono::nanoseconds::zero(); // Wall time, excluding preprocessing
	std::chrono::nanoseconds wall_prepTime = std::chrono::nanoseconds::zero(); // Wall time for preprocessing
    std::chrono::nanoseconds wall_mddTime = std::chrono::nanoseconds::zero(); // Wall time for building MDDs
	std::chrono::nanoseconds wall_lowLevelTime = std::chrono::nanoseconds::zero(); // Wall time of running the low level
	std::chrono::nanoseconds wall_highLevelTime = std::chrono::nanoseconds::zero(); // Wall time for running the high level
	uint64_t HL_num_expanded = 0;
	uint64_t HL_num_generated = 0;
	uint64_t LL_num_expanded = 0;
	uint64_t LL_num_generated = 0;
	uint64_t HL_num_reexpanded = 0;
	string max_mem;

	// statistics of solution quality
	bool solution_found = false;
	int solution_cost = -1;
	int min_f_val;
	double focal_list_threshold;

	// For debugging
	conflict_type conflictType;
	
	// print
	void printPaths(vector<vector<PathEntry> *> &the_paths) const;
	void printResults() const;
	void saveResults(const string& outputFile, const string& agentFile, const string& solver) const;

	void isFeasible()  const;

	bool runICBSSearch();
	bool runIterativeDeepeningICBSSearch();
	ICBSSearch(const MapLoader& ml, const AgentsLoader& al, double focal_w, split_strategy c, bool HL_h, int cutoffTime,
	           int screen = 0);
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
	int num_map_cols;

	std::vector < std::unordered_map<int, AvoidanceState > > root_cat;
	ICBSNode* root_node;
	vector < ICBSSingleAgentLLSearch* > search_engines;  // used to find (single) agents' paths and mdd
	vector<vector<PathEntry>*> paths;  // The paths of the node we're currently working on.
	                                   // This trades the speed of rebuilding it each time we move to a new node for
	                                   // the space of saving it on every node.
	                                   // For best-first-search CBS, this is also the set of paths of the best node in OPEN.
	vector<vector<PathEntry>> paths_found_initially;  // contains the initial path that was found for each agent

	// print
	void printConflicts(const ICBSNode &n) const;
	void printConstraints(const ICBSNode* n) const;

	//conflicts
	void findConflicts(ICBSNode& curr);
	void findConflicts(vector<vector<PathEntry> *> &the_paths, int a1, int a2, ICBSNode &curr);
	std::shared_ptr<Conflict> classifyConflicts(ICBSNode &node, vector<vector<PathEntry> *> &the_paths);
	std::shared_ptr<Conflict> getHighestPriorityConflict(ICBSNode &node, vector<vector<PathEntry> *> &the_paths);
	std::shared_ptr<Conflict> getHighestPriorityConflict(const list<std::shared_ptr<Conflict>> &confs,
                                                         vector<vector<PathEntry> *> &the_paths,
                                                         const vector<double> &widthMDD,
                                                         const vector<int> &metric);
	void copyConflictsFromParent(ICBSNode& curr);
	void clearConflictsOfAgent(ICBSNode &curr, int ag);
	void copyConflicts(const vector<bool>& unchanged,
		const list<std::shared_ptr<Conflict>>& from, list<std::shared_ptr<Conflict>>& to);
	void clearConflictsOfAffectedAgents(bool *unchanged,
                                        list<std::shared_ptr<Conflict>> &lst);

	// branch
	void branch(ICBSNode* curr, ICBSNode* n1, ICBSNode*n2);
	bool findPathForSingleAgent(ICBSNode *node, vector<vector<PathEntry> *> &the_paths,
	                            vector<unordered_map<int, AvoidanceState >> *the_cat,
	                            int timestep, int earliestGoalTimestep, int ag);
	bool generateChild(ICBSNode *node, vector<vector<PathEntry> *> &the_paths);
	bool finishPartialExpansion(ICBSNode *node, vector<vector<PathEntry> *> &the_paths);
	void buildConflictAvoidanceTable(vector<vector<PathEntry> *> &the_paths, int exclude_agent, const ICBSNode &node,
                                     std::vector<std::unordered_map<int, AvoidanceState> > &cat);
	int  buildConstraintTable(ICBSNode* curr, int agent_id, int timestep, 
		std::vector < std::unordered_map<int, ConstraintState > >& cons_table, pair<int, int>& start, pair<int, int>& goal);
	void addPathToConflictAvoidanceTable(vector<PathEntry> *path,
	                                     std::vector<std::unordered_map<int, AvoidanceState> > &cat);
	void removePathFromConflictAvoidanceTable(vector<PathEntry> *path,
	                                          std::vector<std::unordered_map<int, AvoidanceState> > &cat);
	int countMddSingletons(int agent_id, int conflict_timestep);
	int getTotalMddWidth(int agent_id);
	void addPositiveConstraintsOnNarrowLevelsLeadingToPositiveConstraint(int agent_id, int timestep, ICBSNode* n1, ICBSNode* n2, const std::vector < std::unordered_map<int, AvoidanceState > >* cat);


	// update
	inline void populatePaths(ICBSNode *curr, vector<vector<PathEntry> *> &the_paths);
	void collectConstraints(ICBSNode* curr, std::list<pair<int, tuple<int, int, int, bool>>> &constraints);
	bool reinsert(ICBSNode* curr);
	void updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight);


	// high-level heuristics
	int computeHeuristics(const ICBSNode& curr);
	bool KVertexCover(const vector<vector<bool>>& CG, int num_of_CGnodes, int num_of_CGedges, int k);
	
	// tools
    void buildMDD(ICBSNode &curr, vector<vector<PathEntry> *> &the_paths, int ag, int timestep, int lookahead = 0);
	bool arePathsConsistentWithConstraints(vector<vector<PathEntry> *> &the_paths, ICBSNode *curr) const;
	inline int getAgentLocation(vector<vector<PathEntry> *> &the_paths, size_t timestep, int agent_id);

	std::tuple<bool, int> do_idcbsh_iteration(ICBSNode *curr, vector<vector<PathEntry> *> &the_paths,
	                                          vector<unordered_map<int, AvoidanceState >> &the_cat,
	                                          int threshold, int next_threshold, clock_t end_by);
	tuple<bool, bool> idcbsh_add_constraint_and_replan(ICBSNode *node, vector<vector<PathEntry> *> &the_paths,
	                                                   vector<unordered_map<int, AvoidanceState >> &the_cat, int max_cost);
	void idcbsh_unconstrain(ICBSNode *node, vector<vector<PathEntry> *> &the_paths,
	                        vector<unordered_map<int, AvoidanceState >> &the_cat,
	                        vector<PathEntry> &path_backup,
	                        shared_ptr<Conflict> &conflict_backup, int makespan_backup, int g_val_backup,
	                        int h_val_backup);
};

