// ICBS Search (High-level)
#pragma once

#define USE_GUROBI
//#define GUROBI_WITH_GOAL_CONSTRAINTS

#include <chrono>
#ifdef USE_GUROBI
#include <gurobi_c++.h>
#endif

#include "ICBSNode.h"
#include "ICBSSingleAgentLLSearch.h"
#include "heuristic_calculator.h"
#include "agents_loader.h"
#include "conflict_avoidance_table.h"

class ICBSSearch
{
public:
	//settings
	split_strategy split;
	bool posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem = false;
	bool preferFCardinals = true;
	bool preferGoalConflicts = true;
	int screen = 0;
	highlevel_heuristic HL_heuristic;
	double focal_w = 1.0;
	int child_pref_budget;
	int max_child_pref_options;
	int time_limit;

	// Used to ease tracking of the order of nodes in iterative deepening runs
	uint64_t HL_num_generated_before_this_iteration = 0;
	uint64_t HL_num_expanded_before_this_iteration = 0;

	std::clock_t start;  // Lets subroutines check for a timeout
	// statistics of efficiency
	clock_t runtime = 0; // CPU time, excluding prepTime
	clock_t prepTime = 0; // CPU time for initializing CBS and generating the root node
	clock_t hlHeuristicTime = 0; // CPU time for initializing CBS and generating the root node
	clock_t lowLevelTime = 0; // CPU time of running the low level
	clock_t highLevelTime = 0; // CPU time of running the high level
    clock_t highLevelMddBuildingTime = 0; // CPU time of building MDDs in the high level
    clock_t up_and_down_runtime = 0; // CPU time
    clock_t cat_runtime = 0; // CPU time
    clock_t hl_node_verification_runtime = 0; // CPU time
	std::chrono::nanoseconds wall_runtime = std::chrono::nanoseconds::zero(); // Wall time, excluding prepTime
	std::chrono::nanoseconds wall_prepTime = std::chrono::nanoseconds::zero(); // Wall time for initializing CBS and generating the root node
	std::chrono::nanoseconds wall_hlHeuristicTime = std::chrono::nanoseconds::zero(); // Wall time for initializing CBS and generating the root node
    std::chrono::nanoseconds wall_mddTime = std::chrono::nanoseconds::zero(); // Wall time for building MDDs
	std::chrono::nanoseconds wall_lowLevelTime = std::chrono::nanoseconds::zero(); // Wall time of running the low level
	std::chrono::nanoseconds wall_highLevelTime = std::chrono::nanoseconds::zero(); // Wall time for running the high level
	std::chrono::nanoseconds wall_up_and_down_runtime = std::chrono::nanoseconds::zero(); // Wall time, excluding preprocessing
	std::chrono::nanoseconds wall_cat_runtime = std::chrono::nanoseconds::zero(); // Wall time, excluding preprocessing
	std::chrono::nanoseconds wall_hl_node_verification_runtime = std::chrono::nanoseconds::zero(); // Wall time, excluding preprocessing
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
	void printPaths(vector<vector<PathEntry> *> &paths, int max_len = 100) const;
	void printResults() const;
	void saveResults(const string& outputFile, const string& agentFile, const string& solver) const;

	void isSolutionFeasible();

	bool runICBSSearch();
	bool runIterativeDeepeningICBSSearch();
	ICBSSearch(const MapLoader &ml, const AgentsLoader &al, double focal_w, split_strategy p, highlevel_heuristic HL_h,
               int cutoffTime, int child_pref_budget, int max_child_pref_options, int screen,
               bool propagatePositiveCons, bool preferFCardinals, bool preferGoalConflicts);
	~ICBSSearch();


private:
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::secondary_compare_node> > heap_focal_t;
	heap_open_t open_list;
	heap_focal_t focal_list;
	list<ICBSNode*> allNodes_table;

	// input
	int map_size;
	int num_of_agents;
	const int* moves_offset;
	int num_map_cols;

	ConflictAvoidanceTable root_cat;
	ICBSNode* root_node;
	ICBSNode* solution_node;
	vector < ICBSSingleAgentLLSearch* > search_engines;  // used to find (single) agents' paths and mdd
	vector<vector<PathEntry>*> paths;  // The paths of the node we're currently working on, also the paths of the solution once it's found
	                                   // For best-first-search CBS, this is also the set of paths of the best node in OPEN.
	                                   // This is not the case in DFS variants before the solution is found.
	                                   // This trades the speed of rebuilding it each time we move to a new node for
	                                   // the space of saving it on every node.
	                                   // TODO: Why not keep it on every node, and populate it based on the parent?
	vector<vector<PathEntry>> paths_found_initially;  // contains the initial path that was found for each agent

	// print
	void printConflicts(const ICBSNode &n) const;
	void printConstraints(const ICBSNode* n) const;

	//conflicts
	void findConflicts(ICBSNode& curr);
	void findConflicts(ICBSNode& curr, vector<int>& agents_with_new_paths);
	void findConflicts(ICBSNode &curr, int a1, int a2);
	void classifyConflicts(ICBSNode &node, ConflictAvoidanceTable* cat = nullptr);
	std::shared_ptr<Conflict> getHighestPriorityConflict(ICBSNode &node);
	std::shared_ptr<Conflict> getHighestPriorityConflict(ICBSNode &node, const vector<std::shared_ptr<Conflict>> &confs);
	void copyConflictsFromParent(ICBSNode& curr);
	void clearConflictsOfAgent(ICBSNode &curr, int ag);
	void copyConflicts(const vector<bool>& unchanged,
		const vector<std::shared_ptr<Conflict>>& from, vector<std::shared_ptr<Conflict>>& to);
	void clearConflictsOfAffectedAgents(bool *unchanged, vector<std::shared_ptr<Conflict>> &lst);

	// branch
	void branch(ICBSNode* parent, ICBSNode* child1, ICBSNode* child2);
    void disjoint_branch_on_agent(ICBSNode* parent, ICBSNode* child1, ICBSNode* child2, int agent);
	bool branch_and_generate_with_up_and_down(ICBSNode* parent, ICBSNode* child1, ICBSNode*child2, ConflictAvoidanceTable *cat);
	bool disjoint_branch_and_generate_with_up_and_down(ICBSNode* parent, ICBSNode* child1, ICBSNode* child2,
	                                                   ConflictAvoidanceTable *cat, int agent_id);
    void push_child_into_lists(ICBSNode *child);
	bool findPathForSingleAgent(ICBSNode *node, ConflictAvoidanceTable *cat, int newConstraintTimestep,
                                int earliestGoalTimestep,
                                int ag, bool skipNewpaths = false);
    std::tuple<ICBSNode *, bool> generateChild(ICBSNode *child, ConflictAvoidanceTable *cat = nullptr);
	bool finishPartialExpansion(ICBSNode *node, ConflictAvoidanceTable *cat);
	void buildConflictAvoidanceTable(const ICBSNode &node, int exclude_agent, ConflictAvoidanceTable &cat);
	int  buildConstraintTable(ICBSNode* curr, int agent_id, int newConstraintTimestep,
		std::vector < std::unordered_map<int, ConstraintState > >& cons_table, pair<int, int>& start, pair<int, int>& goal);
	void addPathToConflictAvoidanceTable(vector<PathEntry> &path, ConflictAvoidanceTable &cat, int agent_id);
	void removePathFromConflictAvoidanceTable(vector<PathEntry> &path, ConflictAvoidanceTable &cat, int agent_id);
	int countMddSingletons(vector<vector<PathEntry> *> &paths, int agent_id, int conflict_timestep);
	int getTotalMddWidth(vector<vector<PathEntry> *> &paths, int agent_id);
	void addPositiveConstraintsOnNarrowLevelsLeadingToPositiveConstraint(vector<PathEntry> &parent_path,
                             int new_positive_constraint_timestep, ICBSNode* child, const ConflictAvoidanceTable* cat);
    void removePositiveConstraintsOnNarrowLevelsLeadingToPositiveConstraint(vector<PathEntry> &parent_path,
                             int new_positive_constraint_timestep, ICBSNode* child, const ConflictAvoidanceTable* cat);

    void findShortestPathFromPrevNodeToCurr(ICBSNode *curr, ICBSNode* prev,
                                            vector<ICBSNode *>& steps_up_from_prev_node_to_lowest_common_ancestor,
                                            vector<ICBSNode *>& steps_down_from_lowest_common_ancestor_to_curr_node);


	// update
	inline void populatePaths(ICBSNode *curr, vector<vector<PathEntry> *> &paths);
	void collectConstraints(ICBSNode* curr, std::list<pair<int, tuple<int, int, int, bool>>> &constraints);
	bool reinsert(ICBSNode* curr);
	void updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight);


	// high-level heuristics
	int computeHeuristic(ICBSNode& curr);
	bool KVertexCover(const vector<vector<bool>>& CG, int num_of_CGnodes, int num_of_CGedges, int k);
#ifdef USE_GUROBI
    GRBEnv gurobi_env;
    //TEMP:
    GRBModel mvc_model;
    GRBVar* mvc_vars;
    inline void remove_model_constraints(vector<vector<vector<GRBConstr>>>& Constraints, vector<vector<bool>>& CG, vector<bool>& CgNodeDegrees);
    inline void calcNodeDegrees(ICBSNode& curr, vector<int>& nodeDegrees);
#endif

	// tools
	bool buildMDD(ICBSNode &curr, int ag, int timestep, int lookahead = 0, ConflictAvoidanceTable *cat = nullptr);
	bool arePathsConsistentWithConstraints(vector<vector<PathEntry> *> &paths, ICBSNode *curr) const;
	inline int getAgentLocation(vector<vector<PathEntry> *> &paths, size_t timestep, int agent_id);

	std::tuple<bool, int> do_idcbsh_iteration(ICBSNode *curr,
	                                          ConflictAvoidanceTable &cat,
	                                          int threshold, int next_threshold, clock_t end_by, bool after_bypass = false);
	tuple<bool, bool> idcbsh_add_constraint_and_replan(ICBSNode *node,
	                                                   ConflictAvoidanceTable &cat, int max_cost);
	void idcbsh_unconstrain(ICBSNode *node,
	                        ConflictAvoidanceTable &cat,
	                        vector<PathEntry> &path_backup,
	                        shared_ptr<Conflict> &conflict_backup, int makespan_backup, int g_val_backup,
	                        int h_val_backup, ICBSNode::WillCostIncrease left_will_increase_backup,
                            ICBSNode::WillCostIncrease right_will_increase_backup,
	                        bool just_unconstrain = false);

    void update_cat_and_lpas(ICBSNode *prev_node, ICBSNode *curr,
                             vector<vector<PathEntry>*>& paths, ConflictAvoidanceTable *cat);
};
