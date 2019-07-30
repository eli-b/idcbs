#include "lpa_star.h"
#include <cstring>
#include <climits>
#include <vector>
#include <list>
#include <utility>
#include <boost/heap/fibonacci_heap.hpp>
#include <sparsehash/dense_hash_map>
#include "lpa_node.h"
#include "g_logging.h"
#include "conflict_avoidance_table.h"

using google::dense_hash_map;
using std::cout;
using std::endl;
using boost::heap::fibonacci_heap;
using std::pair;
using std::tuple;
using std::get;
using std::string;
using std::memcpy;

// ----------------------------------------------------------------------------
LPAStar::LPAStar(int start_location, int goal_location, const float* my_heuristic, const MapLoader* ml) :
  my_heuristic(my_heuristic), my_map(ml->my_map), actions_offset(ml->moves_offset) {
  this->start_location = start_location;
  this->goal_location = goal_location;
  this->map_rows = ml->rows;
  this->map_cols = ml->cols;
  this->search_iterations = 0;
  this->num_expanded.push_back(0);
  this->paths.push_back(vector<int>());
  this->paths_costs.push_back(0);
  this->expandedHeatMap.push_back(vector<int>());

  // Initialize allNodes_table (hash table) and OPEN (heap).
  empty_node = new LPANode();
  empty_node->loc_id_ = -1;
  deleted_node = new LPANode();
  deleted_node->loc_id_ = -2;
  allNodes_table.set_empty_key(empty_node);
  allNodes_table.set_deleted_key(deleted_node);
  open_list.clear();
  allNodes_table.clear();

  dcm.setML(ml);

  // Create start node and push into OPEN (findPath is incremental).
  start_n = new LPANode(start_location,
                        0,
                        std::numeric_limits<float>::max(),
                        my_heuristic[start_location],
                        nullptr,
                        0);
  start_n->openlist_handle_ = open_list.push(start_n);
  start_n->in_openlist_ = true;
  allNodes_table[start_n] = start_n;

  // Create goal node. (Not being pushed to OPEN.)
  goal_n = new LPANode(goal_location,
                       std::numeric_limits<float>::max(),  // g_val
                       std::numeric_limits<float>::max(),  // v_val
                       my_heuristic[goal_location],         // h_val
                       nullptr,                             // bp
                       std::numeric_limits<int>::max());    // t
  allNodes_table[goal_n] = goal_n;

  // For the case of the trivial path - the start node is never passed to updateState
  if (start_n->loc_id_ == goal_location &&
      nodes_comparator(start_n, goal_n) == false) {
    VLOG(7) << "\t\tupdateState: Goal node update -- from " << goal_n->nodeString() << " to " << start_n->nodeString();
    goal_n = start_n;
  }
}
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
bool LPAStar::updatePath(LPANode* goal) {
  LPANode* curr = goal;
  while (curr != start_n) {
     if (curr == nullptr)
      return false;
    VLOG(11) << curr->nodeString();
    paths[search_iterations].push_back(curr->loc_id_);
    curr = curr->bp_;
  }
  paths[search_iterations].push_back(start_location);
  reverse(paths[search_iterations].begin(),
          paths[search_iterations].end());

  paths_costs[search_iterations] = goal->g_;
  return true;
}
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
void LPAStar::addVertexConstraint(int loc_id, int ts, const std::vector < std::unordered_map<int, AvoidanceState > >& cat) {
  VLOG_IF(1, ts == 0) << "We assume vertex constraints cannot happen at timestep 0.";
  // 1) Invalidate this node (that is, sets bp_=nullptr, g=INF, v=INF) and remove from OPEN.
  LPANode* n = retrieveNode(loc_id, ts).second;
  // "Invalidates" n (that is, sets bp_=nullptr, g=INF, v=INF) and remove from OPEN.
  n->initState();
  if (n->in_openlist_ == true) {
    openlistRemove(n);
  }
  for (int direction = 0; direction < 5; direction++) {
    auto succ_loc_id = loc_id + actions_offset[direction];
    if (0 <= succ_loc_id && succ_loc_id < map_rows*map_cols && !my_map[succ_loc_id] &&
        abs(succ_loc_id % map_cols - loc_id % map_cols) < 2) {
      addEdgeConstraint(loc_id, succ_loc_id, ts+1, cat);
      addEdgeConstraint(succ_loc_id, loc_id, ts, cat);
    }
  }
}
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
void LPAStar::addEdgeConstraint(int from_id, int to_id, int ts, const std::vector < std::unordered_map<int, AvoidanceState > >& cat) {
  dcm.addEdgeConstraint(from_id, to_id, ts);
  LPANode* to_n = retrieveNode(to_id, ts).second;
  if (to_n->bp_ != nullptr && to_n->bp_->loc_id_ == from_id) {
    updateState(to_n, cat, false);
  }
}
// ----------------------------------------------------------------------------


/*
 * Retrieves a pointer to a node:
 * 1) if it was already generated before, it is retrieved from the hash table and returned (along with true)
 * 2) if this state is seen for the first time, a new node is generated (and initialized) and then put into the hash table and returned (along with false)
*/
// ----------------------------------------------------------------------------
inline std::pair<bool, LPANode*> LPAStar::retrieveNode(int loc_id, int t) {  // (t=0 for single agent)
  // create dummy node to be used for table lookup
  LPANode* temp_n = new LPANode(loc_id,
                                std::numeric_limits<float>::max(),  // g_val
                                std::numeric_limits<float>::max(),  // v_val
                                my_heuristic[loc_id],                // h_val
                                nullptr,                             // bp
                                t);                                  // timestep
  hashtable_t::iterator it;
  // try to retrieve it from the hash table
  it = allNodes_table.find(temp_n);
  if ( it == allNodes_table.end() ) {  // case (2) above
    //num_generated[search_iterations]++; -- counted instead when adding to OPEN (so we account for reopening).
    //temp_n->initState();  -- already done correctly in construction above.
    allNodes_table[temp_n] = temp_n;
    VLOG(11) << "\t\t\t\t\tallNodes_table: Added new node" << temp_n->nodeString();
    return (make_pair(false, temp_n));
  } else {  // case (1) above
    delete(temp_n);
    VLOG(11) << "\t\t\t\t\tallNodes_table: Returned existing" << (*it).second->nodeString();
    return (make_pair(true, (*it).second));
  }
  return (make_pair(true, nullptr));  // should never get here...
}
// ----------------------------------------------------------------------------


// Adds a node (that was already initialized via retrieveNode) to OPEN
// ----------------------------------------------------------------------------
inline void LPAStar::openlistAdd(LPANode* n) {
  n->openlist_handle_ = open_list.push(n);
  n->in_openlist_ = true;
}
// ----------------------------------------------------------------------------


// Updates the priority
// ----------------------------------------------------------------------------
inline void LPAStar::openlistUpdate(LPANode* n) {
  open_list.update(n->openlist_handle_);
}
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
inline void LPAStar::openlistRemove(LPANode* n) {
  open_list.erase(n->openlist_handle_);
  n->in_openlist_ = false;
}
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
inline LPANode* LPAStar::openlistPopHead() {
  LPANode* retVal = open_list.top();
  open_list.pop();
  retVal->in_openlist_ = false;
  num_expanded[search_iterations]++;
  expandedHeatMap[search_iterations].push_back(retVal->loc_id_);
  return retVal;
}
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
inline void LPAStar::releaseNodesMemory() {
  for (auto n : allNodes_table) {
    delete(n.second);  // n is std::pair<Key, Data*>
  }
}
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
inline void LPAStar::printAllNodesTable() {
  cout << "Printing all nodes in the hash table:" << endl;
  for (auto n : allNodes_table) {
    cout << "\t" << (n.second)->stateString() << " ; Address:" << (n.second) << endl;  // n is std::pair<Key, Data*>
  }
}
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
inline LPANode* LPAStar::retrieveMinPred(LPANode* n) {
  VLOG(11) << "\t\t\t\tretrieveMinPred: before " << n->nodeString();
  LPANode* retVal = nullptr;
  auto best_vplusc_val = std::numeric_limits<float>::max();
  for (int direction = 0; direction < 5; direction++) {
    auto pred_loc_id = n->loc_id_ - actions_offset[direction];
    if (0 <= pred_loc_id && pred_loc_id < map_rows*map_cols && !my_map[pred_loc_id] &&
        abs(pred_loc_id % map_cols - n->loc_id_ % map_cols) < 2 &&
        !dcm.isDynCons(pred_loc_id,n->loc_id_,n->t_)) {
      auto pred_n = retrieveNode(pred_loc_id, n->t_-1).second; // n->t_ - 1 is pred_timestep
      if (pred_n->v_ + 1 <= best_vplusc_val) {  // Assumes unit edge costs.
        best_vplusc_val = pred_n->v_ + 1;  // Assumes unit edge costs.
        retVal = pred_n;
      }
    }
  }
  VLOG_IF(11, retVal == nullptr) << "\t\t\t\tretrieveMinPred: min is ****NULL**** BAD!!";
  VLOG_IF(11, retVal != nullptr) << "\t\t\t\tretrieveMinPred: min is " << retVal->nodeString();
  return retVal;
}
// ----------------------------------------------------------------------------


// note -- we assume that n was visited (/generated) via a call to retrieveNode earlier
// note2 -- parameter bp_already_set used for optimization (section 6 of the LPA* paper).
// ----------------------------------------------------------------------------
inline void LPAStar::updateState(LPANode* n, const std::vector < std::unordered_map<int, AvoidanceState > >& cat,
                                 bool bp_already_set) {
  if (n != start_n) {
    VLOG(7) << "\t\tupdateState: Start working on " << n->nodeString();
    if (bp_already_set == false) {
        n->bp_ = retrieveMinPred(n);
        if (n->bp_ == nullptr) {  // This node is a "dead-end" or has a vertex constraint on it.
            n->initState();
            if (n->in_openlist_) {
                openlistRemove(n);
            }
            return;
        }
    }
    n->g_ = (n->bp_)->v_ + 1;  // If we got to this point this traversal is legal (Assumes edges have unit cost).
    int n_conflicts = numOfConflictsForStep(n->bp_->loc_id_, n->loc_id_, n->t_, cat, actions_offset);
    n->conflicts_ = (n->bp_)->conflicts_ + n_conflicts;
    VLOG(7) << "\t\tupdateVertex: After updating bp -- " << n->nodeString();
    // UpdateVertex from the paper:
    if ( !n->isConsistent() ) {
      if (n->in_openlist_ == false) {
        openlistAdd(n);  // The open list contains all inconsistent nodes
        VLOG(7) << "\t\t\tand *PUSHED* to OPEN";
      } else {  // node is already in OPEN
        openlistUpdate(n);
        VLOG(7) << "\t\t\tand *UPDATED* in OPEN";
      }
    } else {  // n is consistent
      if (n->in_openlist_) {
        openlistRemove(n);
        VLOG(7) << "\t\t\tand *REMOVED* from OPEN";
      }
    }
    // Not described in the paper, but necessary:
    // If goal was found with better priority then update the relevant node.
    if (n->loc_id_ == goal_location &&  // TODO: MAPF has additional time restrictions on goal condition...
        nodes_comparator(n, goal_n) == false) {
      VLOG(7) << "\t\tupdateState: Goal node update -- from " << goal_n->nodeString() << " to " << n->nodeString();
      goal_n = n;
    }
  }
}
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
bool LPAStar::findPath(const std::vector < std::unordered_map<int, AvoidanceState > >& cat, int fLowerBound, int minTimestep) {

  search_iterations++;
  num_expanded.push_back(0);
  expandedHeatMap.push_back(vector<int>());

  VLOG(5) << "*** Starting LPA* findPath() ***";
  while (open_list.empty() == false &&
         (nodes_comparator( open_list.top(), goal_n ) == false ||  // open.minkey < key(goal).
         goal_n->v_ < goal_n->g_)) {  // Safe if both are numeric_limits<float>::max.
    VLOG(5) << "OPEN: { " << openToString(true) << " }\n";
    auto curr = openlistPopHead();
    VLOG(5) << "\tPopped node: " << curr->nodeString();
    if (curr->v_ > curr->g_) {  // Overconsistent (v>g).
      VLOG(7) << "(it is *over*consistent)";
      curr->v_ = curr->g_;
      for (int direction = 0; direction < 5; direction++) {
        auto next_loc_id = curr->loc_id_ + actions_offset[direction];
        if (0 <= next_loc_id && next_loc_id < map_rows*map_cols && !my_map[next_loc_id] &&
            abs(next_loc_id % map_cols - curr->loc_id_ % map_cols) < 2) {
            auto next_n = retrieveNode(next_loc_id, curr->t_+1);
            if (next_n.second->g_ > curr->v_ + 1) {
                next_n.second->bp_ = curr;
                //next_n.second->g_ = curr->v_ + 1;  // Done in updateState
                updateState(next_n.second, cat, true);
            }
        }
      }
    } else {  // Underconsistent (v<g).
      VLOG(7) << "(it is *under*consistent)";
      curr->v_ = std::numeric_limits<float>::max();
      updateState(curr, cat);
      for (int direction = 0; direction < 5; direction++) {
        auto next_loc_id = curr->loc_id_ + actions_offset[direction];
        if (0 <= next_loc_id && next_loc_id < map_rows*map_cols && !my_map[next_loc_id] &&
            abs(next_loc_id % map_cols - curr->loc_id_ % map_cols) < 2) {
          auto next_n = retrieveNode(next_loc_id, curr->t_+1);
          updateState(next_n.second, cat, false);
        }
      }
    }
  }
  paths.push_back(vector<int>());
  paths_costs.push_back(0);
  if (goal_n->g_ < std::numeric_limits<float>::max()) {  // If a solution was found.
    return updatePath(goal_n);
  }
  return false;  // No solution found.
}
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
string LPAStar::openToString(bool print_priorities) const {
  string retVal;
  for (auto it = open_list.ordered_begin(); it != open_list.ordered_end(); ++it) {
    if (print_priorities == true)
      retVal = retVal + (*it)->nodeString() + " ; ";
    else
      retVal = retVal + (*it)->stateString() + " ; ";
  }
  return retVal;
}
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
// Note -- Fibonacci has amortized constant time insert. That's why rebuilding the heap is linear time.
//         TODO: maybe find out how to create a *deep* copy of the heap (but this isn't easy...)
LPAStar::LPAStar (const LPAStar& other) :
start_location(other.start_location),
goal_location(other.goal_location),
my_heuristic(other.my_heuristic),
my_map(other.my_map),
map_rows(other.map_rows),
map_cols(other.map_cols),
actions_offset(other.actions_offset),
dcm(other.dcm)
{
    search_iterations = 0;
    num_expanded.push_back(0);
    paths.push_back(vector<int>());
    paths_costs.push_back(0);
    expandedHeatMap.push_back(vector<int>());
    empty_node = new LPANode(*(other.empty_node));
    deleted_node = new LPANode(*(other.deleted_node));
    // Create a deep copy of each node and store it in the new Hash table.
    allNodes_table.set_empty_key(empty_node);
    allNodes_table.set_deleted_key(deleted_node);
    // Map
    for (auto n : other.allNodes_table) {
        allNodes_table[n.first] = new LPANode(*(n.second));  // n is std::pair<Key, Data*>.
    }
    // Reconstruct the OPEN list with the cloned nodes.
    // This is efficient enough since FibHeap has amortized constant time insert.
    for (auto it = other.open_list.ordered_begin(); it != other.open_list.ordered_end(); ++it) {
        LPANode* n = allNodes_table[*it];
        n->openlist_handle_ = open_list.push(n);
    }
    // Update the backpointers of all cloned versions.
    // (before this its bp_ is the original pointer, but we can use the state in it to
    // retrieve the new clone from the newly built hash table).
    for (auto n : allNodes_table) {
        if (n.second->bp_ != nullptr) {
            n.second->bp_ = allNodes_table[n.second->bp_];
        }
    }
    // Update start and goal nodes.
    start_n = allNodes_table[other.start_n];
    goal_n = allNodes_table[other.goal_n];
}
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
LPAStar::~LPAStar() {
  releaseNodesMemory();
  delete(empty_node);
  delete(deleted_node);
}
// ----------------------------------------------------------------------------