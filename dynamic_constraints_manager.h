#ifndef DYNAMICCONSTRAINTSMANAGER_H
#define DYNAMICCONSTRAINTSMANAGER_H

#include <stdlib.h>
#include <vector>
#include <list>
#include <utility>
#include <string>
#include <iostream>
#include <cassert>
#include <fstream>
#include <stdio.h>
#include <string>
#include "map_loader.h"

using std::cout;
using std::vector;
using std::list;
using std::pair;
using std::make_pair;
using std::string;

class DynamicConstraintsManager {
public:

  // dyn_constraints[i] is a list of disallowed <from_id,to_id> transitions for timestep i.
  // Meaning arriving to to_id from from_id at time i is disallowed.
  vector < list< pair<int, int> > > dyn_constraints_;

  // A reference to the environment (so we can convert Vertex constraints to Edge constraints).
  const MapLoader* ml_;

  // Vertex constraint semantics: being at loc_id at time ts is disallowed.
  void addVertexConstraint(int loc_id, int ts);
  void popVertexConstraint(int loc_id, int ts);
  // Edge constraint semantics: moving from from_id to to_id and arriving there at ts is disallowed.
  void addEdgeConstraint(int from_id, int to_id, int ts);
  void popEdgeConstraint(int from_id, int to_id, int ts);

/* Returns true if the move <curr_id, next_id, next_t> is valid w.r.t. dyn_constraints.
*/
  // Returns true if the move from from_id (being there at ts) to to_id (getting there at next_ts) is valid
  bool isDynCons(int curr_id, int next_id, int next_timestep);

  void setML(const MapLoader* ml) {ml_ = ml;}
  DynamicConstraintsManager();  // C'tor.
  DynamicConstraintsManager(const DynamicConstraintsManager& other); // Copy c'tor.
  ~DynamicConstraintsManager();  // D'tor.

private:
  // Both constraint methods above use this one. Assumes <from,to,ts> is otherwise a valid move! (that is, from/to are not a blocked cell.)
  void addDynConstraint(int from_id, int to_id, int ts);
  void popDynConstraint(int from_id, int to_id, int ts);

};
#endif
