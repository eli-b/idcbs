#pragma once

#include <boost/heap/fibonacci_heap.hpp>
#include <google/dense_hash_map>

#include <cassert>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <vector>
#include <iostream>
#include <list>
#include <functional>  // for std::hash (c++11 and above)
#include <ctime>
#include <memory> // for std::shared_ptr
#include <map>
#include <cstring>
#include <algorithm>
#include "spdlog/fmt/ostr.h" // For defining operator<< on Conflict and Constraint objects in spdlog

// FIXME: Too many includes above
// FIXME: This file should be renamed to ICBSCommon.h

enum split_strategy { NON_DISJOINT, RANDOM, SINGLETONS, WIDTH, DISJOINT3, MVC_BASED, SPLIT_STRATEGY_COUNT };

extern int GRID_COLS;

// Default: use the true distance to the goal location of the agent
// DH: Differential Heuristics where all goal locations are used as pivots
enum lowlevel_heuristic { DEFAULT, DH, LLH_COUNT };

enum highlevel_heuristic { NONE, CG, DG, EWDG, VWCG, EWVWDG, HLH_COUNT };

enum conflict_type { F_CARDINAL, CARDINAL_GOAL, SEMI_F_CARDINAL, CARDINAL, SEMICARDINAL_GOAL, SEMICARDINAL, NONCARDINAL, CONFLICT_TYPE_COUNT };

using namespace std;

// <int loc1, int loc2, int timestep, bool positive_constraint>
// NOTE loc2 = -1 for vertex constraints; loc2 = location2 for edge constraints
typedef std::tuple<int, int, int, bool> Constraint;
namespace std  // Forces the templating stage to consider our typedef above (see https://github.com/gabime/spdlog/issues/1227#issuecomment-532009129)
{
std::ostream& operator<<(std::ostream& os, const Constraint& constraint);
}

// <int loc1, int loc2, bool positive_constraint>
typedef std::tuple<int, int, bool> ConstraintForKnownTimestep;

// <int agent1, int agent2, int loc1, int loc2, int timestep>
// NOTE loc2 = -1 for vertex conflicts; loc2 = location2 for edge conflicts
typedef std::tuple<int, int, int, int, int> Conflict;
namespace std  // Forces the templating stage to consider our typedef above (see https://github.com/gabime/spdlog/issues/1227#issuecomment-532009129)
{
std::ostream& operator<<(std::ostream& os, const Conflict& conflict);
}

struct ConstraintState
{
    bool vertex = false;
    bool edge[5] = { false, false, false, false, false };
};

