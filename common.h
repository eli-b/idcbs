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

// FIXME: Too many includes above
// FIXME: This file should be renamed to ICBSCommon.h

enum split_strategy { NON_DISJOINT, RANDOM, SINGLETONS, WIDTH, DISJOINT3, SPLIT_COUNT };

extern int GRID_COLS;

// Default: use the true distance to the goal location of the agent
// DH: Differential Heuristics where all goal locations are used as pivots 
enum lowlevel_hval { DEFAULT, DH, LLH_COUNT };


enum conflict_type { CARDINAL, SEMICARDINAL, NONCARDINAL, CONFLICT_COUNT };

using namespace std;

// <int loc1, int loc2, int timestep, bool positive_constraint>
// NOTE loc2 = -1 for vertex constraints; loc2 = location2 for edge constraints
typedef std::tuple<int, int, int, bool> Constraint;
std::ostream& operator<<(std::ostream& os, const Constraint& constraint);

// <int loc1, int loc2, bool positive_constraint>
typedef std::tuple<int, int, bool> ConstraintForKnownTimestep;

// <int agent1, int agent2, int loc1, int loc2, int timestep>
// NOTE loc2 = -1 for vertex conflicts; loc2 = location2 for edge conflicts
typedef std::tuple<int, int, int, int, int> Conflict;
std::ostream& operator<<(std::ostream& os, const Conflict& conflict);


struct ConstraintState
{
    bool vertex = false;
    bool edge[5] = { false, false, false, false, false };
};

struct AvoidanceState
{
    uint8_t vertex = 0;
    uint8_t edge[5] = { 0, 0, 0, 0, 0 };
};
