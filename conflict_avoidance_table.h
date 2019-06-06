#pragma once

#include "common.h"
#include "map_loader.h"


int numOfConflictsForStep(int curr_id, int next_id, int next_timestep,
                          const std::vector < std::unordered_map<int, AvoidanceState > >& cat,
                          const int* actions_offset);
