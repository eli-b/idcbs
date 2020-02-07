#pragma once

#include "common.h"

using namespace std;

// Low-level heuristic calculator and holder for CBS
class HeuristicCalculator
{
public:
	int start_location;  // Unused
	int goal_location;
	int agent_size;
	const bool* my_map;
	int map_rows;
	int map_cols;
	const int* moves_offset;

	HeuristicCalculator(int start_location, int goal_location, const bool* my_map, int map_rows, int map_cols, const int* moves_offset);
	void getAllPairsHVals(vector<vector<int>>& res);
	void computeHVals(int* res);

};

