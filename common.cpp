#include "common.h"

constexpr int GRID_ROWS = 8;

std::ostream& operator<<(std::ostream& os, const Constraint& constraint)
{
    auto [loc1, loc2, timestep, positive_constraint] = constraint;
    if (loc2 == -1)
        os << "<(" << loc1 / GRID_ROWS << "," << loc1 % GRID_ROWS << "), " << timestep;
    else
        os << "<(" << loc1 / GRID_ROWS << "," << loc1 % GRID_ROWS << "), (" << loc2 / GRID_ROWS << "," << loc2 % GRID_ROWS << "), " << timestep;
	if (positive_constraint)
		os << ", positive>";
	else
		os << ", negative>";
	return os;
}

std::ostream& operator<<(std::ostream& os, const Conflict& conflict)
{
    auto [agent1, agent2, loc1, loc2, timestep] = conflict;
    if (loc2 == -1)
        os << "<" << agent1 << ", " << agent2 << ", (" << loc1 / GRID_ROWS << "," << loc1 % GRID_ROWS << "), " << timestep << ">";
    else
        os << "<" << agent1 << ", " << agent2 << ", (" << loc1 / GRID_ROWS << "," << loc1 % GRID_ROWS << "), ("
                                                       << loc2 / GRID_ROWS << "," << loc2 % GRID_ROWS << "), " << timestep << ">";
	return os;
}
