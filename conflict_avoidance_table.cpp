#include "conflict_avoidance_table.h"

// Return the number of conflicts with the known_paths (by looking at the conflict avoidance table)
// for the move [curr_id,next_id], entering next_id at next_timestep.
// Returns 0 if no conflict, 1 for vertex or edge conflict, 2 for both.
int numOfConflictsForStep(int curr_id, int next_id, int next_timestep,
                          const std::vector < std::unordered_map<int, AvoidanceState > >& cat,
                          const int* actions_offset)
{
    int retVal = 0;
    if (next_timestep >= cat.size())
    {
        // Check vertex constraints (being at an agent's goal when it is staying there because it is done planning)
        auto it = cat.back().find(next_id);
        if (it != cat.back().end())
            retVal += it->second.vertex;
        // Note -- there cannot be edge conflicts when the other agent is done moving
    }
    else
    {
        // Check vertex constraints (being in next_id at next_timestep is disallowed)
        auto it = cat[next_timestep].find(next_id);
        if (it != cat[next_timestep].end())
        {
            retVal += it->second.vertex;
            // check edge constraints (the move from curr_id to next_id at next_timestep-1 is disallowed)
            for (int i = 0; i < MapLoader::WAIT_MOVE; i++) // the wait action cannot lead to edge conflict
            {
                if (next_id - curr_id == actions_offset[i])
                    retVal += it->second.edge[i];
            }
        }
    }
    return retVal;
}
