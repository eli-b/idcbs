#include "conflict_avoidance_table.h"

// Return the number of conflicts with the known_paths (by looking at the conflict avoidance table)
// for the move [curr_id,next_id], entering next_id at next_timestep.
int ConflictAvoidanceTable::num_conflicts_for_step(int curr_id, int next_id, int next_timestep) const
{
    int retVal = 0;

    // Check if next_id collides with an agent that reached its goal
    auto goal_it = at_goal.find(next_id);
    if (goal_it != at_goal.end())
    {
        int other_agent_first_timestep = goal_it->second;

        if (other_agent_first_timestep <= next_timestep)  // next_id was found in the map for the last timestep in the plans of other agents,
            retVal += 1;
    }

    if (next_timestep >= toward_goal.size())
        return retVal;

    // Check for other collisions
    auto it = toward_goal[next_timestep].find(next_id);
    if (it != toward_goal[next_timestep].end())
    {
        retVal += it->second.vertex;
        // check edge constraints (the move from curr_id to next_id at next_timestep-1 is disallowed)
        for (int i = 0; i < MapLoader::WAIT_MOVE; i++) // the wait action cannot lead to edge conflicts
        {
            if (next_id - curr_id == actions_offset[i])
                retVal += it->second.edge[i];
        }
    }

    return retVal;
}

void ConflictAvoidanceTable::add_action(int timestep, int from, int to)
{
    if (timestep >= toward_goal.size())
        toward_goal.resize(timestep + 1);
    AvoidanceState& to_entry = toward_goal[timestep][to];
    AvoidanceState& from_entry = toward_goal[timestep][from];
    if (to_entry.vertex < 255)
        to_entry.vertex++;
    for (int i = 0; i < MapLoader::valid_moves_t::WAIT_MOVE; i++)
    {
        if (from - to == actions_offset[i]) {
            if (from_entry.edge[i] < 255)
                from_entry.edge[i]++;
            break;
        }
    }
    // TODO: Have a small table mapping from move offsets to their index and use it instead of iterating
}

void ConflictAvoidanceTable::add_wait_at_goal(int timestep, int location) {
    at_goal[location] = timestep;
}

void ConflictAvoidanceTable::remove_wait_at_goal(int timestep, int location) {
    at_goal.erase(location);
}

void ConflictAvoidanceTable::remove_action(int timestep, int from, int to)
{
    AvoidanceState& to_entry = toward_goal[timestep][to];
    AvoidanceState& from_entry = toward_goal[timestep][from];
    if (to_entry.vertex > 0)
        to_entry.vertex--;
    else {
        std::cerr << "Path already removed?? Vertex " << to << " at timestep " << timestep << " is not in the CAT" << std::endl;
        std::abort();
    }
    for (int i = 0; i < MapLoader::valid_moves_t::WAIT_MOVE; i++)
    {
        if (from - to == actions_offset[i]) {
            if (from_entry.edge[i] > 0)
                from_entry.edge[i]--;
            else {
                std::cerr << "Path already removed?? Edge from " << from << " to " << to << " at timestep " << timestep << " is not in the CAT" << std::endl;
                std::abort();
            }
            break;
        }
    }
    // TODO: Have a small table mapping from move offsets to their index and use it instead of iterating
}
