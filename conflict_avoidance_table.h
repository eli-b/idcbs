#pragma once

#include <unordered_map>
#include "XytHolder.h"


struct AvoidanceState
{
    uint8_t vertex = 0;
    uint8_t edge[5] = { 0, 0, 0, 0, 0 };
};

class ConflictAvoidanceTable {
public:
    const int* actions_offset;

    explicit ConflictAvoidanceTable(const int* actions_offset, int map_size)
     : actions_offset(actions_offset), toward_goal(map_size) {}

    XytHolder<AvoidanceState> toward_goal;  // maps location+time pairs to their avoidance state.
    std::unordered_map<int, int> at_goal;  // Maps locations to when agents reach them and never leave (because the location is their goal)

    virtual int num_conflicts_for_step(int curr_id, int next_id, int next_timestep) const;
    virtual void add_action(int timestep, int from, int to);
    virtual void remove_action(int timestep, int from, int to);
    virtual void add_wait_at_goal(int timestep, int location);
    virtual void remove_wait_at_goal(int timestep, int location);
};

class EmptyConflictAvoidanceTable : public ConflictAvoidanceTable {
public:
    explicit EmptyConflictAvoidanceTable() : ConflictAvoidanceTable(nullptr, 0) {}
    int num_conflicts_for_step(int curr_id, int next_id, int next_timestep) const { return 0; }
    void add_action(int timestep, int from, int to) {}
    void remove_action(int timestep, int from, int to) {}
    void add_wait_at_goal(int timestep, int location) {}
    void remove_wait_at_goal(int timestep, int location) {}
};
