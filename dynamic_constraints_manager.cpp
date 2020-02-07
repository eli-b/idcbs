#include "dynamic_constraints_manager.h"
#include "g_logging.h"


DynamicConstraintsManager::DynamicConstraintsManager() {
	dyn_constraints_.resize(0);
	ml_ = nullptr;
}


DynamicConstraintsManager::DynamicConstraintsManager(const DynamicConstraintsManager& other) {
	dyn_constraints_ = other.dyn_constraints_;
	ml_ = other.ml_;
}


DynamicConstraintsManager::~DynamicConstraintsManager() {
}


void DynamicConstraintsManager::addVertexConstraint(int loc_id, int ts) {
	/* Note -- We replace a vertex constraint with 10 respective edge constraints. (Old: addDynConstraint(loc_id, -1, ts);)
		Vertex constraint at loc_id at time ts means:
		1) We cannot move to it from any of the neighbors (and get there at time ts).
		2) We cannot move from it to any of the neighbors (and get there at time ts+1).
		3) We cannot move from it to it (and get there at time ts+1 or ts)
	*/
	VLOG_IF(1, ml_==nullptr) << "ERROR: Assumes ml_ was set.";
	for (int direction = 0; direction < 5; direction++) {
		auto succ_loc_id = loc_id + ml_->moves_offset[direction];
		if (0 <= succ_loc_id && succ_loc_id < ml_->map_size() && !ml_->is_blocked(succ_loc_id)) {
			addEdgeConstraint(loc_id, succ_loc_id, ts+1);
			addEdgeConstraint(succ_loc_id, loc_id, ts);
		}
	}
}

void DynamicConstraintsManager::popVertexConstraint(int loc_id, int ts) {
    /* Note -- We replace a vertex constraint with 10 respective edge constraints. (Old: addDynConstraint(loc_id, -1, ts);)
        Vertex constraint at loc_id at time ts means:
        1) We cannot move to it from any of the neighbors (and get there at time ts).
        2) We cannot move from it to any of the neighbors (and get there at time ts+1).
        3) We cannot move from it to it (and get there at time ts+1 or ts)
    */
    VLOG_IF(1, ml_==nullptr) << "ERROR: Assumes ml_ was set.";
    for (int direction = 0; direction < 5; direction++) {
        auto succ_loc_id = loc_id + ml_->moves_offset[direction];
        if (0 <= succ_loc_id && succ_loc_id < ml_->map_size() && !ml_->is_blocked(succ_loc_id)) {
            popEdgeConstraint(loc_id, succ_loc_id, ts+1);
            popEdgeConstraint(succ_loc_id, loc_id, ts);
        }
    }
}


void DynamicConstraintsManager::addEdgeConstraint(int from_id, int to_id, int ts) {
	addDynConstraint(from_id, to_id, ts);
}


void DynamicConstraintsManager::popEdgeConstraint(int from_id, int to_id, int ts) {
    popDynConstraint(from_id, to_id, ts);
}


void DynamicConstraintsManager::addDynConstraint(int from_id, int to_id, int ts) {
 	//if (isDynCons(from_id, to_id, ts)) - causes popped constraints not to be in their expected place
 	//    return;

    // If dyn_constraints is not big enough, extend it up to next_timestep (with empty lists).
	if (dyn_constraints_.size() <= (size_t)ts) {
		dyn_constraints_.resize(ts+1, list< pair<int,int> >());
	}
 	// Add it to the list of dynamic constraints.
	dyn_constraints_[ts].emplace_back(from_id, to_id);
}

void DynamicConstraintsManager::popDynConstraint(int from_id, int to_id, int ts) {
    // Remove it from the list of dynamic constraints.
    auto [back_from_id, back_to_id] = dyn_constraints_[ts].back();
    VLOG_IF(1, back_from_id != from_id) << "ERROR: We assume constraints are popped in the same order as they're added, but in reverse";
    assert(back_from_id == from_id);
    VLOG_IF(1, back_to_id != to_id) << "ERROR: We assume constraints are popped in the same order as they're added, but in reverse";
    assert(back_to_id == to_id);
    dyn_constraints_[ts].pop_back();
}

bool DynamicConstraintsManager::isDynCons(int curr_id, int next_id, int next_ts) {
	VLOG(11) << "\t\t\tisDynConstrained: <from=" << curr_id << ", to=" << next_id << ", t=" << next_ts << ">";
	// Check edge constraints (move from curr_id to next_id (getting to next_id at next_ts) is disallowed).
	if ( next_ts > 0 && next_ts < static_cast<int>(dyn_constraints_.size()) ) {
		for (auto c : dyn_constraints_[next_ts]) {
			if ( c.first == curr_id && c.second == next_id ) {
				VLOG(11) << "\t\t\t\tYES DYN_CONS!";
				return true;
			}
		}
	}

	VLOG(11) << "\t\t\t\tNOT DYN_CONS!";
	return false;
}
