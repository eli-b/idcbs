#include "lpa_node.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>

LPANode::LPANode() {
}

LPANode::LPANode(int id, float g_val, float v_val, float h_val, LPANode* parent, int timestep, int num_internal_conf, bool in_openlist):
    loc_id_(id),  g_(g_val), v_(v_val), h_(h_val), bp_(parent), t_(timestep),
    num_internal_conf_(num_internal_conf), in_openlist_(in_openlist) {
}

LPANode::LPANode(const LPANode& other) {
  loc_id_ = other.loc_id_;
  g_ = other.g_;
  v_ = other.v_;
  h_ = other.h_;
  bp_ = other.bp_;
  t_ = other.t_;
  num_internal_conf_ = other.num_internal_conf_;
  in_openlist_ = other.in_openlist_;
  openlist_handle_ = other.openlist_handle_;
  focallist_handle_ = other.focallist_handle_;
  // For TLPA*, we copy the is_truncated_ but will repopulate the rest later.
  is_truncated_ = other.is_truncated_;
  g_pi_ = std::numeric_limits<float>::max();
  //pi_.clear();  -- node starts with an empty pi_.
}


LPANode::~LPANode() {
}

std::ostream& operator<<(std::ostream& os, const LPANode& n) {
  os << n.nodeString();
  return os;
}
