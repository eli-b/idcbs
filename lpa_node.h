#ifndef LPANODE_H
#define LPANODE_H

#include <boost/heap/fibonacci_heap.hpp>
#include <string>
#include <vector>
#include <list>
#include <functional>  // for std::hash (c++11 and above)
#include <algorithm>  // for std::min
#include <limits>  // for std::numeric_limits
#include <iostream>
#include <sparsehash/dense_hash_map>

using google::dense_hash_map;
using boost::heap::fibonacci_heap;
using boost::heap::compare;

class LPANode {
 public:
  int loc_id_ = -1;
  float g_ = std::numeric_limits<float>::max();
  float v_ = std::numeric_limits<float>::max();
  float h_ = 0;
  LPANode* bp_ = nullptr;
  int t_ = -1;
  int num_internal_conf_ = 0;  // used in ECBS and iECBS
  bool in_openlist_ = false;  // *not* used in Focal Search (reexpansions are required)
  bool is_truncated_ = false;  // used in TLPA*
  float g_pi_ = std::numeric_limits<float>::max();  // used in TLPA*
  std::list<LPANode*> pi_;  // used in TLPA*

  ///////////////////////////////////////////////////////////////////////////////
  // NOTE -- Normally, compare_node (lhs,rhs) suppose to return true if lhs<rhs.
  //         However, Heaps in STL and Boost are implemented as max-Heap.
  //         Hence, to achieve min-Heap, we return true if lhs>rhs
  ///////////////////////////////////////////////////////////////////////////////
  
  // the following is used to compare nodes in (Incremental Search) OPEN list
  struct compare_node {
    // returns true if n1 > n2
    bool operator()(const LPANode* n1, const LPANode* n2) const {
      if ( n1->getKey1() == n2->getKey1() )
        return n1->getKey2() >= n2->getKey2();  // break ties towards *lower* g-vals
      return n1->getKey1() >= n2->getKey1();
    }
  };


  // the following is used to compare nodes in the FOCAL list
  struct secondary_compare_node {
    // returns true if n1 > n2
    bool operator()(const LPANode* n1, const LPANode* n2) const {
      if (n1->num_internal_conf_ == n2->num_internal_conf_) {
        if (n1->g_ + n1->h_ == n2->g_ + n2->h_)
          return n1->g_ <= n2->g_;  // break ties towards larger g_vals
        return n1->g_ + n1->h_ >= n2->g_ + n2->h_;  // break ties towards smaller f_vals (prefer shorter solutions)
      }
      return n1->num_internal_conf_ >= n2->num_internal_conf_;  // n1 > n2 if it has more conflicts
    }
  };  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)


  // define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
  typedef boost::heap::fibonacci_heap< LPANode* , compare<compare_node> >::handle_type openlist_handle_t;
  typedef boost::heap::fibonacci_heap< LPANode* , compare<secondary_compare_node> >::handle_type focallist_handle_t;

  openlist_handle_t openlist_handle_;
  focallist_handle_t focallist_handle_;

  LPANode();  // ctor
  LPANode(const LPANode& other);  // copy ctor (deep copy).
//  LPANode(const LPANode* other);  // copy ctor (deep copy).
  LPANode(int id, float g_val, float v_val, float h_val, LPANode* parent, int timestep = 0,
    int num_internal_conf = 0, bool in_openlist = false);

  ~LPANode();  // dtor.

  inline float getFVal() const {return g_ + h_;}

  inline float getKey1() const {return std::min(g_, v_) + h_;}

  inline float getKey2() const {return std::min(g_, v_);}

  inline void initState() {
    g_ = std::numeric_limits<float>::max();
    v_ = std::numeric_limits<float>::max();
    bp_ = nullptr;
  }

  std::string stateString() const {
    return ("<" + std::to_string(loc_id_) + "," + std::to_string(t_) + ">");
  }

  std::string nodeString() const {
    std::string bp_str = (bp_ == nullptr) ? "NULL" : bp_->stateString();
    std::string v_str = (v_ == std::numeric_limits<float>::max()) ? "INF" : std::to_string(v_).substr(0,4);
    std::string g_str = (g_ == std::numeric_limits<float>::max()) ? "INF" : std::to_string(g_).substr(0,4);
    std::string h_str = (h_ == std::numeric_limits<float>::max()) ? "INF" : std::to_string(h_).substr(0,4);
    std::string gpi_str = (g_pi_ == std::numeric_limits<float>::max()) ? "INF" : std::to_string(g_pi_).substr(0,4);
    return ("(" + stateString() + ", v=" + v_str
            + ", g=" + g_str + ", h=" + h_str + ", bp=" + bp_str + ", g_pi=" + gpi_str + ")");
  }

  std::string piString() const {
    std::string retVal;
    for (auto n:pi_) {
      retVal = retVal + n->stateString() + " ; ";
    }
    return retVal;
  }

  inline bool isConsistent() const {return (g_ == v_);}

  // The following is used by googledensehash for checking whether two nodes are equal
  // we say that two nodes, s1 and s2, are equal if
  // both are non-NULL and agree on the id and timestep
  struct eqnode {
    bool operator()(const LPANode* s1, const LPANode* s2) const {
      return (s1 == s2) || (s1 && s2 && s1->loc_id_ == s2->loc_id_ && s1->t_ == s2->t_);
    }
  };

  // The following is used by googledensehash for generating the hash value of a nodes
  // This is needed because otherwise we'll have to define the specialized template inside std namespace
  struct LPANodeHasher {
    std::size_t operator()(const LPANode* n) const {
      size_t id_hash = std::hash<int>()(n->loc_id_);
      size_t timestep_hash = std::hash<int>()(n->t_);
      return ( id_hash ^ (timestep_hash << 1) );
    }
  };
};

std::ostream& operator<<(std::ostream& os, const LPANode& n);

#endif
