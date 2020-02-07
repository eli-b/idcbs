#pragma once
#include "common.h"

using boost::heap::fibonacci_heap;
using boost::heap::compare;


struct PathEntry
{
	int location;
	bool single; // it is the singleton in its MDD
	int numMDDNodes; // number of MDD nodes at the given timestep
	bool builtMDD; // only when builtMDD is true, single is useful.
	PathEntry(){location = -1; builtMDD= false; single = false; numMDDNodes = 0;}
};

class ICBSSingleAgentLLNode
{
public:
	int loc;
	int g_val;
	int h_val = 0;
	ICBSSingleAgentLLNode* parent;
	int timestep = 0;
	int num_internal_conf = 0;
	bool in_openlist = false;

	///////////////////////////////////////////////////////////////////////////////
	// NOTE -- Normally, compare_node (lhs,rhs) is supposed to return true if lhs<rhs.
	//         However, Heaps in STL and Boost are implemented as max-Heap.
	//         Hence, to achieve min-Head, we return true if lhs>rhs
	///////////////////////////////////////////////////////////////////////////////

	// the following is used to compare nodes in the OPEN list
	struct compare_node 
	{
		// returns true if n1 > n2 (note -- this gives us *min*-heap).
		bool operator()(const ICBSSingleAgentLLNode* n1, const ICBSSingleAgentLLNode* n2) const
		{
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
		}
	};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

	// the following is used to compare nodes in the FOCAL list
	struct secondary_compare_node 
	{
		bool operator()(const ICBSSingleAgentLLNode* n1, const ICBSSingleAgentLLNode* n2) const // returns true if n1 > n2
		{
			if (n1->num_internal_conf == n2->num_internal_conf)
			{
				return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
			}
			return n1->num_internal_conf >= n2->num_internal_conf;  // n1 > n2 if it has more conflicts
		}
	};  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)


	// define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
	typedef boost::heap::fibonacci_heap< ICBSSingleAgentLLNode*, boost::heap::compare<ICBSSingleAgentLLNode::compare_node> >
		::handle_type open_handle_t;
	typedef boost::heap::fibonacci_heap< ICBSSingleAgentLLNode*, boost::heap::compare<ICBSSingleAgentLLNode::secondary_compare_node> >
		::handle_type focal_handle_t;

	open_handle_t open_handle;
	focal_handle_t focal_handle;


	ICBSSingleAgentLLNode();
	ICBSSingleAgentLLNode(const ICBSSingleAgentLLNode& other);
	ICBSSingleAgentLLNode(int loc, int g_val, int h_val,
		ICBSSingleAgentLLNode* parent, int timestep,
		int num_internal_conf = 0, bool in_openlist = false);
	inline int getFVal() const { return g_val + h_val; }
	~ICBSSingleAgentLLNode();

	// The following is used by googledensehash for checking whether two nodes are equal
	// we say that two nodes, s1 and s2, are equal if
	// both agree on the location and timestep
	struct eqnode {
		bool operator()(const ICBSSingleAgentLLNode* s1, const ICBSSingleAgentLLNode* s2) const {
			return (s1 == s2) || (s1 && s2 &&
				s1->loc == s2->loc &&
				s1->timestep == s2->timestep);
		}
	};

	// The following is used by googledensehash for generating the hash value of a node
	struct NodeHasher
	{
		std::size_t operator()(const ICBSSingleAgentLLNode* n) const
		{
			size_t loc_hash = std::hash<int>()(n->loc);
			size_t timestep_hash = std::hash<int>()(n->timestep);
			return (loc_hash ^ (timestep_hash << 1));
		}
	};

};

std::ostream& operator<<(std::ostream& os, const ICBSSingleAgentLLNode& n);
