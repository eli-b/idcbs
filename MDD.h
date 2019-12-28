#pragma once
#include "ICBSSingleAgentLLSearch.h"

class MDDNode
{
public:
	MDDNode(int currloc, MDDNode* parent)
	{
		location = currloc; 
		if (parent == nullptr)
			level = 0;
		else
		{
			level = parent->level + 1;
			parents.push_back(parent);
		}
	}
	int location;
	int level;

	bool operator == (const MDDNode & node) const
	{
		return (this->location == node.location) && (this->level == node.level);
	}


	list<MDDNode*> children;
	list<MDDNode*> parents;
	MDDNode* parent;

    struct eqnode {
        bool operator()(const MDDNode* s1, const MDDNode* s2) const {
            return (s1 == s2) || (s1 && s2 &&
                                  s1->location == s2->location &&
                                  s1->level == s2->level);
        }
    };

    // The following is used by googledensehash for generating the hash value of a node
    struct NodeHasher
    {
        std::size_t operator()(const MDDNode* n) const
        {
            size_t loc_hash = std::hash<int>()(n->location);
            size_t timestep_hash = std::hash<int>()(n->level);
            return (loc_hash ^ (timestep_hash << 1));
        }
    };
};

class MDD
{
public:
	vector<list<MDDNode*>> levels;

	bool buildMDD(const std::vector < std::unordered_map<int, ConstraintState > >& cons_table,
		const pair<int, int> &start, const pair<int, int>&goal, int lookahead, const ICBSSingleAgentLLSearch & solver,
        clock_t end_by);
	bool updateMDD(const tuple<int, int, int> &constraint, int num_col);
	MDDNode* find(int location, int level);
	void deleteNode(MDDNode* node);
	void clear();
	void printMDD() const;
	

	MDD(){};
	MDD(MDD & cpy);
	~MDD();
};

