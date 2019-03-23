#pragma once
#include "SingleAgentICBS.h"

class MDDNode
{
public:
	MDDNode(int currloc, MDDNode* parent)
	{
		location = currloc; 
		if(parent == NULL)
			level = 0;
		else
		{
			level = parent->level + 1;
			parents.push_back(parent);
		}

		parent = NULL;
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
};

class MDD
{
public:
	vector<list<MDDNode*>> levels;

	int numPointers; //used to count how many pointers pointed to this
	bool buildMDD(const std::vector < std::unordered_map<int, ConstraintState > >& cons_table,
		const pair<int, int> &start, const pair<int, int>&goal, int lookahead, const SingleAgentICBS & solver);
	bool updateMDD(const tuple<int, int, int> &constraint, int num_col);
	MDDNode* find(int location, int level);
	void deleteNode(MDDNode* node);
	void clear();
	void printMDD() const;
	

	MDD(){};
	MDD(MDD & cpy);
	~MDD();
};

