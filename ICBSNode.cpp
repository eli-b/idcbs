#include "ICBSNode.h"

ICBSNode::ICBSNode(ICBSNode* parent) : parent(parent)
{
	g_val = parent->g_val;
	makespan = parent->makespan;
	depth = parent->depth + 1;
}

void ICBSNode::clear()
{
	cardinalConf.clear();
	semiConf.clear();
	nonConf.clear();
	unknownConf.clear();
}
