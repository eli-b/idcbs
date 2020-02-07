#include "MDD.h"

// Returns whether building was successful (false if it timed out)
bool MDD::buildMDD(const std::vector < std::unordered_map<int, ConstraintState > >& cons_table,
	const pair<int, int> &start, const pair<int, int>&goal, int lookahead, const ICBSSingleAgentLLSearch & solver, clock_t end_by)
{
	int numOfLevels = goal.second - start.second + lookahead + 1;
	int current_cost = goal.second - start.second;
	auto root = new MDDNode(start.first, nullptr); // Root
	queue<MDDNode*> open;
    typedef google::dense_hash_map<MDDNode*, MDDNode*, MDDNode::NodeHasher, MDDNode::eqnode> hashtable_t;
    hashtable_t closed;
    auto empty_node = new MDDNode(-1, nullptr);
    auto deleted_node = new MDDNode(-2, nullptr);
    closed.set_empty_key(empty_node);
    closed.set_deleted_key(deleted_node);
	open.push(root);
	closed[root] = root;
	levels.resize(numOfLevels);
	while (!open.empty())  // BFS from root node
	{
	    if (std::clock() > end_by) {
            cerr << "Timed out building an MDD!" << endl;
            break;
        }

		MDDNode* node = open.front();
		open.pop();

		if (node->level == numOfLevels - 1)
		{
			if (node->location != goal.first)
				continue;
			levels[numOfLevels - 1].push_back(node);
			break;
		}
		int heuristicBound = numOfLevels - node->level - 2; // We want (g + 1) + h <= f = numOfLevels - 1, so h <= numOfLevels - g. -1 because it's the bound of the _children_.
		if (heuristicBound < 0)
		{
			std::cout << "heuristic bound is negative!" << std::endl; // Will it happen?
			heuristicBound = 0;
		}

		for (int i = 0; i < 5; i++) // Try every possible move. We only add backward edges in this step.
		{
			int newLoc = node->location + solver.moves_offset[i];
			int next_timestep = node->level + 1 + start.second;
			if (0 <= newLoc && newLoc < solver.map_size && abs(newLoc % solver.num_col - node->location % solver.num_col) < 2)
			{
			    int newH = solver.getDifferentialHeuristic(newLoc, goal.first);
				if (newH > heuristicBound)
					continue;
				else if (solver.isConstrained(i, newLoc, next_timestep, cons_table))
					continue;

                auto childNode = new MDDNode(newLoc, node);
                auto it = closed.find(childNode);
				if (it != closed.end()) // If the child node exists
                {
				    delete childNode;
                    (*it).second->parents.push_back(node); // then add corresponding parent link to it
                }
				else // generate a new mdd node
				{
					open.push(childNode);
					closed[childNode] = childNode;
				}
			}
		}
	}
	if (levels[numOfLevels - 1].empty()) {  // Timed out or goal can't be reached in given number of steps
        for (auto node : closed)
	        delete node.second;
        return false;
	}

	// Backward sweep: levels[numOfLevels-1] is the only non-empty level at the moment, and contains the goal node
	for (int t = numOfLevels - 1; t > 0; t--)
	{
		for (auto it = levels[t].begin() ; it != levels[t].end() ; )
		{
            auto node = *it;
		    if (node->children.empty() && t != (numOfLevels - 1)) {  // Useless node
                // Delete the node
                delete node;
                // Delete it from the level and continue
                it = levels[t].erase(it);
                continue;
		    }

			for (auto parent : node->parents)
			{
				if (parent->children.empty()) // a new node in the mdd
				{
					levels[t - 1].push_back(parent);
				}
				parent->children.push_back(node); // add forward edge
			}
            ++it;
		}
	}

	return true;
}

bool MDD::updateMDD(const tuple<int, int, int> &constraint, int num_col)
{
	auto [loc1, loc2, t] = constraint;

	if (loc2 < 0) // edge constraint - TODO: explain this hack. Looks like when loc2<0, loc1 and (-loc2-1) are indices
	              //                         in cell enumeration, and otherwise loc1 and loc2 are row and column values.
	{
		for (list<MDDNode*>::iterator it = levels[t].begin(); it != levels[t].end(); ++it)
			if ((*it)->location == loc1)
				for (list<MDDNode*>::iterator child = (*it)->children.begin(); child != (*it)->children.end(); ++child)
					if ((*child)->location == -loc2 - 1)  // FIXME: HACK!
					{
						(*it)->children.erase(child);
						(*child)->parents.remove(*it);
						if ((*it)->children.empty())
                            deleteNode(*it);  // Careful! Will invalidate the it iterator, but we won't be using it again
						if ((*child)->parents.empty()) {
                            deleteNode(*child);  // Careful! Will invalidate the child iterator, but we won't be using it again
						}
						return true;
					}
	}
	else // vertex constraint
	{
		list<MDDNode*> ToDelete;
		for (auto node : levels[t])
			if (loc1 / num_col <= (node)->location / num_col  && node->location / num_col <= loc2 / num_col
				&& loc1 % num_col <= (node)->location % num_col  && node->location % num_col <= loc2 % num_col)
				ToDelete.push_back(node);
		for (auto node : ToDelete)
			deleteNode(node);
		return true;
	}
	return false;
}

void MDD::printMDD() const
{
	for (int i = 0; i < levels.size(); i++)
	{
		std::cout << "Time " << i << " : ";
		for (auto node : levels[i])
		{
			std::cout << node->location << ", ";
		}
		std:: cout << std::endl;
	}
}


void MDD::deleteNode(MDDNode* node)
{
	levels[node->level].remove(node);
	for (list<MDDNode*>::iterator child = node->children.begin(); child != node->children.end(); ++child)
	{
		(*child)->parents.remove(node);
		if ((*child)->parents.empty())
			deleteNode(*child);
	}
	for (list<MDDNode*>::iterator parent = node->parents.begin(); parent != node->parents.end(); ++parent)
	{
		(*parent)->children.remove(node);
		if ((*parent)->children.empty())
			deleteNode(*parent);
	}
}

void MDD::clear()
{
	if (levels.empty())
		return;
	for (int i = 0; i < levels.size(); i++)
	{
		for (list<MDDNode*>::iterator it = levels[i].begin(); it != levels[i].end(); ++it)
			delete *it;
	}
}

MDDNode* MDD::find(int location, int level) 
{
	if (level < levels.size())
		for (list<MDDNode*>::iterator it = levels[level].begin(); it != levels[level].end(); ++it)
			if ((*it)->location == location)
				return (*it);
	return nullptr;
}

MDD::MDD(MDD & cpy) // deep copy
{
	levels.resize(cpy.levels.size());
	MDDNode* root = new MDDNode(cpy.levels[0].front()->location, NULL);
	levels[0].push_back(root);
	for(int t = 0; t < levels.size() - 1; t++)
	{
		for (list<MDDNode*>::iterator node = levels[t].begin(); node != levels[t].end(); ++node)
		{
			MDDNode* cpyNode = cpy.find((*node)->location, (*node)->level);
			for (list<MDDNode*>::const_iterator cpyChild = cpyNode->children.begin(); cpyChild != cpyNode->children.end(); ++cpyChild)
			{
				MDDNode* child = find((*cpyChild)->location, (*cpyChild)->level);
				if (child == nullptr)
				{
					child = new MDDNode((*cpyChild)->location, (*node));
					levels[child->level].push_back(child);
					(*node)->children.push_back(child);
				}
				else
				{
					child->parents.push_back(*node);
					(*node)->children.push_back(child);
				}
			}
		}
		
	}
}

MDD::~MDD()
{
	clear();
}
