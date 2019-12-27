#include "MDD.h"

bool MDD::buildMDD(const std::vector < std::unordered_map<int, ConstraintState > >& cons_table,
	const pair<int, int> &start, const pair<int, int>&goal, int lookahead, const ICBSSingleAgentLLSearch & solver)
{
	int numOfLevels = goal.second - start.second + lookahead + 1;
	int current_cost = goal.second - start.second;
	MDDNode* root = new MDDNode(start.first, nullptr); // Root
	queue<MDDNode*> open;
	list<MDDNode*> closed;
	open.push(root);
	closed.push_back(root);
	levels.resize(numOfLevels);
	while (!open.empty()) //BFS from root node
	{
		MDDNode* node = open.front();
		open.pop();

		if (node->level == numOfLevels - 1)
		{
			if (node->location != goal.first)
				continue;
			levels[numOfLevels - 1].push_back(node);
			break;
		}
		int heuristicBound = numOfLevels - node->level - 2; // We want (g + 1)+h <= f = numOfLevels - 1, so h <= numOfLevels - g. -1 because it's the bound of the _children_.
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
				if (solver.getDifferentialHeuristic(newLoc, goal.first) > heuristicBound)
					continue;
				else if (solver.isConstrained(i, newLoc, next_timestep, cons_table))
					continue;

				list<MDDNode*>::reverse_iterator child = closed.rbegin();
				bool find = false;
				for (; child != closed.rend() && ((*child)->level == node->level + 1); ++child)
					if ((*child)->location == newLoc) // If the child node exists
					{
						(*child)->parents.push_back(node); // then add corresponding parent link and child link
						find = true;
						break;
					}
				if (!find) // Else generate a new mdd node
				{
					MDDNode* childNode = new MDDNode(newLoc, node);
					open.push(childNode);
					closed.push_back(childNode);
				}
			}
		}
	}
	// Backward
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

	if (loc2 < 0) // Edge constraint
	{
		for (list<MDDNode*>::iterator it = levels[t].begin(); it != levels[t].end(); ++it)
			if ((*it)->location == loc1)
				for (list<MDDNode*>::iterator child = (*it)->children.begin(); child != (*it)->children.end(); ++child)
					if ((*child)->location == -loc2 - 1)
					{
						(*it)->children.erase(child);
						(*child)->parents.remove(*it);
						if ((*it)->children.empty())
							deleteNode(*it);
						if ((*child)->parents.empty())
							deleteNode(*child);
						return true;
					}
	}
	else // Vertex constraint
	{
		list<MDDNode*> ToDelete;
		for (list<MDDNode*>::iterator it = levels[t].begin(); it != levels[t].end(); ++it)
			if (loc1 / num_col <= (*it)->location / num_col  && (*it)->location / num_col <= loc2 / num_col
				&& loc1 % num_col <= (*it)->location % num_col  && (*it)->location % num_col <= loc2 % num_col)
				ToDelete.push_back(*it);
		for (list<MDDNode*>::iterator it = ToDelete.begin(); it !=ToDelete.end(); ++it)
			deleteNode(*it);
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
