#include "compute_heuristic.h"
#include "ICBSSingleAgentLLNode.h"

using google::dense_hash_map;      // namespace where class lives by default
using std::cout;
using std::endl;
using boost::heap::fibonacci_heap;


ComputeHeuristic::ComputeHeuristic(int start_location, int goal_location, 
	const bool* my_map, int map_rows, int map_cols, const int* moves_offset) :
    my_map(my_map), map_rows(map_rows), map_cols(map_cols), moves_offset(moves_offset),
	start_location(start_location), goal_location(goal_location){}

void ComputeHeuristic::getHVals(vector<int>& res)
{
	int root_location = goal_location;
	res.resize(map_rows * map_cols);
	for (int i = 0; i < map_rows * map_cols; i++)
		res[i] = INT_MAX;
	// generate a heap that can save nodes (and a open_handle)
	boost::heap::fibonacci_heap< ICBSSingleAgentLLNode*, boost::heap::compare<ICBSSingleAgentLLNode::compare_node> > heap;
	boost::heap::fibonacci_heap< ICBSSingleAgentLLNode*, boost::heap::compare<ICBSSingleAgentLLNode::compare_node> >::handle_type open_handle;
	// generate hash_map (key is a node pointer, data is a node handler,
	//                    NodeHasher is the hash function to be used,
	//                    eqnode is used to break ties when hash values are equal)
	dense_hash_map<ICBSSingleAgentLLNode*, fibonacci_heap<ICBSSingleAgentLLNode*, boost::heap::compare<ICBSSingleAgentLLNode::compare_node> >::handle_type, ICBSSingleAgentLLNode::NodeHasher, ICBSSingleAgentLLNode::eqnode> nodes;
	nodes.set_empty_key(NULL);
	dense_hash_map<ICBSSingleAgentLLNode*, fibonacci_heap<ICBSSingleAgentLLNode*, boost::heap::compare<ICBSSingleAgentLLNode::compare_node> >::handle_type, ICBSSingleAgentLLNode::NodeHasher, ICBSSingleAgentLLNode::eqnode>::iterator it; // will be used for find()

	ICBSSingleAgentLLNode* root = new ICBSSingleAgentLLNode(root_location, 0, 0, NULL, 0);
	root->open_handle = heap.push(root);  // add root to heap
	nodes[root] = root->open_handle;       // add root to hash_table (nodes)
	while (!heap.empty()) {
		ICBSSingleAgentLLNode* curr = heap.top();
		heap.pop();
		// cout << endl << "CURRENT node: " << curr << endl;
		for (int direction = 0; direction < 5; direction++)
		{
			int next_loc = curr->loc + moves_offset[direction];
			if (0 <= next_loc && next_loc < map_rows * map_cols && !my_map[next_loc] &&
				abs(next_loc % map_cols - curr->loc % map_cols) < 2)
			{  // if that grid is not blocked
				int next_g_val = (int) curr->g_val + 1;
				ICBSSingleAgentLLNode* next = new ICBSSingleAgentLLNode(next_loc, next_g_val, 0, NULL, 0);
				it = nodes.find(next);
				if (it == nodes.end()) {  // add the newly generated node to heap and hash table
					next->open_handle = heap.push(next);
					nodes[next] = next->open_handle;
				}
				else {  // update existing node's g_val if needed (only in the heap)
					delete(next);  // not needed anymore -- we already generated it before
					ICBSSingleAgentLLNode* existing_next = (*it).first;
					open_handle = (*it).second;
					if (existing_next->g_val > next_g_val) {
						existing_next->g_val = next_g_val;
						heap.update(open_handle);
					}
				}
			}
		}
	}
	// iterate over all nodes
	for (it = nodes.begin(); it != nodes.end(); it++) {
		ICBSSingleAgentLLNode* s = (*it).first;
		res[s->loc] = (int) s->g_val;
		delete (s);
	}
	nodes.clear();
	heap.clear();
}

void ComputeHeuristic::getAllPairsHVals(vector<vector<int>>& res)
{
	int map_size = map_rows * map_cols;
	res.resize(map_size);
	for (int root_location = 0; root_location < map_size; root_location++)
	{
		if (my_map[root_location])
			continue;
		res[root_location].resize(map_size, INT_MAX);

		// generate a heap that can save nodes (and a open_handle)
		boost::heap::fibonacci_heap< ICBSSingleAgentLLNode*, boost::heap::compare<ICBSSingleAgentLLNode::compare_node> > heap;
		boost::heap::fibonacci_heap< ICBSSingleAgentLLNode*, boost::heap::compare<ICBSSingleAgentLLNode::compare_node> >::handle_type open_handle;
		// generate hash_map (key is a node pointer, data is a node handler,
		//                    NodeHasher is the hash function to be used,
		//                    eqnode is used to break ties when hash values are equal)
		dense_hash_map<ICBSSingleAgentLLNode*, fibonacci_heap<ICBSSingleAgentLLNode*, boost::heap::compare<ICBSSingleAgentLLNode::compare_node> >::handle_type, ICBSSingleAgentLLNode::NodeHasher, ICBSSingleAgentLLNode::eqnode> nodes;
		nodes.set_empty_key(NULL);
		dense_hash_map<ICBSSingleAgentLLNode*, fibonacci_heap<ICBSSingleAgentLLNode*, boost::heap::compare<ICBSSingleAgentLLNode::compare_node> >::handle_type, ICBSSingleAgentLLNode::NodeHasher, ICBSSingleAgentLLNode::eqnode>::iterator it; // will be used for find()

		ICBSSingleAgentLLNode* root = new ICBSSingleAgentLLNode(root_location, 0, 0, NULL, 0);
		root->open_handle = heap.push(root);  // add root to heap
		nodes[root] = root->open_handle;       // add root to hash_table (nodes)
		while (!heap.empty()) {
			ICBSSingleAgentLLNode* curr = heap.top();
			heap.pop();
			for (int direction = 0; direction < 5; direction++)
			{
				int next_loc = curr->loc + moves_offset[direction];
				if (0 <= next_loc && next_loc < map_rows * map_cols && !my_map[next_loc] &&
					abs(next_loc % map_cols - curr->loc % map_cols) < 2)
				{  // if that grid is not blocked
					int next_g_val = (int) curr->g_val + 1;
					ICBSSingleAgentLLNode* next = new ICBSSingleAgentLLNode(next_loc, next_g_val, 0, NULL, 0);
					it = nodes.find(next);
					if (it == nodes.end()) {  // add the newly generated node to heap and hash table
						next->open_handle = heap.push(next);
						nodes[next] = next->open_handle;
					}
					else {  // update existing node's g_val if needed (only in the heap)
						delete(next);  // not needed anymore -- we already generated it before
						ICBSSingleAgentLLNode* existing_next = (*it).first;
						open_handle = (*it).second;
						if (existing_next->g_val > next_g_val) {
							existing_next->g_val = next_g_val;
							heap.update(open_handle);
						}
					}
				}
			}
		}
		// iterate over all nodes
		for (it = nodes.begin(); it != nodes.end(); it++) {
			ICBSSingleAgentLLNode* s = (*it).first;
			res[root_location][s->loc] = (int) s->g_val;
			delete (s);
		}
		nodes.clear();
		heap.clear();
	}
	
}
