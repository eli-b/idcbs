#include "agents_loader.h"

#include <cstdlib>  // for atoi
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string/predicate.hpp>  // For ends_with

using namespace boost;
using namespace std;

int RANDOM_WALK_STEPS = 100000;

/// For .agents files, the agentsNum and width are for warehouse instance generation
/// For .scne files, the agentsNum
AgentsLoader::AgentsLoader(string fname, const MapLoader &ml, int agentsNum = 0, int width = 0)
{
	string line;

	if (ends_with(fname, ".agents")) {
		ifstream myfile(fname.c_str());

		if (myfile.is_open()) {
			getline(myfile, line);
			char_separator<char> sep(",");
			tokenizer<char_separator<char> > tok(line, sep);
			tokenizer<char_separator<char> >::iterator beg = tok.begin();
			this->num_of_agents = atoi((*beg).c_str());

			for (int i = 0; i < num_of_agents; i++) {
				getline(myfile, line);
				tokenizer<char_separator<char> > col_tok(line, sep);
				auto c_beg = col_tok.begin();
				pair<int, int> curr_pair;
				// read start [row,col] for agent i
				curr_pair.first = strtol((*c_beg).c_str(), nullptr, 10);
				c_beg++;
				curr_pair.second = strtol((*c_beg).c_str(), nullptr, 10);

				this->initial_locations.push_back(curr_pair);
				// read goal [row,col] for agent i
				c_beg++;
				curr_pair.first = strtol((*c_beg).c_str(), nullptr, 10);
				c_beg++;
				curr_pair.second = strtol((*c_beg).c_str(), nullptr, 10);

				this->goal_locations.push_back(curr_pair);
			}
			myfile.close();
		} else if (agentsNum > 0 && width == 0)  // Generate agents randomly
		{
			this->num_of_agents = agentsNum;
			vector<bool> starts(ml.rows * ml.cols, false);
			vector<bool> goals(ml.rows * ml.cols, false);
			// Choose random start locations
			for (int k = 0; k < agentsNum; k++) {
				int x = rand() % ml.rows, y = rand() % ml.cols;
				int start = x * ml.cols + y;
				if (!ml.my_map[start] && !starts[start]) {
					// update start
					this->initial_locations.push_back(make_pair(x, y));
					starts[start] = true;

					// random walk
					int loc = start;
					bool *temp_map = new bool[ml.rows * ml.cols];
					for (int walk = 0; walk < RANDOM_WALK_STEPS; walk++) {
						int directions[] = {0, 1, 2, 3, 4};
						random_shuffle(directions, directions + 5);
						int i = 0;
						for (; i < 5; i++) {
							int next_loc = loc + ml.moves_offset[directions[i]];
							if (0 <= next_loc && next_loc < ml.rows * ml.cols &&
								abs(next_loc % ml.cols - loc % ml.cols) < 2 &&
								!ml.my_map[next_loc]) {
								loc = next_loc;
								break;
							}
						}
					}
					// find goal
					bool flag = false;
					int goal = loc;
					while (!flag) {
						int directions[] = {0, 1, 2, 3, 4};
						random_shuffle(directions, directions + 5);
						int i = 0;
						for (; i < 5; i++) {
							int next_loc = goal + ml.moves_offset[directions[i]];
							if (0 <= next_loc && next_loc < ml.rows * ml.cols &&
								abs(next_loc % ml.cols - loc % ml.cols) < 2 &&
								!ml.my_map[next_loc]) {
								goal = next_loc;
								break;
							}
						}
						flag = true;
						if (goals[goal])
							flag = false;
					}
					//update goal
					this->goal_locations.push_back(make_pair(goal / ml.cols, goal % ml.cols));
					goals[goal] = true;
				} else {
					k--;
				}
			}
			saveToFile(fname);
		} else if (agentsNum > 0 && width > 0)  // Generate agents for a warehouse scenario
		{
			this->num_of_agents = agentsNum;
			vector<bool> starts(ml.rows * ml.cols, false);
			vector<bool> goals(ml.rows * ml.cols, false);
			// Choose random start locations
			for (int k = 0; k < agentsNum; k++) {
				int x = rand() % ml.rows, y = rand() % width;
				if (k % 2 == 0)
					y = ml.cols - y - 1;
				int start = x * ml.cols + y;
				if (!starts[start]) {
					// update start
					this->initial_locations.push_back(make_pair(x, y));
					starts[start] = true;
				} else {
					k--;
				}
			}
			// Choose random goal locations
			for (int k = 0; k < agentsNum; k++) {
				int x = rand() % ml.rows, y = rand() % width;
				if (k % 2 == 1)
					y = ml.cols - y - 1;
				int goal = x * ml.cols + y;
				if (!goals[goal]) {
					// update start
					this->goal_locations.push_back(make_pair(x, y));
					goals[goal] = true;
				} else {
					k--;
				}
			}
			saveToFile(fname);
		} else {
			cerr << "Missing .agents file!" << endl;
			abort();
		}
	}
	else if (ends_with(fname, ".scen")) {
		this->num_of_agents = agentsNum;
		ifstream myfile(fname.c_str());

		if (myfile.is_open()) {
			getline(myfile, line);
			char_separator<char> space(" ");
			tokenizer<char_separator<char> > first_line_tokens(line, space);
			auto first_line_token = first_line_tokens.begin();
			if ((*first_line_token) != "version") {
				cerr << "Bad .scen format in line 0. Expected \"version\"" << endl;
				abort();
			}
			++first_line_token;
			int version = strtol((*first_line_token).c_str(), nullptr, 10);
			if (version != 1) {
				cerr << "Only version 1 of scen files is currently supported" << endl;
				abort();
			}

			char_separator<char> tab("\t");
			for (int i = 0; i < num_of_agents; i++) {
				getline(myfile, line);
				tokenizer<char_separator<char> > agent_line_fields(line, tab);
				auto c_beg = agent_line_fields.begin();
				c_beg++;  // The first field is the block number
				c_beg++;  // The second field is the map file name
				c_beg++;  // The third field is the number of map rows
				c_beg++;  // The fourth field is the number of map columns

				pair<int, int> curr_pair;
				// read start [col,row] for agent i
				curr_pair.second = strtol((*c_beg).c_str(), nullptr, 10);
				c_beg++;
				curr_pair.first = strtol((*c_beg).c_str(), nullptr, 10);

				this->initial_locations.push_back(curr_pair);
				// read goal [col,row] for agent i
				c_beg++;
				curr_pair.second = strtol((*c_beg).c_str(), nullptr, 10);
				c_beg++;
				curr_pair.first = strtol((*c_beg).c_str(), nullptr, 10);

				this->goal_locations.push_back(curr_pair);
			}
			myfile.close();
		}
		else {
			cerr << "Missing .scen file!" << endl;
			abort();
		}
	}
	else {
		cerr << "Unsupported instance format " << fname << endl;
		abort();
	}

}

void AgentsLoader::printAgentsInitGoal () 
{
	cout << "AGENTS:" << endl;;
	for (int i=0; i<num_of_agents; i++)
	{
		cout << "Agent" << i << " : I=(" << initial_locations[i].first << "," << initial_locations[i].second << ") ; G=(" <<
			goal_locations[i].first << "," << goal_locations[i].second << ")" << endl;
	}
	cout << endl;
}


void AgentsLoader::saveToFile(std::string fname)
{
	ofstream myfile;
	myfile.open(fname);
	myfile << num_of_agents << endl;
	for (int i = 0; i < num_of_agents; i++)
		myfile << initial_locations[i].first << "," << initial_locations[i].second << ","
				<< goal_locations[i].first << "," << goal_locations[i].second << "," << endl;
	myfile.close();
}
