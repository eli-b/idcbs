#include "map_loader.h"

#include <limits>
#include <boost/tokenizer.hpp>
#include <libgen.h>

using namespace boost;
using namespace std;

MapLoader::MapLoader(int rows, int cols) {
  int i, j;
  this->rows = rows;
  this->cols = cols;
  GRID_COLS = cols;
  this->my_map = new bool[rows*cols];
  for (i=0; i<rows*cols; i++)
    this->my_map[i] = false;
  // Possible moves [WAIT, NORTH, EAST, SOUTH, WEST]
  moves_offset = new int[MapLoader::MOVE_COUNT];
  moves_offset[MapLoader::valid_moves_t::WAIT_MOVE] = 0;
  moves_offset[MapLoader::valid_moves_t::NORTH] = -cols;
  moves_offset[MapLoader::valid_moves_t::EAST] = 1;
  moves_offset[MapLoader::valid_moves_t::SOUTH] = cols;
  moves_offset[MapLoader::valid_moves_t::WEST] = -1;

  // Set the edges of the map to be an impassable border
  i = 0;
  for (j=0; j<cols; j++)
    this->my_map[linearize_coordinate(i,j)] = true;
  i=rows-1;
  for (j=0; j<cols; j++)
    this->my_map[linearize_coordinate(i,j)] = true;
  j=0;
  for (i=0; i<rows; i++)
    this->my_map[linearize_coordinate(i,j)] = true;
  j=cols-1;
  for (i=0; i<rows; i++)
    this->my_map[linearize_coordinate(i,j)] = true;

}

MapLoader::MapLoader(string fname) : name(fname) {
  string line;
  ifstream myfile (fname.c_str());
  if (myfile.is_open()) {
    getline(myfile, line);
    if (line != "type octile") {
        cerr << "Bad .map format. Expected first line to be \"type octile\" but got \"" << line << "\"" << endl;
        abort();
    }
    getline(myfile, line);
    char_separator<char> space(" ");
    tokenizer< char_separator<char> > height_line_tokens(line, space);
    auto field = height_line_tokens.begin();
    if (*field != "height") {
        cerr << "Bad .map format. Expected second line to start with \"height\" but got \"" << *field << "\"" << endl;
        abort();
    }
    ++field;
    this->rows = strtol((*field).c_str(), nullptr, 10); // read number of rows

    getline(myfile, line);
    tokenizer< char_separator<char> > width_line_tokens(line, space);
    field = width_line_tokens.begin();
    if (*field != "width") {
        cerr << "Bad .map format. Expected third line to start with \"width\" but got \"" << *field << "\"" << endl;
        abort();
    }
    ++field;
    this->cols = strtol((*field).c_str(), nullptr, 10); // read number of cols
    GRID_COLS = this->cols;

    getline(myfile, line);
    if (line != "map") {
        cerr << "Bad .map format. Expected fourth line to be \"map\" but got \"" << line << "\"" << endl;
        abort();
    }

    this->my_map =  new bool[rows*cols];
    for (int i=0 ; i < rows * cols ; i++)
        my_map[i] = false;
    // read map (and start/goal locations)
    for (int i=0 ; i < rows ; i++) {
		getline(myfile, line);
		for (int j=0 ; j < cols ; j++) {
		  my_map[cols*i + j] = (line[j] != '.');
		}
    }
    myfile.close();

    // initialize moves_offset array
    moves_offset = new int[MapLoader::MOVE_COUNT];
    moves_offset[MapLoader::valid_moves_t::WAIT_MOVE] = 0;
    moves_offset[MapLoader::valid_moves_t::NORTH] = -cols;
    moves_offset[MapLoader::valid_moves_t::EAST] = 1;
    moves_offset[MapLoader::valid_moves_t::SOUTH] = cols;
    moves_offset[MapLoader::valid_moves_t::WEST] = -1;
  }
  else
    cerr << "Map file not found." << std::endl;
}

char* MapLoader::mapToChar() {
  char* mapChar = new char[rows*cols];
  for (int i=0; i<rows*cols; i++) {
    if ( i == start_loc )
      mapChar[i] = 'S';
    else if ( i == goal_loc )
      mapChar[i] = 'G';
    else if (this->my_map[i] == true)
      mapChar[i] = '*';
    else
      mapChar[i] = ' ';
  }
  return mapChar;
}

void MapLoader::printMap () {
  char* mapChar = mapToChar();
  printMap (mapChar);
  delete[] mapChar;
}


void MapLoader::printMap (char* mapChar) {
  cout << "MAP:";
  for (int i=0; i<rows*cols; i++) {
    if (i % cols == 0)
      cout << endl;
    cout << mapChar[i];
  }
  cout << endl;
}

void MapLoader::printHeuristic(const double* mapH, const int agent_id) {
  cout << endl << "AGENT "<<agent_id<<":";
  for (int i=0; i<rows*cols; i++) {
    if (i % cols == 0)
      cout << endl;
    if (mapH[i] == std::numeric_limits<double>::max())
      cout << "*,";
    else
      cout << mapH[i] << ",";
  }
  cout << endl;
}

bool* MapLoader::copy_map() const {
  bool* retVal = new bool [ this->rows * this->cols ];
  memcpy (retVal, this->my_map, sizeof(bool)* this->rows * this->cols );
  return retVal;
}

MapLoader::~MapLoader() {
  delete[] this->my_map;
  delete[] this->moves_offset;
}

void MapLoader::saveToFile(std::string fname) {
  ofstream myfile;
  myfile.open (fname);
  myfile << rows << "," << cols << endl;
  for (int i=0; i<rows; i++) {
    for (int j=0; j<cols; j++) {
      if ( my_map[linearize_coordinate(i,j)] == true)
	myfile << "1";
      else
	myfile << "0";
      if (j<cols-1)
	myfile << ",";
      else
	myfile << endl;
    }
  }
  myfile.close();
}

void MapLoader::printPath(vector<int> path) {
  for (size_t i=0; i<path.size(); i++) {
    cout << "[" << row_coordinate(path[i]) << "," << col_coordinate(path[i]) << "] ; ";
  }
  cout << endl;
}
