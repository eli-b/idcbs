#include "map_loader.h"
#include "agents_loader.h"
#include "ICBSSearch.h"

#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include "g_logging.h"

namespace pt = boost::property_tree;
using namespace std;

int main(int argc, char** argv) 
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("map,m", po::value<std::string>()->required(), "input file for map")
		("agents,a", po::value<std::string>()->required(), "input file for agents")
		("output,o", po::value<std::string>()->required(), "output file for schedule")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("warehouseWidth,b", po::value<int>()->default_value(0), "width of working stations on both sides, for generating instacnes")
		("heuristic,h", po::value<bool>()->default_value(true), "heuristics for the high-level")
		("split,p", po::value<std::string>()->default_value("NON_DISJOINT"), "Split Strategy (NON_DISJOINT, RANDOM, SINGLETONS, WIDTH, DISJOINT3)")		
		("propagation", po::value<bool>()->default_value(true), "propagating positive constraints for SINGLETONS and WIDTH")
		("screen", po::value<int>()->default_value(0), "screen (0: only results; 1: details)")
		("cutoffTime", po::value<int>()->default_value(300), "cutoff time (seconds)")
		("seed", po::value<int>()->default_value(0), "random seed")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);
	
	srand((int)time(0));

	// read the map file and construct its two-dim array
	MapLoader ml(vm["map"].as<string>());

	// read agents' start and goal locations
	AgentsLoader al(vm["agents"].as<string>(), ml, vm["agentNum"].as<int>(), vm["warehouseWidth"].as<int>());

	srand(vm["seed"].as<int>());

	
	split_strategy p;
	if (vm["split"].as<string>() == "NON_DISJOINT")
		p = split_strategy::NON_DISJOINT;
	else if (vm["split"].as<string>() == "RANDOM")
		p = split_strategy::RANDOM;
	else if (vm["split"].as<string>() == "SINGLETONS")
		p = split_strategy::SINGLETONS;
	else if (vm["split"].as<string>() == "WIDTH")
		p = split_strategy::WIDTH;
	else if (vm["split"].as<string>() == "DISJOINT3")
		p = split_strategy::DISJOINT3;
	else
	{
		cout << "ERROR SPLIT STRATEGY!";
		return 0;
	}
	if (vm["split"].as<string>() != "NON_DISJOINT")
		cout << vm["split"].as<string>() << "+";

	ICBSSearch icbs(ml, al, 1.0, p, vm["heuristic"].as<bool>(), vm["cutoffTime"].as<int>());
	icbs.screen = vm["screen"].as<int>();
	icbs.propagation = vm["propagation"].as<bool>();
	// run 
	icbs.runICBSSearch();
	// validate the solution
	icbs.isFeasible();
	// save data
	icbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>(), vm["split"].as<string>()+"/LPA*");


	return 0;

}
