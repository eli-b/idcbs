#include "map_loader.h"
#include "agents_loader.h"
#include "ICBSSearch.h"

#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/process.hpp>
#include "g_logging.h"

namespace pt = boost::property_tree;
using namespace std;

int glog_v;

int main(int argc, char** argv) 
{
    // Init GLOG.
    FLAGS_logtostderr = 1;
    google::InitGoogleLogging(argv[0]);

	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("map,m", po::value<std::string>()->required(), "input file for map")
		("agents,a", po::value<std::string>()->required(), "input file for agents")
		("output,o", po::value<std::string>()->required(), "output file for schedule")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("warehouseWidth,b", po::value<int>()->default_value(0), "width of working stations on both sides, for generating instances")
		("heuristic,h", po::value<bool>()->default_value(true), "heuristics for the high-level")
		("split,p", po::value<std::string>()->default_value("NON_DISJOINT"), "Split Strategy (NON_DISJOINT, RANDOM, SINGLETONS, WIDTH, DISJOINT3)")		
		("propagation", po::value<bool>()->default_value(true), "propagate positive constraints to narrow levels down the MDD")
		("screen", po::value<int>()->default_value(0), "screen (0: only results; 1: details)")
		("cutoffTime", po::value<int>()->default_value(300), "cutoff time (seconds)")
		("seed", po::value<int>()->default_value(0), "random seed")
		("verbosity,v", po::value<int>(&glog_v)->default_value(0), "Set verbose logging level")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);

    FLAGS_v = glog_v;
	
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

	ICBSSearch icbs(ml, al, 1.0, p, vm["heuristic"].as<bool>(), vm["cutoffTime"].as<int>(), vm["screen"].as<int>());
	icbs.posConstraintsAlsoAddPosConstraintsOnMddNarrowLevelsLeadingToThem = vm["propagation"].as<bool>();
	// run 
	//icbs.runICBSSearch();
	icbs.runIterativeDeepeningICBSSearch();
	// validate the solution
	icbs.isFeasible();
	// save data:
	// 1. Get max memory
	pid_t pid = getpid();
	char cmd[200];
	sprintf(cmd, "cat /proc/%d/status | grep -i peak | xargs echo | cut -d' ' -f2", pid);
	string string_cmd(cmd);
	boost::process::ipstream pipe_stream;
	boost::process::child child_p("/bin/bash", std::vector<std::string> {"-c", cmd}, boost::process::std_out > pipe_stream);
	getline(pipe_stream, icbs.max_mem);
	child_p.wait();

    // 2. print results
	icbs.printResults();

	// 3. save results to file
#ifndef LPA
	icbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>(), vm["split"].as<string>());
#else
	icbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>(), vm["split"].as<string>()+"/LPA*");
#endif

	return 0;
}
