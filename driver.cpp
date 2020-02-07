#include "map_loader.h"
#include "agents_loader.h"
#include "ICBSSearch.h"

#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/process.hpp>
#include <execinfo.h>
#include "g_logging.h"
#include <fstream>
#include <string>

#ifdef USE_GUROBI
#include <gurobi_c++.h>
#endif

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
		("map,m", po::value<std::string>()->required(), "path to the input .map file")
		("agents,a", po::value<std::string>()->required(), "path to the input .agents or .scen file")
		("output,o", po::value<std::string>()->required(), "path to the .csv output file")
		("agentNum,k", po::value<int>()->required(), "number of agents to read from the .scen file, or to generate")
		("warehouseWidth,b", po::value<int>()->default_value(0), "width of working stations on both sides, for generating instances")
		("heuristic,h", po::value<bool>()->default_value(true), "heuristics for the high-level")
		("split,p", po::value<std::string>()->default_value("NON_DISJOINT"), "Split Strategy (NON_DISJOINT, RANDOM, SINGLETONS, WIDTH, DISJOINT3,MVC_BASED)")
		("propagation", po::value<bool>()->default_value(true), "propagate positive constraints to narrow levels down the MDD")
		("prefer_f_cardinal", po::value<bool>()->default_value(true), "prefer f-cardinal conflicts")
		("prefer_goal_conflicts", po::value<bool>()->default_value(true), "prefer goal conflicts")
		("screen", po::value<int>()->default_value(0), "screen (0: only results; 1: details)")
		("cutoffTime", po::value<int>()->default_value(60), "cutoff time (seconds)")
		("seed", po::value<int>(), "random seed")
		("childPrefBudget", po::value<int>()->default_value(5), "Child preference budget")
		("maxChildPrefOptions", po::value<int>()->default_value(20), "Max child preference options")
		("focalW", po::value<float>()->default_value(1), "ECBS focal W")
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

	// read the map file and construct its two-dimensional array
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
    else if (vm["split"].as<string>() == "MVC_BASED")
        p = split_strategy::MVC_BASED;
	else
	{
		cout << "ERROR SPLIT STRATEGY!";
		return 0;
	}
	if (vm["split"].as<string>() != "NON_DISJOINT")
		cout << vm["split"].as<string>() << "+";

    try {
        ICBSSearch icbs(ml, al, vm["focalW"].as<float>(), p, vm["heuristic"].as<bool>(), vm["cutoffTime"].as<int>(),
                        vm["childPrefBudget"].as<int>(), vm["maxChildPrefOptions"].as<int>(),
                        vm["screen"].as<int>(), vm["propagation"].as<bool>(),
                        vm["prefer_f_cardinal"].as<bool>(), vm["prefer_goal_conflicts"].as<bool>());

        // run
    	bool success = icbs.runICBSSearch();
//        bool success = icbs.runIterativeDeepeningICBSSearch();
        // validate the solution
        if (success)
            icbs.isSolutionFeasible();
        // save data:
        // 1. Get max memory
        pid_t pid = getpid();
        char my_proc_status_filename[100];
        sprintf(my_proc_status_filename, "/proc/%d/status", pid);
        ifstream my_proc_status_file(my_proc_status_filename);
        string line;
        do {
            std::getline(my_proc_status_file, line);
        } while ((strncmp(line.data(), "VmPeak:", 7) != 0) && (line.size() != 0));
        if (line.size() != 0) {
            my_proc_status_file.close();
            char max_mem_chars[100];
            sscanf(line.data(), "VmPeak:\t%s kB", max_mem_chars);  // max_mem_chars will catch some leading spaces
            icbs.max_mem = max_mem_chars;
        } else
            icbs.max_mem = "VmPeak line not found :(";

        // 2. print results
        icbs.printResults();

        // 3. save results to file
    #ifndef LPA
        icbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>(), vm["split"].as<string>());
    #else
        icbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>(), vm["split"].as<string>()+"/LPA*");
    #endif
        if (success)
            return 0;
        else
            return 1;
    }
    catch (const GRBException& exception) {
        cerr << exception.getMessage() << endl;
        throw;
    }
}
