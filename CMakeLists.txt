cmake_minimum_required(VERSION 3.12)
project(disjoint_CBSH)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

link_libraries(boost_program_options
        boost_graph
        glog
        )

add_executable(disjoint_CBSH
        agents_loader.cpp
        agents_loader.h
        common.cpp
        common.h
        compute_heuristic.cpp
        compute_heuristic.h
        dynamic_constraints_manager.cpp
        dynamic_constraints_manager.h
        driver.cpp
        g_logging.h
        ICBSNode.cpp
        ICBSNode.h
        ICBSSearch.cpp
        ICBSSearch.h
        LLNode.cpp
        LLNode.h
        lpa_node.cpp
        lpa_node.h
        lpa_star.cpp
        lpa_star.h
        map_loader.cpp
        map_loader.h
        MDD.cpp
        MDD.h
        ICBSSingleAgentLLSearch.cpp
        ICBSSingleAgentLLSearch.h)
