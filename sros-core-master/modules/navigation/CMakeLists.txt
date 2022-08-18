cmake_minimum_required(VERSION 2.8)
project(navigation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(osqp REQUIRED)
find_package(iplplus)
find_package(X11)

add_subdirectory(lib)
add_subdirectory(avoid_oba)

set(MODULE_FILES
        navigation_module.h
        navigation_module.cpp
        navigation_on_net.h
        navigation_on_net.cpp
        path_smooth/smoother.cpp path_smooth/smoother.h
        path_follow/path_follow.cpp
        hybrid_astar/collisiondetection.cpp hybrid_astar/hybrid_a_star.cpp  hybrid_astar/algorithm.cpp
        hybrid_astar/dubins.cpp hybrid_astar/node3d.cpp hybrid_astar/node2d.cpp
        hybrid_astar/reeds_shepp.cpp
        planning_fsm.cpp hybrid_astar/visualization.cpp
        local_map.cpp nav_config.h
        hybrid_astar/map_cell.cpp hybrid_astar/map_grid.cpp 
        check_collision.cpp check_collision.h speed_plan.cpp speed_plan.h
        reference_paths/reference_paths.cpp)

#path_follow/path_follow.cpp path_follow/path_follow.h planning_fsm.cpp planning_fsm.h

add_library(module-navigation ${MODULE_FILES})

target_link_libraries(module-navigation lib-navigation avoid_oba )
target_link_libraries(module-navigation ${iplplus_LIBRARIES} X11)
target_link_libraries(module-navigation libiplplus.so)
target_link_libraries(module-navigation PRIVATE osqp)


