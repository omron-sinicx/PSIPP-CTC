cmake_minimum_required(VERSION 3.16)
project(MAPF_Benchmark)

add_compile_options(-g -O3 -std=c++14)
find_package(yaml-cpp)
find_package(CGAL)

set( CMAKE_BUILD_TYPE "Release")

# CCBS sources
set( CCBS_SOURCE_FILES 
  config.cpp
     tinyxml2.cpp
     xml_logger.cpp 
     map.cpp 
     heuristic.cpp 
     sipp.cpp 
     task.cpp
     cbs.cpp 
	 simplex/columnset.cpp
	 simplex/constraint.cpp
	 simplex/matrix.cpp
	 simplex/objectivefunction.cpp
	 simplex/simplex.cpp
	 simplex/variable.cpp)
set( CCBS_HEADER_FILES 
     const.h  
     tinyxml2.h
	 config.h
     xml_logger.h 
     map.h 
     heuristic.h 
     sipp.h 
     task.h
     cbs.h
	 structs.h
	 simplex/columnset.h
	 simplex/constraint.h
	 simplex/datamismatchexception.h
	 simplex/divisionbyzeroexception.h
	 simplex/indexoutofboundexception.h
	 simplex/matrix.h
	 simplex/matrixissingularexception.h
	 simplex/matrixnotsquareexception.h
     simplex/memoryreachedoutexception.h
     simplex/notavectorexception.h
     simplex/objectivefunction.h
     simplex/pilal.h
     simplex/pilalexceptions.h
	 simplex/simplex.h
	 simplex/simplexexceptions.h
	 simplex/sizemismatchexception.h
	 simplex/variable.h)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/third_party")

add_library(CCBS)
foreach(file IN LISTS CCBS_HEADER_FILES)
  target_sources(CCBS PRIVATE third_party/Continuous-CBS/${file})
endforeach(file)
foreach(file IN LISTS CCBS_SOURCE_FILES)
  target_sources(CCBS PRIVATE third_party/Continuous-CBS/${file})
endforeach(file)
target_sources(CCBS PRIVATE src/planners/WrappedContinuousCBS2.cpp)

add_library(PSIPP src/planners/PrioritizedSIPP.cpp)
add_library(load_problems src/base/load_problems.cpp)
add_library(load_planners src/base/load_planners.cpp)
add_library(load_generators src/base/load_generators.cpp)
add_library(CDT src/roadmap_generators/ConstrainedTriangulation.cpp)
target_link_libraries(CDT gmp mpfr)
target_link_libraries(load_problems yaml-cpp)
target_link_libraries(load_planners yaml-cpp PSIPP CCBS)
target_link_libraries(load_generators yaml-cpp CDT)

add_executable(planner_benchmark src/planner_benchmark.cpp)
target_link_libraries(planner_benchmark load_problems load_planners)
add_executable(roadmap_generation src/roadmap_generation.cpp)
target_link_libraries(roadmap_generation load_problems load_generators)
add_executable(geometry_2D_test src/test/geometry_2D_test.cpp)
add_executable(roadmap_benchmark src/roadmap_benchmark.cpp)
target_link_libraries(roadmap_benchmark load_problems load_generators PSIPP)
add_executable(explicit_collision_test src/test/explicit_collision_test.cpp)
add_executable(compare_geometric_and_explicit src/experiments/compare_geometric_and_explicit.cpp)
target_link_libraries(compare_geometric_and_explicit load_problems load_generators PSIPP)
add_executable(simplify_grid src/experiments/simplify_grid.cpp)
target_link_libraries(simplify_grid load_problems)
add_executable(planning_incrementally src/experiments/planning_incrementally.cpp)
target_link_libraries(planning_incrementally load_problems load_planners load_generators)
add_executable(make_roadmaps src/experiments/make_roadmaps.cpp)
target_link_libraries(make_roadmaps load_problems load_generators)
