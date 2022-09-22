/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#ifndef MAPF_BENCHMARK_LOAD_PROBLEMS
#define MAPF_BENCHMARK_LOAD_PROBLEMS

#include <yaml-cpp/yaml.h>

#include "base/geometry_2D_for_generator.hpp"
#include "base/problem.hpp"
#include "base/roadmap_generator.hpp"
#include "base/spatial.hpp"
#include "base/spatial_planner.hpp"

std::shared_ptr<SpatialProblemInstance<Space2D>> makeSpatialProblemInstance(
    const YAML::Node& config);
std::shared_ptr<ProblemInstance> makeProblemInstance(const YAML::Node& config);
std::shared_ptr<SpatialPlanner<Space2D>> makeSpatialPlanner(
    const YAML::Node& config);

#endif
