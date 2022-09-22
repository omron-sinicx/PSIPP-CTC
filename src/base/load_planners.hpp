/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#ifndef MAPF_BENCHMARK_LOAD_PLANNERS
#define MAPF_BENCHMARK_LOAD_PLANNERS

#include <yaml-cpp/yaml.h>

#include "base/planner.hpp"
std::shared_ptr<Planner> makePlanner(const YAML::Node& config);

#endif
