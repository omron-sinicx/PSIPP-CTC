/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#ifndef MAPF_BENCHMARK_LOAD_GENERATORS
#define MAPF_BENCHMARK_LOAD_GENERATORS

#include <yaml-cpp/yaml.h>

#include "base/geometry_2D_for_generator.hpp"
#include "base/roadmap_generator.hpp"

std::shared_ptr<RoadmapGenerator<Space2D>> makeGenerator(
    const YAML::Node& config);

#endif
