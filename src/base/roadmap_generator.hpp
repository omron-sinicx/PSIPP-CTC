/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#ifndef MAPF_BENCHMARK_ROADMAP_GENERATOR
#define MAPF_BENCHMARK_ROADMAP_GENERATOR

#include "base/input.hpp"
#include "base/spatial.hpp"

template <class SpaceGeometry>
class RoadmapGenerator {
 public:
  ProblemInstanceWithGeometry<typename SpaceGeometry::Geometry>
      problem_instance;
  virtual void generate(
      const SpatialProblemInstance<SpaceGeometry> &spacial_problem_instance) {
    fprintf(stderr, "No real generator is specified");
  }
};
#endif
