/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#ifndef MAPF_BENCHMARK_SPACIAL_PLANNER
#define MAPF_BENCHMARK_SPACIAL_PLANNER

#include "base/spatial.hpp"

template <class SpaceGeometry>
class SpatialPlanner {
 public:
  virtual void setInstanceMap(
      const SpatialProblemInstancePtr<SpaceGeometry>& problem_instance) = 0;
  virtual void setInstanceTasks(
      const SpatialProblemInstancePtr<SpaceGeometry>& problem_instance) = 0;

  virtual void solve() = 0;
  virtual bool succeeded() = 0;

  virtual typename SpatialProblemInstance<SpaceGeometry>::Plan getPlan() = 0;
};

template <class SpaceGeometry>
using SpatialPlannerPtr = std::shared_ptr<SpatialPlanner<SpaceGeometry>>;
#endif
