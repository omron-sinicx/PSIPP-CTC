/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#ifndef MAPF_BENCHMARK_PLANNER
#define MAPF_BENCHMARK_PLANNER

#include "base/problem.hpp"

class Planner {
 public:
  virtual void setInstance(ProblemInstancePtr problem_instance_ptr) = 0;
  virtual void solve() { fprintf(stderr, "No real planner is specified\n"); }
  virtual bool succeeded() = 0;
  virtual typename ProblemInstance::Plan getPlan() = 0;
};

using PlannerPtr = std::shared_ptr<Planner>;
#endif
