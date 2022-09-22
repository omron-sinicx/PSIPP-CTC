/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include "base/planner.hpp"

class WrappedContinuousCBS2 : public Planner {
 public:
  WrappedContinuousCBS2(const double time_limit = 30.);
  ~WrappedContinuousCBS2();
  void setInstance(ProblemInstancePtr problem_instance_ptr) override;
  void solve() override;
  bool succeeded() override;
  ProblemInstance::Plan getPlan() override;
};
