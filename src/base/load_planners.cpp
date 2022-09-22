/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include "base/load_planners.hpp"

#include "planners/ExtendedIncreasingCostTreeSearch.cpp"
#include "planners/PrioritizedSIPP.cpp"
#include "planners/WrappedContinuousCBS2.hpp"

std::shared_ptr<Planner> makePlanner(const YAML::Node& config) {
  std::shared_ptr<Planner> planner;
  std::string planner_type = config["type"].as<std::string>();
  double time_limit = 30.;
  if (config["time_limit"]) {
    time_limit = config["time_limit"].as<double>();
  }
  if (planner_type == "CCBS") {
    planner = std::shared_ptr<WrappedContinuousCBS2>(
        new WrappedContinuousCBS2(time_limit));
  } else if (planner_type == "PSIPP") {
    planner =
        std::shared_ptr<PrioritizedSIPP>(new PrioritizedSIPP(0, time_limit));
  } else {
    throw std::runtime_error("Unknown Planner Type");
  }
  return planner;
}
