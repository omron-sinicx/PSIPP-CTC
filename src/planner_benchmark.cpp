/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include <cstdlib>

#include "base/benchmark.hpp"
#include "base/load_planners.hpp"
#include "base/load_problems.hpp"

int main(int argc, char** argv) {
  Benchmark benchmark;
  benchmark.iterations = 1;

  std::string path(realpath(__FILE__, NULL));
  path.erase(path.find_last_of('/'));
  YAML::Node config = YAML::LoadFile(
      argc > 1 ? argv[1] : path + "/../config/planner_benchmark.yaml");

  YAML::Node problems_data = config["Problems"];
  fprintf(stderr, "   problem   time(ms)\n");
  for (YAML::const_iterator it = problems_data.begin();
       it != problems_data.end(); ++it) {
    std::string name = it->first.as<std::string>();
    auto start_time = std::chrono::system_clock::now();
    auto problem = makeProblemInstance(it->second);
    auto making_time_duration = std::chrono::system_clock::now() - start_time;
    int making_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                          making_time_duration)
                          .count();
    fprintf(stderr, "%10s %10d\n", name.c_str(), making_time);
    benchmark.addProblemInstance(problem, name);
  }
  YAML::Node planners_data = config["Planners"];
  for (YAML::const_iterator it = planners_data.begin();
       it != planners_data.end(); ++it) {
    benchmark.addPlanner(makePlanner(it->second), it->first.as<std::string>());
  }
  if (config["Output"]) {
    benchmark.runAll(config["Output"].as<std::string>());
  } else {
    benchmark.runAll();
  }
  benchmark.displayAllResults();
  return 0;
}
