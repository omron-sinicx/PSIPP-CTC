/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include "base/benchmark.hpp"
#include "base/load_generators.hpp"
#include "base/load_problems.hpp"
#include "base/problem_with_explicit_collisions.hpp"
#include "planners/PrioritizedSIPP.hpp"

int main() {
  YAML::Node config =
      YAML::LoadFile("../config/compare_geometric_and_explicit.yaml");
  auto problem = makeSpatialProblemInstance(config["Input"]);
  std::shared_ptr<PrioritizedSIPP> PSIPP(new PrioritizedSIPP);
  YAML::Node generators_data = config["Generators"];
  for (YAML::const_iterator it = generators_data.begin();
       it != generators_data.end(); ++it) {
    std::string name = it->first.as<std::string>();
    auto generator = makeGenerator(it->second);
    generator->generate(*problem);
    auto geometric = std::make_shared<decltype(generator->problem_instance)>(
        generator->problem_instance);
    PSIPP->setInstance(geometric);
    RunPlanner run(PSIPP, geometric);
    if (!run.success) {
      continue;
    }
    int used_time = run.used_time,
        used_time_for_collisions = run.used_time_for_collisions;
    auto start_time = std::chrono::system_clock::now();
    std::shared_ptr<ProblemInstanceWithExplicitCollisions> explc(
        new ProblemInstanceWithExplicitCollisions(*geometric));
    auto making_time_duration = std::chrono::system_clock::now() - start_time;
    int making_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                          making_time_duration)
                          .count();
    PSIPP->setInstance(explc);
    RunPlanner rerun(PSIPP, explc);
    int used_time_explicit = rerun.used_time;
    printf("%s,%d,%d,%d,%d\n", name.c_str(), used_time,
           used_time_for_collisions, making_time, used_time_explicit);
  }
  return 0;
}
