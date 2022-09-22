/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include "base/benchmark.hpp"
#include "base/load_generators.hpp"
#include "base/load_problems.hpp"
#include "planners/PrioritizedSIPP.hpp"

int main(int argc, char** argv) {
  Benchmark benchmark;
  std::string path(realpath(__FILE__, NULL));
  path.erase(path.find_last_of('/'));
  YAML::Node config = YAML::LoadFile(
      argc > 1 ? argv[1] : path + "../config/roadmap_benchmark.yaml");

  YAML::Node generators_data = config["Generators"];
  for (YAML::const_iterator it = generators_data.begin();
       it != generators_data.end(); ++it) {
    std::string name = it->first.as<std::string>();
    auto generator = makeGenerator(it->second);
    benchmark.addGenerator(name, generator);
  }

  auto problem_instance = makeSpatialProblemInstance(config["Input"]);
  int bucket_size = config["Input"]["bucket_size"].as<int>();
  int number_of_buckets = problem_instance->tasks.size() / bucket_size;
  std::vector<std::vector<int>> subinstances(number_of_buckets);
  for (int i = 0; i < number_of_buckets; i++) {
    for (int j = 0; j < bucket_size; j++) {
      subinstances[i].push_back(bucket_size * i + j);
    }
  }
  bool precalc = config["Precalc"].as<bool>();
  std::shared_ptr<PrioritizedSIPP> planner(new PrioritizedSIPP);
  int iterations = config["Iterations"].as<int>();
  benchmark.RoadmapBenchmarkAll(problem_instance, subinstances, planner,
                                precalc, iterations);
  return 0;
}
