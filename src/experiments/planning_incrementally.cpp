/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include "base/benchmark.hpp"
#include "base/load_generators.hpp"
#include "base/load_planners.hpp"
#include "base/load_problems.hpp"

int main(int argc, char** argv) {
  Benchmark benchmark;
  benchmark.iterations = 1;
  std::string config_file(argc > 1 ? argv[1]
                                   : "../config/planning_incrementally.yaml");
  YAML::Node config = YAML::LoadFile(config_file);

  int iterations = config["Iterations"].as<int>();
  std::string roadmap_file = config["Roadmaps"].as<std::string>();
  for (int count = 0; count < iterations; count++) {
    int annotation_time;
    std::shared_ptr<ProblemInstanceWithGeometry<Geometry2D>> roadmap(
        new ProblemInstanceWithGeometry<Geometry2D>(roadmap_file +
                                                    std::to_string(count)));
    benchmark.addProblemInstance(roadmap,
                                 "Unannotated" + std::to_string(count));
    auto start_time = std::chrono::system_clock::now();
    ProblemInstanceWithExplicitCollisionsPtr annotated_roadmap(
        new ProblemInstanceWithExplicitCollisions(*roadmap));
    annotation_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::system_clock::now() - start_time)
                          .count();
    benchmark.addProblemInstance(annotated_roadmap,
                                 "Annotated" + std::to_string(count));
    printf("%d annotation: %d\n", count, annotation_time);
  }
  YAML::Node planners_data = config["Planners"];
  for (YAML::const_iterator it = planners_data.begin();
       it != planners_data.end(); ++it) {
    benchmark.addPlanner(makePlanner(it->second), it->first.as<std::string>());
  }
  YAML::Node pairs_to_run = config["PairsToRun"];
  for (auto it = pairs_to_run.begin(); it != pairs_to_run.end(); ++it) {
    benchmark.addPairToRun((*it)["Problem"].as<std::string>(),
                           (*it)["Planner"].as<std::string>());
  }
  std::vector<std::vector<std::pair<int, int>>> permutations;
  if (config["Permutations"]) {
    YAML::Node permutation_data = config["Permutations"];
    for (auto it = permutation_data.begin(); it != permutation_data.end();
         ++it) {
      std::vector<std::pair<int, int>> permutation((*it).size());
      for (int i = 0; i < permutation.size(); i++) {
        permutation[i].first = (*it)[i][0].as<int>();
        permutation[i].second = (*it)[i][1].as<int>();
      }
      permutations.push_back(permutation);
    }
  }
  benchmark.runIncrementallyAllPairs(iterations, permutations);
  return 0;
}
