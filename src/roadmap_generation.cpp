/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include <cstdlib>

#include "base/load_generators.hpp"
#include "base/load_problems.hpp"

int main(int argc, char** argv) {
  std::string path(realpath(__FILE__, NULL));
  path.erase(path.find_last_of('/'));
  YAML::Node config = YAML::LoadFile(
      argc > 1 ? argv[1] : path + "/../config/roadmap_generation.yaml");

  auto generator = makeGenerator(config["Generator"]);
  auto problem_instance = makeSpatialProblemInstance(config["Input"]);
  int iterations = 1;
  if (config["Iterations"]) {
    iterations = config["Iterations"].as<int>();
  }
  std::string output_file = config["Output"].as<std::string>();
  for (int count = 0; count < iterations; count++) {
    generator->generate(*problem_instance);
    generator->problem_instance.print(
        output_file + (iterations == 1 ? "" : std::to_string(count)));
    problem_instance->checkRoadspaceValidity(generator->problem_instance);
  }
  return 0;
}
