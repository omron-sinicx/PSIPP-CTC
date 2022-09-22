/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include "base/geometry_2D_for_generator.hpp"
#include "base/load_generators.hpp"
#include "base/load_problems.hpp"

int main(int argc, char** argv) {
  YAML::Node config =
      YAML::LoadFile(argc > 1 ? argv[1] : "../config/make_roadmaps.yaml");

  auto generator = makeGenerator(config["Generator"]);
  auto problem_instance = makeSpatialProblemInstance(config["Input"]);
  int iterations = config["Iterations"].as<int>();
  int number_of_agents = config["number_of_agents"].as<int>();
  std::string output_file = config["Roadmaps"].as<std::string>();
  for (int count = 0; count < iterations; count++) {
    auto start_points = Space2D::getSepareteRandomPoints(
        problem_instance->space, problem_instance->agent_geometry,
        number_of_agents);
    auto goal_points = Space2D::getSepareteRandomPoints(
        problem_instance->space, problem_instance->agent_geometry,
        number_of_agents);
    problem_instance->tasks.resize(number_of_agents);
    for (int i = 0; i < number_of_agents; i++) {
      problem_instance->tasks[i].initial_point = start_points[i];
      problem_instance->tasks[i].goal_point = goal_points[i];
    }
    generator->generate(*problem_instance);
    problem_instance->checkRoadspaceValidity(generator->problem_instance);
    generator->problem_instance.print(
        output_file + (iterations == 1 ? "" : std::to_string(count)));
  }
  return 0;
}
