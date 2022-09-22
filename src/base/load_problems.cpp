/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include "base/load_problems.hpp"

#include "base/geometry_2D_for_generator.hpp"
#include "base/input.hpp"
#include "base/input_spatial.hpp"
#include "base/problem_with_explicit_collisions.hpp"
#include "base/problem_with_grid.hpp"
#include "base/problem_with_turning.hpp"
#include "base/spatial.hpp"

std::shared_ptr<SpatialProblemInstance<Space2D>> makeSpatialProblemInstance(
    const YAML::Node& config) {
  std::shared_ptr<SpatialProblemInstance<Space2D>> problem_instance(
      new SpatialProblemInstance<Space2D>());
  std::string input_type = config["type"].as<std::string>();

  if (input_type == "Text") {
    std::string input_file = config["file"].as<std::string>();
    problem_instance->loadTextFile(input_file);
  } else if (input_type == "MAPF_Benchmark") {
    std::string map_file = config["map"].as<std::string>();
    problem_instance->loadGripMap(map_file);
    problem_instance->agent_geometry = config["agent_size"].as<double>();
    if (config["scenario"]) {
      std::string scenario_file = config["scenario"].as<std::string>();
      problem_instance->loadScenario(scenario_file);
    }
  } else {
    throw std::runtime_error("Unknown Input Type");
  }
  return problem_instance;
}

std::shared_ptr<ProblemInstance> makeProblemInstance(const YAML::Node& config) {
  std::shared_ptr<ProblemInstance> problem_instance;
  std::string problem_type = config["type"].as<std::string>();

  if (problem_type == "Geometric") {
    std::string input_file = config["file"].as<std::string>();
    problem_instance = std::shared_ptr<ProblemInstanceWithGeometry<Geometry2D>>(
        new ProblemInstanceWithGeometry<Geometry2D>(input_file));
  } else if (problem_type == "Explicit") {
    std::string input_file = config["file"].as<std::string>();
    ProblemInstanceWithGeometry<Geometry2D> geometric(input_file);
    auto temp = std::shared_ptr<ProblemInstanceWithExplicitCollisions>(
        new ProblemInstanceWithExplicitCollisions(geometric));
    problem_instance = temp;
  } else if (problem_type == "Turning") {
    std::string input_file = config["file"].as<std::string>();
    ProblemInstanceWithGeometry<Geometry2D> geometric(input_file);
    ProblemInstanceWithExplicitCollisions annotated(geometric);
    double turning_speed = config["turning_speed"].as<double>(),
           turning_offset = config["turning_offset"].as<double>(),
           move_offset = config["move_offset"].as<double>();
    auto temp = ProblemInstanceWithTurningPtr(new ProblemInstanceWithTurning(
        annotated, turning_speed, turning_offset, move_offset));
    problem_instance = temp;
  } else if (problem_type == "Grid") {
    std::string map_file = config["map"].as<std::string>();
    auto grid = std::move(loadMAPFBGrid(map_file));
    std::string scenario_file = config["scenario"].as<std::string>();
    auto tasks = std::move(loadMAPFBScenario(scenario_file));
    double agent_geometry = config["agent_size"].as<double>();
    int K = config["K"].as<int>();
    problem_instance = ProblemInstanceWithGridPtr(
        new ProblemInstanceWithGrid(K, grid, agent_geometry, tasks));
  } else {
    throw std::runtime_error("Unknown Problem Type");
  }
  if (config["number_of_agents"]) {
    problem_instance->tasks.resize(config["number_of_agents"].as<int>());
  }
  return problem_instance;
}
