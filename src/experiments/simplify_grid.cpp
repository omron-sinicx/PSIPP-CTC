/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include "base/load_problems.hpp"

template <>
inline void SpatialProblemInstance<Space2D>::printTextFile(
    const std::string &file_path) const {
  FILE *output_file = fopen(file_path.c_str(), "w");
  if (output_file == NULL) {
    fprintf(stderr, "File cannot be opened\n");
    return;
  }
  fprintf(output_file, "%d\n", (int)space.inners().size());
  fprintf(output_file, "%d\n", (int)space.outer().size() - 1);
  for (int i = 0; i < space.outer().size() - 1; i++) {
    Geometry2D::printPoint(output_file, space.outer()[i]);
  }
  for (int j = 0; j < space.inners().size(); j++) {
    fprintf(output_file, "%d\n", (int)space.inners()[j].size() - 1);
    for (int i = 0; i < space.inners()[j].size() - 1; i++) {
      Geometry2D::printPoint(output_file, space.inners()[j][i]);
    }
  }
  fprintf(output_file, "%d %lf\n", (int)tasks.size(), agent_geometry);
  for (int k = 0; k < tasks.size(); k++) {
    Geometry2D::printPoint(output_file, tasks[k].initial_point);
    Geometry2D::printPoint(output_file, tasks[k].goal_point);
  }
  fclose(output_file);
};

template <>
inline void SpatialProblemInstance<Space2D>::simplifySpace(
    const double max_distance) {
  Space2D::Space temp;
  boost::geometry::simplify(space, temp, max_distance);
  space = temp;
};

int main(int argc, char **argv) {
  YAML::Node config = YAML::LoadFile("../config/simplify_grid.yaml");

  auto problem_instance = makeSpatialProblemInstance(config["Input"]);
  double max_distance = config["max_distance"].as<double>();
  problem_instance->simplifySpace(max_distance);
  int number_of_agents = config["number_of_agents"].as<int>();
  problem_instance->tasks.resize(number_of_agents);
  auto start_points = Space2D::getSepareteRandomPoints(
      problem_instance->space, problem_instance->agent_geometry,
      number_of_agents);
  auto goal_points = Space2D::getSepareteRandomPoints(
      problem_instance->space, problem_instance->agent_geometry,
      number_of_agents);
  for (int i = 0; i < number_of_agents; i++) {
    problem_instance->tasks[i].initial_point = start_points[i];
    problem_instance->tasks[i].goal_point = goal_points[i];
  }
  std::string output_file = config["Output"].as<std::string>();
  problem_instance->printTextFile(output_file);
  return 0;
}
