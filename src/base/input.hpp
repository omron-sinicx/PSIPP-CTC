/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#ifndef MAPF_BENCHMARK_INPUT
#define MAPF_BENCHMARK_INPUT

#include "base/geometry_2D.hpp"
#include "base/problem_with_geometry.hpp"

template <>
inline ProblemInstanceWithGeometry<Geometry2D>::ProblemInstanceWithGeometry(
    const std::string &file_path) {
  FILE *input_file =
      file_path == "stdin" ? stdin : fopen(file_path.c_str(), "r");
  if (input_file == NULL) {
    fprintf(stderr, "Input file %s cannot be opened\n", file_path.c_str());
    return;
  }
  int number_of_vertice, number_of_edges, number_of_agents;
  fscanf(input_file, "%d%d%d", &number_of_vertice, &number_of_edges,
         &number_of_agents);
  coordinates.resize(number_of_vertice);
  for (int vertex_id = 0; vertex_id < number_of_vertice; vertex_id++) {
    double x, y;
    fscanf(input_file, "%lf%lf", &x, &y);
    coordinates[vertex_id] = Point(x, y);
  }
  edges.resize(number_of_vertice);
  for (int edge = 0; edge < number_of_edges; edge++) {
    int start, target;
    fscanf(input_file, "%d%d", &start, &target);
    edges[start].push_back(target);
  }
  tasks.resize(number_of_agents);
  for (int agent_id = 0; agent_id < number_of_agents; agent_id++) {
    fscanf(input_file, "%d%d", &tasks[agent_id].initial_vertex,
           &tasks[agent_id].goal_vertex);
  }
  fscanf(input_file, "%lf", &agent_geometry);
  if (input_file != stdin) {
    fclose(input_file);
  }
}

template <>
inline void ProblemInstanceWithGeometry<Geometry2D>::print(
    const std::string &file_path) {
  FILE *output_file =
      file_path == "stdout" ? stdout : fopen(file_path.c_str(), "w");
  if (output_file == NULL) {
    fprintf(stderr, "Ouput file %s cannot be opened\n", file_path.c_str());
    return;
  }
  int number_of_edges = 0;
  for (int i = 0; i < edges.size(); i++) {
    number_of_edges += edges[i].size();
  }
  fprintf(output_file, "%lu %d %lu\n", edges.size(), number_of_edges,
          tasks.size());
  for (int vertex_id = 0; vertex_id < coordinates.size(); vertex_id++) {
    fprintf(output_file, "%030.15lf %030.15lf\n", coordinates[vertex_id].x,
            coordinates[vertex_id].y);
  }
  for (int vertex_id = 0; vertex_id < edges.size(); vertex_id++) {
    for (int edge_id = 0; edge_id < edges[vertex_id].size(); edge_id++) {
      fprintf(output_file, "%d %d\n", vertex_id, edges[vertex_id][edge_id]);
    }
  }
  for (int agent_id = 0; agent_id < tasks.size(); agent_id++) {
    fprintf(output_file, "%d %d\n", tasks[agent_id].initial_vertex,
            tasks[agent_id].goal_vertex);
  }
  fprintf(output_file, "%030.15lf\n", agent_geometry);
  if (output_file != stdout) {
    fclose(output_file);
  }
}
#endif
