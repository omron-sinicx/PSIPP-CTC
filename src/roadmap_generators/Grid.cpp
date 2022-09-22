/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include <set>

#include "base/geometry_2D_for_generator.hpp"
#include "base/roadmap_generator.hpp"

class GridGenerator : public RoadmapGenerator<Space2D> {
  using Point = typename Space2D::Point;

 public:
  double grid_size;
  int K;
  std::vector<std::pair<int, int>> neighbors;
  GridGenerator(const double grid_size, const int K) {
    this->grid_size = grid_size;
    this->K = K;
    neighbors =
        std::vector<std::pair<int, int>>{{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    for (int k = 3; k <= K; k++) {
      std::vector<std::pair<int, int>> new_neighbors;
      for (int i = 0; i < neighbors.size(); i++) {
        new_neighbors.push_back(neighbors[i]);
        int j = (i + 1) % neighbors.size();
        new_neighbors.push_back(
            std::make_pair(neighbors[i].first + neighbors[j].first,
                           neighbors[i].second + neighbors[j].second));
      }
      neighbors = new_neighbors;
    }
  }

  void generate(const SpatialProblemInstance<Space2D>& spacial_problem_instance)
      override {
    auto& problem_instance = RoadmapGenerator<Space2D>::problem_instance;
    auto& space = spacial_problem_instance.space;
    auto& tasks = spacial_problem_instance.tasks;

    auto x_bound = spacial_problem_instance.getBound(0);
    auto y_bound = spacial_problem_instance.getBound(1);
    int x_width =
        std::floor((x_bound.second - x_bound.first) / grid_size + 0.5);
    int y_width =
        std::floor((y_bound.second - y_bound.first) / grid_size + 0.5);
    std::map<std::pair<int, int>, int> vertex_ids;
    for (int i = 0; i < x_width; i++) {
      double x = x_bound.first + (i + 0.5) * grid_size;
      for (int j = 0; j < y_width; j++) {
        double y = y_bound.first + (j + 0.5) * grid_size;
        if (spacial_problem_instance.isInside(Point(x, y))) {
          vertex_ids[std::make_pair(i, j)] =
              problem_instance.coordinates.size();
          problem_instance.coordinates.push_back(Point(x, y));
        }
      }
    }
    problem_instance.edges.resize(problem_instance.coordinates.size() +
                                  2 * tasks.size());
    for (int i = 0; i < x_width; i++) {
      for (int j = 0; j < y_width; j++) {
        auto ids_it = vertex_ids.find(std::make_pair(i, j));
        if (ids_it == vertex_ids.end()) continue;
        int id = ids_it->second;

        for (int v = 0; v < neighbors.size() / 2; v++) {
          int neighbor_i = i + neighbors[v].first;
          int neighbor_j = j + neighbors[v].second;
          ids_it = vertex_ids.find(std::make_pair(neighbor_i, neighbor_j));
          if (ids_it == vertex_ids.end()) continue;
          int neighbor_id = ids_it->second;

          if (spacial_problem_instance.isInside(
                  problem_instance.coordinates[id],
                  problem_instance.coordinates[neighbor_id])) {
            problem_instance.addEdge(id, neighbor_id);
            problem_instance.addEdge(neighbor_id, id);
          }
        }
      }
    }
    problem_instance.tasks.resize(tasks.size());
    int dx[4] = {0, 0, 1, 1}, dy[4] = {0, 1, 0, 1};
    for (int agent = 0; agent < tasks.size(); agent++) {
      int start_id = problem_instance.coordinates.size();
      problem_instance.coordinates.push_back(tasks[agent].initial_point);
      int start_i = std::floor(
          (tasks[agent].initial_point.x - x_bound.first) / grid_size - 0.5);
      int start_j = std::floor(
          (tasks[agent].initial_point.y - y_bound.first) / grid_size - 0.5);
      for (int v = 0; v < 4; v++) {
        int i = start_i + dx[v], j = start_j + dy[v];
        auto ids_it = vertex_ids.find(std::make_pair(i, j));
        if (ids_it == vertex_ids.end()) continue;
        if (spacial_problem_instance.isInside(
                problem_instance.coordinates[start_id],
                problem_instance.coordinates[ids_it->second])) {
          problem_instance.addEdge(start_id, ids_it->second);
        }
      }
      int goal_id = problem_instance.coordinates.size();
      problem_instance.coordinates.push_back(tasks[agent].goal_point);
      int goal_i = std::floor(
          (tasks[agent].goal_point.x - x_bound.first) / grid_size - 0.5);
      int goal_j = std::floor(
          (tasks[agent].goal_point.y - y_bound.first) / grid_size - 0.5);
      for (int v = 0; v < 4; v++) {
        int i = goal_i + dx[v], j = goal_j + dy[v];
        auto ids_it = vertex_ids.find(std::make_pair(i, j));
        if (ids_it == vertex_ids.end()) continue;
        if (spacial_problem_instance.isInside(
                problem_instance.coordinates[ids_it->second],
                problem_instance.coordinates[goal_id])) {
          problem_instance.addEdge(ids_it->second, goal_id);
        }
      }
      problem_instance.tasks[agent].initial_vertex = start_id;
      problem_instance.tasks[agent].goal_vertex = goal_id;
    }
    problem_instance.agent_geometry = spacial_problem_instance.agent_geometry;
  }
};
