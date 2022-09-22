/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#ifndef MAPF_BENCHMARK_PROBLEM_WITH_TURNING
#define MAPF_BENCHMARK_PROBLEM_WITH_TURNING

#include <algorithm>
#include <string>
#include <vector>

#include "base/problem_with_explicit_collisions.hpp"

class ProblemInstanceWithTurning
    : public ProblemInstanceWithExplicitCollisions {
 public:
  double turning_speed, turning_offset, move_offset;

  std::vector<std::vector<int>> original_edges;
  std::vector<std::vector<double>> edge_costs;
  std::vector<int> original_vertices;
  std::vector<std::vector<std::pair<double, int>>> connected_edges;

  std::vector<std::vector<int>> corresponding_vertices;
  std::vector<std::vector<std::pair<int, int>>> stay_edges;
  std::vector<std::vector<std::pair<int, int>>> corresponding_edges;

 private:
  void addEdge(const int source, const int target, const double length) {
    edges[source].push_back(target);
    edge_costs[source].push_back(length);
  }

 public:
  ProblemInstanceWithTurning(ProblemInstanceWithExplicitCollisions& problem,
                             const double turning_speed,
                             const double turning_offset,
                             const double move_offset) {
    const double PI = acos(-1);

    this->turning_speed = turning_speed;
    this->turning_offset = turning_offset;
    this->move_offset = move_offset;

    this->coordinates = problem.coordinates;
    this->edges = problem.edges;
    this->agent_geometry = problem.agent_geometry;
    original_edges = problem.edges;
    number_of_vertices = 0;

    int number_of_original_vertices = original_edges.size();
    edges.clear();
    edge_costs.clear();
    coordinates.clear();
    connected_edges.clear();
    connected_edges.resize(number_of_original_vertices);
    corresponding_vertices.clear();
    corresponding_vertices.resize(number_of_original_vertices);
    stay_edges.clear();
    stay_edges.resize(number_of_original_vertices);
    corresponding_edges.clear();
    corresponding_edges.resize(number_of_original_vertices);

    for (int source = 0; source < number_of_original_vertices; source++) {
      for (int edge = 0; edge < original_edges[source].size(); edge++) {
        int target = original_edges[source][edge];
        Point vec = problem.coordinates[target] - problem.coordinates[source];
        double angle = std::atan2(vec.y, vec.x);
        connected_edges[source].push_back(
            std::make_pair(angle, number_of_vertices));
        connected_edges[target].push_back(
            std::make_pair(-angle, number_of_vertices + 3));

        edges.resize(number_of_vertices + 4);
        edge_costs.resize(number_of_vertices + 4);
        addEdge(number_of_vertices, number_of_vertices + 1,
                move_offset + turning_offset);
        addEdge(number_of_vertices + 1, number_of_vertices + 2,
                problem.getEdgeLength(source, edge));
        addEdge(number_of_vertices + 2, number_of_vertices + 3, move_offset);
        coordinates.resize(number_of_vertices + 4);
        coordinates[number_of_vertices] = problem.coordinates[source];
        coordinates[number_of_vertices + 1] = problem.coordinates[source];
        coordinates[number_of_vertices + 2] = problem.coordinates[target];
        coordinates[number_of_vertices + 3] = problem.coordinates[target];

        corresponding_vertices[source].push_back(number_of_vertices);
        corresponding_vertices[source].push_back(number_of_vertices + 1);
        corresponding_vertices[target].push_back(number_of_vertices + 2);
        corresponding_vertices[target].push_back(number_of_vertices + 3);

        stay_edges[source].push_back(std::make_pair(
            number_of_vertices, edges[number_of_vertices].size() - 1));
        stay_edges[target].push_back(std::make_pair(
            number_of_vertices + 2, edges[number_of_vertices + 2].size() - 1));

        corresponding_edges[source].push_back(std::make_pair(
            number_of_vertices + 1, edges[number_of_vertices + 1].size() - 1));

        number_of_vertices += 4;
      }
    }
    tasks.clear();
    for (auto& task : problem.tasks) {
      int initial = task.initial_vertex, goal = task.goal_vertex;
      connected_edges[initial].push_back(
          std::make_pair(0., number_of_vertices));
      connected_edges[goal].push_back(
          std::make_pair(0., number_of_vertices + 1));
      coordinates.resize(number_of_vertices + 2);
      coordinates[number_of_vertices] = problem.coordinates[initial];
      coordinates[number_of_vertices + 1] = problem.coordinates[goal];
      corresponding_vertices[initial].push_back(number_of_vertices);
      corresponding_vertices[goal].push_back(number_of_vertices + 1);
      tasks.push_back(ProblemInstance::AgentTask(number_of_vertices,
                                                 number_of_vertices + 1));
      number_of_vertices += 2;
      edges.resize(number_of_vertices);
      edge_costs.resize(number_of_vertices);
    }
    for (int vertex = 0; vertex < original_edges.size(); vertex++) {
      auto& connected = connected_edges[vertex];
      if (connected.size() < 2) continue;
      std::sort(connected.begin(), connected.end());
      for (int x = 0; x < connected.size(); x++) {
        int vertex_0 = connected[x].second, vertex_1;
        double angle_distance;
        if (x + 1 < connected.size()) {
          vertex_1 = connected[x + 1].second;
          angle_distance = connected[x + 1].first - connected[x].first;
        } else {
          vertex_1 = connected[0].second;
          angle_distance = 2 * PI + connected[0].first - connected[x].first;
        }
        double turning_cost = angle_distance / turning_speed;
        addEdge(vertex_0, vertex_1, turning_cost);
        stay_edges[vertex].push_back(
            std::make_pair(vertex_0, edges[vertex_0].size() - 1));
        addEdge(vertex_1, vertex_0, turning_cost);
        stay_edges[vertex].push_back(
            std::make_pair(vertex_1, edges[vertex_1].size() - 1));
      }
    }

    number_of_edges = 0;
    edge_ids.clear();
    edge_ids.resize(number_of_vertices);
    for (int i = 0; i < number_of_vertices; i++) {
      int outdegree = edges[i].size();
      edge_ids[i].resize(outdegree);
      std::iota(edge_ids[i].begin(), edge_ids[i].end(), number_of_edges);
      number_of_edges += outdegree;
      for (int x = 0; x < outdegree; x++) {
        int target = edges[i][x];
        edge_data.push_back(std::make_tuple(i, target, x));
      }
    }

    collisions_lists_v2v.clear();
    collisions_lists_v2v.resize(number_of_vertices);
    collisions_lists_e2v.clear();
    collisions_lists_e2v.resize(number_of_edges);
    collisions_lists_v2e.clear();
    collisions_lists_v2e.resize(number_of_vertices);
    collisions_lists_e2e.clear();
    collisions_lists_e2e.resize(number_of_edges);

    for (int vertex_0 = 0; vertex_0 < number_of_original_vertices; vertex_0++) {
      for (auto& vertex_1 : problem.getAllVVCollisions(vertex_0)) {
        for (auto& vertex_0_vertex : corresponding_vertices[vertex_0]) {
          for (auto& vertex_1_vertex : corresponding_vertices[vertex_1]) {
            collisions_lists_v2v[vertex_0_vertex].push_back(vertex_1_vertex);
          }
          for (auto& vertex_1_edge : stay_edges[vertex_1]) {
            double duration_1 =
                edge_costs[vertex_1_edge.first][vertex_1_edge.second];
            collisions_lists_v2e[vertex_0_vertex].push_back(std::make_tuple(
                vertex_1_edge.first, vertex_1_edge.second, -duration_1, 0.));
          }
        }
        for (auto& vertex_0_edge : stay_edges[vertex_0]) {
          double duration_0 =
              edge_costs[vertex_0_edge.first][vertex_0_edge.second];
          int edge_id_0 = edge_ids[vertex_0_edge.first][vertex_0_edge.second];
          for (auto& vertex_1_vertex : corresponding_vertices[vertex_1]) {
            collisions_lists_e2v[edge_id_0].push_back(
                std::make_tuple(vertex_1_vertex, 0., duration_0));
          }
          for (auto& vertex_1_edge : stay_edges[vertex_1]) {
            double duration_1 =
                edge_costs[vertex_1_edge.first][vertex_1_edge.second];
            collisions_lists_e2e[edge_id_0].push_back(
                std::make_tuple(vertex_1_edge.first, vertex_1_edge.second,
                                -duration_1, duration_0));
          }
        }
      }
      for (auto& collisionVE : problem.getAllVECollisions(vertex_0)) {
        auto edge_1 = corresponding_edges[std::get<0>(collisionVE)]
                                         [std::get<1>(collisionVE)];
        double start = std::get<2>(collisionVE), end = std::get<3>(collisionVE);
        for (auto& vertex_0_vertex : corresponding_vertices[vertex_0]) {
          collisions_lists_v2e[vertex_0_vertex].push_back(
              std::make_tuple(edge_1.first, edge_1.second, start, end));
        }
        for (auto& vertex_0_edge : stay_edges[vertex_0]) {
          double duration_0 =
              edge_costs[vertex_0_edge.first][vertex_0_edge.second];
          int edge_id_0 = edge_ids[vertex_0_edge.first][vertex_0_edge.second];
          collisions_lists_e2e[edge_id_0].push_back(std::make_tuple(
              edge_1.first, edge_1.second, start, end + duration_0));
        }
      }
      for (int x = 0; x < original_edges[vertex_0].size(); x++) {
        auto edge_0 = corresponding_edges[vertex_0][x];
        int edge_id_0 = edge_ids[edge_0.first][edge_0.second];
        for (auto& collisionEV : problem.getAllEVCollisions(vertex_0, x)) {
          int vertex_1 = std::get<0>(collisionEV);
          double start = std::get<1>(collisionEV),
                 end = std::get<2>(collisionEV);
          for (auto& vertex_1_vertex : corresponding_vertices[vertex_1]) {
            collisions_lists_e2v[edge_id_0].push_back(
                std::make_tuple(vertex_1_vertex, start, end));
          }
          for (auto& vertex_1_edge : stay_edges[vertex_1]) {
            double duration_1 =
                edge_costs[vertex_1_edge.first][vertex_1_edge.second];
            collisions_lists_e2e[edge_id_0].push_back(
                std::make_tuple(vertex_1_edge.first, vertex_1_edge.second,
                                start - duration_1, end));
          }
        }
        for (auto& collisionEE : problem.getAllEECollisions(vertex_0, x)) {
          auto edge_1 = corresponding_edges[std::get<0>(collisionEE)]
                                           [std::get<1>(collisionEE)];
          double start = std::get<2>(collisionEE),
                 end = std::get<3>(collisionEE);
          collisions_lists_e2e[edge_id_0].push_back(
              std::make_tuple(edge_1.first, edge_1.second, start, end));
        }
      }
    }
  }

  double getEdgeLength(const int source, const int edge_id) const override {
    return edge_costs[source][edge_id];
  }
};

using ProblemInstanceWithTurningPtr =
    std::shared_ptr<ProblemInstanceWithTurning>;

#endif
