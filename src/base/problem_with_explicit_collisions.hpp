/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#ifndef MAPF_BENCHMARK_PROBLEM_WITH_EXPLICIT_COLLISIONS
#define MAPF_BENCHMARK_PROBLEM_WITH_EXPLICIT_COLLISIONS

#include <cassert>

#include "base/collision_report.hpp"
#include "base/problem_with_geometry.hpp"

class ProblemInstanceWithExplicitCollisions
    : public ProblemInstanceWithGeometry<Geometry2D> {
 protected:
  int number_of_vertices, number_of_edges;
  std::vector<std::vector<int>> edge_ids, reverse_edge_ids;
  std::vector<std::tuple<int, int, int>> edge_data;
  std::set<std::pair<int, int>> collision_pairs_v2v;
  std::map<std::pair<int, int>, std::pair<double, double>> collision_pairs_v2e,
      collision_pairs_e2e;

  std::vector<std::vector<int>> collisions_lists_v2v;
  std::vector<std::vector<std::tuple<int, double, double>>>
      collisions_lists_e2v;
  std::vector<std::vector<std::tuple<int, int, double, double>>>
      collisions_lists_v2e, collisions_lists_e2e;

  void makeLists() {
    collisions_lists_v2v.resize(number_of_vertices);
    collisions_lists_v2e.resize(number_of_vertices);
    collisions_lists_e2v.resize(number_of_edges);
    collisions_lists_e2e.resize(number_of_edges);
    for (auto& pair : collision_pairs_v2v) {
      collisions_lists_v2v[pair.first].push_back(pair.second);
      if (pair.first != pair.second) {
        collisions_lists_v2v[pair.second].push_back(pair.first);
      }
    }
    for (auto& pair : collision_pairs_v2e) {
      int vertex = pair.first.first, edge = pair.first.second;
      auto& interval = pair.second;
      collisions_lists_v2e[vertex].push_back(std::make_tuple(
          std::get<0>(edge_data[edge]), std::get<2>(edge_data[edge]),
          -interval.second, -interval.first));
      collisions_lists_e2v[edge].push_back(
          std::make_tuple(vertex, interval.first, interval.second));
    }
    for (auto& pair : collision_pairs_e2e) {
      int edge_0 = pair.first.first, edge_1 = pair.first.second;
      auto& interval = pair.second;
      collisions_lists_e2e[edge_0].push_back(std::make_tuple(
          std::get<0>(edge_data[edge_1]), std::get<2>(edge_data[edge_1]),
          -interval.second, -interval.first));
      if (edge_0 != edge_1) {
        collisions_lists_e2e[edge_1].push_back(std::make_tuple(
            std::get<0>(edge_data[edge_0]), std::get<2>(edge_data[edge_0]),
            interval.first, interval.second));
      }
    }
  }

  void addCollisionVE(const int vertex, const int edge) {
    if (collision_pairs_v2e.find(std::make_pair(vertex, edge)) !=
        collision_pairs_v2e.end()) {
      return;
    }
    int source = std::get<0>(edge_data[edge]),
        target = std::get<1>(edge_data[edge]);
    double length =
        Geometry2D::distance(coordinates[source], coordinates[target]);
    if (length == 0.0) {
      return;
    }
    double lower, upper;
    bool collision = Geometry2D::getCollisionInterval(
        coordinates[vertex], coordinates[source], coordinates[target], length,
        agent_geometry, lower, upper);
    assert(collision);
    collision_pairs_v2e[std::make_pair(vertex, edge)] =
        std::make_pair(lower, upper);
  }

  void addCollisionEE(const int edge_a, const int edge_b) {
    int edge_0 = std::min(edge_a, edge_b), edge_1 = std::max(edge_a, edge_b);
    if (collision_pairs_e2e.find(std::make_pair(edge_0, edge_1)) !=
        collision_pairs_e2e.end()) {
      return;
    }
    int source_0 = std::get<0>(edge_data[edge_0]),
        target_0 = std::get<1>(edge_data[edge_0]);
    int source_1 = std::get<0>(edge_data[edge_1]),
        target_1 = std::get<1>(edge_data[edge_1]);
    double length_0 =
        Geometry2D::distance(coordinates[source_0], coordinates[target_0]);
    double length_1 =
        Geometry2D::distance(coordinates[source_1], coordinates[target_1]);
    if (length_0 == 0.0 || length_1 == 0.0) {
      return;
    }
    double lower, upper;
    bool collision = Geometry2D::getCollisionInterval(
        coordinates[source_0], coordinates[target_0], length_0,
        coordinates[source_1], coordinates[target_1], length_1, agent_geometry,
        lower, upper);
    assert(collision);
    collision_pairs_e2e[std::make_pair(edge_0, edge_1)] =
        std::make_pair(lower, upper);
  }

  std::vector<std::vector<int>> all_collisions_v2v;
  std::vector<std::vector<std::tuple<int, double, double>>> all_collisions_v2e,
      all_collisions_e2v, all_collisions_e2e;

 public:
  ProblemInstanceWithExplicitCollisions() {}
  ProblemInstanceWithExplicitCollisions(
      const ProblemInstanceWithGeometry<Geometry2D>& problem_instance) {
    // pre-calculation for all collision

    edges = problem_instance.edges;
    coordinates = problem_instance.coordinates;
    tasks = problem_instance.tasks;
    agent_geometry = problem_instance.agent_geometry;

    number_of_vertices = problem_instance.getNumberOfVertices();
    number_of_edges = 0;
    edge_ids.resize(number_of_vertices);
    reverse_edge_ids.resize(number_of_vertices);
    for (int i = 0; i < number_of_vertices; i++) {
      int outdegree = problem_instance.getOutDegree(i);
      edge_ids[i].resize(outdegree);
      std::iota(edge_ids[i].begin(), edge_ids[i].end(), number_of_edges);
      number_of_edges += outdegree;
      for (int x = 0; x < outdegree; x++) {
        int target = problem_instance.getTargetVertex(i, x);
        reverse_edge_ids[target].push_back(edge_data.size());
        edge_data.push_back(std::make_tuple(i, target, x));
      }
    }

    Buckets2D buckets(2 * agent_geometry - EPS, coordinates);
    collision_pairs_v2v = std::move(buckets.calculateAllPairs());
    for (auto& vertices_pair : collision_pairs_v2v) {
      int vertex_0 = vertices_pair.first, vertex_1 = vertices_pair.second;
      for (auto& id : edge_ids[vertex_1]) {
        addCollisionVE(vertex_0, id);
      }
      for (auto& id : reverse_edge_ids[vertex_1]) {
        addCollisionVE(vertex_0, id);
      }
      for (auto& id : edge_ids[vertex_0]) {
        addCollisionVE(vertex_1, id);
      }
      for (auto& id : reverse_edge_ids[vertex_0]) {
        addCollisionVE(vertex_1, id);
      }
      for (auto& id_0 : edge_ids[vertex_0]) {
        for (auto& id_1 : edge_ids[vertex_1]) {
          addCollisionEE(id_0, id_1);
        }
        for (auto& id_1 : reverse_edge_ids[vertex_1]) {
          addCollisionEE(id_0, id_1);
        }
      }
      for (auto& id_0 : reverse_edge_ids[vertex_0]) {
        for (auto& id_1 : edge_ids[vertex_1]) {
          addCollisionEE(id_0, id_1);
        }
        for (auto& id_1 : reverse_edge_ids[vertex_1]) {
          addCollisionEE(id_0, id_1);
        }
      }
    }

    for (int edge = 0; edge < number_of_edges; edge++) {
      int source = std::get<0>(edge_data[edge]),
          target = std::get<1>(edge_data[edge]);
      auto collision_vertices = buckets.calculateCollisionVertices(
          coordinates[source], coordinates[target]);
      for (auto vertex : collision_vertices) {
        addCollisionVE(vertex, edge);
        for (auto edge_1 : edge_ids[vertex]) {
          addCollisionEE(edge, edge_1);
        }
        for (auto edge_1 : reverse_edge_ids[vertex]) {
          addCollisionEE(edge, edge_1);
        }
      }
    }
    std::vector<std::pair<Geometry2D::Point, Geometry2D::Point>> segments(
        number_of_edges);
    for (int edge = 0; edge < number_of_edges; edge++) {
      segments[edge] =
          std::make_pair(coordinates[std::get<0>(edge_data[edge])],
                         coordinates[std::get<1>(edge_data[edge])]);
    }
    BentleyOttoman BO(segments);
    auto all_crosses = std::move(BO.calculateAllCrosses());
    for (auto& cross_pair : all_crosses) {
      addCollisionEE(cross_pair.first, cross_pair.second);
    }

    makeLists();
  }

  bool CollisionCheck(const int vertex_0, const int vertex_1) override {
    return collision_pairs_v2v.find(std::make_pair(
               std::min(vertex_0, vertex_1), std::max(vertex_0, vertex_1))) !=
           collision_pairs_v2v.end();
  }

  bool getCollisionInterval(const int vertex_0, const int source_vertex_1,
                            const int edge_id_1, double& lower,
                            double& upper) override {
    auto it = collision_pairs_v2e.find(
        std::make_pair(vertex_0, edge_ids[source_vertex_1][edge_id_1]));
    if (it == collision_pairs_v2e.end()) return false;
    lower = it->second.first;
    upper = it->second.second;
    return true;
  }

  bool CollisionCheck(const int vertex_0, const double start_time_0,
                      const double end_time_0, const int source_vertex_1,
                      const double start_time_1, const int edge_id_1) override {
    double lower, upper;
    if (!getCollisionInterval(vertex_0, source_vertex_1, edge_id_1, lower,
                              upper)) {
      return false;
    }
    return std::max(start_time_0, start_time_1 + lower) + EPS <
           std::min(end_time_0, start_time_1 + upper);
  }

  bool getCollisionInterval(const int source_vertex_0, const int edge_id_0,
                            const int source_vertex_1, const int edge_id_1,
                            double& lower, double& upper) override {
    int edge_0 = edge_ids[source_vertex_0][edge_id_0],
        edge_1 = edge_ids[source_vertex_1][edge_id_1];
    if (edge_0 <= edge_1) {
      auto it = collision_pairs_e2e.find(std::make_pair(edge_0, edge_1));
      if (it == collision_pairs_e2e.end()) return false;
      lower = it->second.first;
      upper = it->second.second;
    } else {
      auto it = collision_pairs_e2e.find(std::make_pair(edge_1, edge_0));
      if (it == collision_pairs_e2e.end()) return false;
      lower = -it->second.second;
      upper = -it->second.first;
    }
    return true;
  }

  bool CollisionCheck(const int source_vertex_0, const double start_time_0,
                      const int edge_id_0, const int source_vertex_1,
                      const double start_time_1, const int edge_id_1) override {
    double lower, upper;
    if (!getCollisionInterval(source_vertex_0, edge_id_0, source_vertex_1,
                              edge_id_1, lower, upper)) {
      return false;
    }
    return lower + EPS < start_time_0 - start_time_1 &&
           start_time_0 - start_time_1 < upper - EPS;
  }

  std::vector<int>& getAllVVCollisions(const int vertex) override {
    return collisions_lists_v2v[vertex];
  }
  std::vector<std::tuple<int, int, double, double>>& getAllVECollisions(
      const int vertex) override {
    return collisions_lists_v2e[vertex];
  }
  std::vector<std::tuple<int, double, double>>& getAllEVCollisions(
      const int vertex, const int edge_id) override {
    return collisions_lists_e2v[edge_ids[vertex][edge_id]];
  }
  std::vector<std::tuple<int, int, double, double>>& getAllEECollisions(
      const int vertex, const int edge_id) override {
    return collisions_lists_e2e[edge_ids[vertex][edge_id]];
  }
};

using ProblemInstanceWithExplicitCollisionsPtr =
    std::shared_ptr<ProblemInstanceWithExplicitCollisions>;
#endif
