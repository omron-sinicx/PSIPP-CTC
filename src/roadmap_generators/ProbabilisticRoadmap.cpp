/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include <set>

#include "base/collision_report.hpp"
#include "base/roadmap_generator.hpp"

template <class SpaceGeometry>
class KNearestProbabilisticRoadmap : public RoadmapGenerator<SpaceGeometry> {
  using Point = typename SpaceGeometry::Point;

  std::set<std::pair<double, int>> getNeighbors(const int limit, const int id) {
    auto& problem_instance = RoadmapGenerator<SpaceGeometry>::problem_instance;
    std::set<std::pair<double, int>> neighbors;
    Point point = problem_instance.getPoint(id);
    for (int i = 0; i < limit; i++) {
      if (i == id) continue;
      double d = SpaceGeometry::Geometry::distance(
          point, problem_instance.getPoint(i));
      if (neighbors.size() < K) {
        neighbors.insert(std::make_pair(d, i));
      } else {
        auto last = --neighbors.end();
        if (last->first > d) {
          neighbors.erase(last);
          neighbors.insert(std::make_pair(d, i));
        }
      }
    }
    return neighbors;
  }

 public:
  double small_value = 0.1;

  int number_of_vertices, K;
  bool use_other_agent_points;

  KNearestProbabilisticRoadmap(const int number_of_vertices, const int K = 0,
                               const bool use_other_agent_points = true) {
    this->number_of_vertices = number_of_vertices;
    if (K == 0) {
      double n = number_of_vertices;
      this->K = std::ceil(
          (std::exp(1.) * (1. + 1. / (double)SpaceGeometry::Dimension) +
           small_value) *
          std::log(n));
    } else {
      this->K = K;
    }
    this->use_other_agent_points = use_other_agent_points;
  }

  void generate(const SpatialProblemInstance<SpaceGeometry>&
                    spacial_problem_instance) override {
    auto& problem_instance = RoadmapGenerator<SpaceGeometry>::problem_instance;
    auto& space = spacial_problem_instance.space;
    auto& tasks = spacial_problem_instance.tasks;

    problem_instance.tasks.resize(tasks.size());
    problem_instance.agent_geometry = spacial_problem_instance.agent_geometry;

    problem_instance.setNumberOfVertex(number_of_vertices + 2 * tasks.size());
    std::vector<Point> random_points = SpaceGeometry::getRandomPoints(
        space, problem_instance.agent_geometry, number_of_vertices);

    for (int i = 0; i < number_of_vertices; i++) {
      problem_instance.setPoint(i, random_points[i]);
    }
    for (int agent = 0; agent < tasks.size(); agent++) {
      const Point &initial_point = tasks[agent].initial_point,
                  goal_point = tasks[agent].goal_point;
      int initial_vertex = number_of_vertices + 2 * agent;
      int goal_vertex = number_of_vertices + 2 * agent + 1;
      problem_instance.tasks[agent].initial_vertex = initial_vertex;
      problem_instance.tasks[agent].goal_vertex = goal_vertex;
      problem_instance.setPoint(initial_vertex, initial_point);
      problem_instance.setPoint(goal_vertex, goal_point);
      if (use_other_agent_points) continue;
      auto initial_neighbors =
          std::move(getNeighbors(number_of_vertices, initial_vertex));
      for (auto it = initial_neighbors.begin(); it != initial_neighbors.end();
           ++it) {
        if (SpaceGeometry::checkEdgeInside(
                space, initial_point, problem_instance.getPoint(it->second),
                problem_instance.agent_geometry)) {
          problem_instance.addEdge(initial_vertex, it->second);
          problem_instance.addEdge(it->second, initial_vertex);
        }
      }
      auto goal_neighbors =
          std::move(getNeighbors(number_of_vertices, goal_vertex));
      for (auto it = goal_neighbors.begin(); it != goal_neighbors.end(); ++it) {
        if (SpaceGeometry::checkEdgeInside(
                space, problem_instance.getPoint(it->second), goal_point,
                problem_instance.agent_geometry)) {
          problem_instance.addEdge(it->second, goal_vertex);
          problem_instance.addEdge(goal_vertex, it->second);
        }
      }
    }
    int number_of_bidirectionals;
    if (use_other_agent_points) {
      number_of_bidirectionals = number_of_vertices + 2 * tasks.size();
    } else {
      number_of_bidirectionals = number_of_vertices;
    }
    std::vector<std::pair<int, int>> edge_list;
    for (int i = 0; i < number_of_bidirectionals; i++) {
      auto neighbors = std::move(getNeighbors(number_of_bidirectionals, i));
      for (auto it = neighbors.begin(); it != neighbors.end(); ++it) {
        edge_list.push_back(
            std::make_pair(std::min(i, it->second), std::max(i, it->second)));
      }
    }
    std::sort(edge_list.begin(), edge_list.end());
    edge_list.erase(std::unique(edge_list.begin(), edge_list.end()),
                    edge_list.end());
    for (auto& pair : edge_list) {
      if (SpaceGeometry::checkEdgeInside(space,
                                         problem_instance.getPoint(pair.first),
                                         problem_instance.getPoint(pair.second),
                                         problem_instance.agent_geometry)) {
        problem_instance.addEdge(pair.first, pair.second);
        problem_instance.addEdge(pair.second, pair.first);
      }
    }
  }
};

#include "base/geometry_2D_for_generator.hpp"

class RadiusProbabilisticRoadmap2D : public RoadmapGenerator<Space2D> {
  using Point = typename Space2D::Point;

 public:
  double small_value = 0.1;

  int number_of_vertices;
  double radius;
  bool use_other_agent_points;

  std::set<std::pair<int, int>> getAllPairs() {
    auto& problem_instance = RoadmapGenerator<Space2D>::problem_instance;
    std::vector<Point> points(problem_instance.getNumberOfVertices());
    for (int i = 0; i < points.size(); i++) {
      points[i] = problem_instance.getPoint(i);
    }
    Buckets2D buckets(radius, points);
    return buckets.calculateAllPairs();
  }

  RadiusProbabilisticRoadmap2D(const int number_of_vertices,
                               const double radius = 0.,
                               const bool use_other_agent_points = true) {
    this->number_of_vertices = number_of_vertices;
    this->radius = radius;
    this->use_other_agent_points = use_other_agent_points;
  }

  void generate(const SpatialProblemInstance<Space2D>& spacial_problem_instance)
      override {
    auto& problem_instance = RoadmapGenerator<Space2D>::problem_instance;
    auto& space = spacial_problem_instance.space;
    auto& tasks = spacial_problem_instance.tasks;

    if (radius == 0.0) {
      double n = number_of_vertices, pi = 4. * std::atan(1.0);
      this->radius = (std::sqrt(6. *
                                Space2D::calculateFreeVolume(
                                    spacial_problem_instance.space,
                                    spacial_problem_instance.agent_geometry) /
                                pi) +
                      small_value) *
                     std::sqrt(std::log(n) / n);
    }

    problem_instance.tasks.resize(tasks.size());
    problem_instance.agent_geometry = spacial_problem_instance.agent_geometry;

    problem_instance.setNumberOfVertex(number_of_vertices + 2 * tasks.size());
    std::vector<Point> random_points = Space2D::getRandomPoints(
        space, problem_instance.agent_geometry, number_of_vertices);

    for (int i = 0; i < number_of_vertices; i++) {
      problem_instance.setPoint(i, random_points[i]);
    }
    for (int agent = 0; agent < tasks.size(); agent++) {
      const Point &initial_point = tasks[agent].initial_point,
                  goal_point = tasks[agent].goal_point;
      problem_instance.tasks[agent].initial_vertex =
          number_of_vertices + 2 * agent;
      problem_instance.tasks[agent].goal_vertex =
          number_of_vertices + 2 * agent + 1;
      problem_instance.setPoint(problem_instance.tasks[agent].initial_vertex,
                                initial_point);
      problem_instance.setPoint(problem_instance.tasks[agent].goal_vertex,
                                goal_point);
    }
    auto all_pairs = std::move(getAllPairs());
    for (auto& pair : all_pairs) {
      if (pair.first == pair.second) continue;
      bool f_to_s = false, s_to_f = false;
      if (use_other_agent_points) {
        f_to_s = s_to_f = true;
      } else if (pair.first < number_of_vertices) {
        if (pair.second < number_of_vertices) {
          f_to_s = s_to_f = true;
        } else if ((pair.second - number_of_vertices) % 2 == 0) {
          s_to_f = true;
        } else if ((pair.second - number_of_vertices) % 2 == 1) {
          f_to_s = true;
        }
      } else if ((pair.first - number_of_vertices) % 2 == 0) {
        if (pair.second < number_of_vertices || pair.second == pair.first + 1) {
          f_to_s = true;
        }
      } else if ((pair.first - number_of_vertices) % 2 == 1) {
        if (pair.second < number_of_vertices || pair.second == pair.first - 1) {
          s_to_f = true;
        }
      }
      if (!f_to_s && !s_to_f) continue;
      if (Space2D::checkEdgeInside(space, problem_instance.getPoint(pair.first),
                                   problem_instance.getPoint(pair.second),
                                   problem_instance.agent_geometry)) {
        if (f_to_s) {
          problem_instance.addEdge(pair.first, pair.second);
        }
        if (s_to_f) {
          problem_instance.addEdge(pair.second, pair.first);
        }
      }
    }
  }
};
