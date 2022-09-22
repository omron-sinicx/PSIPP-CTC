/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#ifndef MAPF_BENCHMARK_PROBLEM_WITH_GEOMETRY
#define MAPF_BENCHMARK_PROBLEM_WITH_GEOMETRY

#include <algorithm>
#include <base/problem.hpp>
#include <string>
#include <vector>

template <class Geometry>
class ProblemInstanceWithGeometry : public ProblemInstance {
 public:
  using Point = typename Geometry::Point;
  using AgentGeometry = typename Geometry::AgentGeometry;

  std::vector<Point> coordinates;

  ProblemInstanceWithGeometry(const ProblemInstanceWithGeometry &problem) {
    this->coordinates = problem.coordinates;
    this->edges = problem.edges;
    this->agent_geometry = problem.agent_geometry;
    this->tasks = problem.tasks;
  }

  void setNumberOfVertex(const int n) override {
    coordinates.resize(n);
    edges.clear();
    edges.resize(n);
  }

  void setPoint(const int vertex, const Point &point) {
    coordinates[vertex] = point;
  }

  Point getPoint(const int vertex) const { return coordinates[vertex]; }

  double getCoordinateX(const int vertex) const override {
    return coordinates[vertex].x;
  }

  double getCoordinateY(const int vertex) const override {
    return coordinates[vertex].y;
  }

  AgentGeometry agent_geometry;

  double getAgentRadius() const override { return agent_geometry; }

  double getEdgeLength(const int source, const int edge_id) const override {
    return Geometry::distance(coordinates[source],
                              coordinates[edges[source][edge_id]]);
  }

  bool CollisionCheck(const int vertex_0, const int vertex_1) override {
    number_of_collision_checks++;
    timer_on();
    return timer_off(Geometry::CollisionCheck(
        coordinates[vertex_0], coordinates[vertex_1], agent_geometry, EPS));
  }

  bool CollisionCheck2(const int vertex_0, const double start_time_0,
                       const double end_time_0, const int source_vertex_1,
                       const double start_time_1,
                       const int target_vertex_1) override {
    number_of_collision_checks++;
    timer_on();
    auto &start_point_1 = coordinates[source_vertex_1];
    auto &target_point_1 = coordinates[target_vertex_1];
    return timer_off(Geometry::CollisionCheck(
        coordinates[vertex_0], start_time_0, end_time_0, start_point_1,
        start_time_1, target_point_1,
        start_time_1 + Geometry::distance(start_point_1, target_point_1),
        agent_geometry, EPS));
  }

  bool CollisionCheck(const int vertex_0, const double start_time_0,
                      const double end_time_0, const int source_vertex_1,
                      const double start_time_1, const int edge_id_1) override {
    number_of_collision_checks++;
    timer_on();
    return timer_off(Geometry::CollisionCheck(
        coordinates[vertex_0], start_time_0, end_time_0,
        coordinates[source_vertex_1], start_time_1,
        coordinates[edges[source_vertex_1][edge_id_1]],
        start_time_1 + getEdgeLength(source_vertex_1, edge_id_1),
        agent_geometry, EPS));
  }

  bool getCollisionInterval2(const int vertex_0, const int source_vertex_1,
                             const int target_vertex_1, double &lower,
                             double &upper) override {
    number_of_interval_calculations++;
    timer_on();
    auto &start_point_1 = coordinates[source_vertex_1];
    auto &target_point_1 = coordinates[target_vertex_1];
    return timer_off(Geometry::getCollisionInterval(
        coordinates[vertex_0], start_point_1, target_point_1,
        Geometry::distance(start_point_1, target_point_1), agent_geometry,
        lower, upper));
  }

  bool getCollisionInterval(const int vertex_0, const int source_vertex_1,
                            const int edge_id_1, double &lower,
                            double &upper) override {
    number_of_interval_calculations++;
    timer_on();
    double length = getEdgeLength(source_vertex_1, edge_id_1);
    if (length < EPS) {
      return timer_off(false);
    }
    return timer_off(Geometry::getCollisionInterval(
        coordinates[vertex_0], coordinates[source_vertex_1],
        coordinates[edges[source_vertex_1][edge_id_1]], length, agent_geometry,
        lower, upper));
  }

  bool getCollisionInterval2(const int vertex_0, const int source_vertex_1,
                             const double start_time_1,
                             const int target_vertex_1, const double end_time_1,
                             double &lower, double &upper) override {
    number_of_interval_calculations++;
    timer_on();
    bool collide = Geometry::getCollisionInterval(
        coordinates[vertex_0], coordinates[source_vertex_1],
        coordinates[target_vertex_1], end_time_1 - start_time_1, agent_geometry,
        lower, upper);
    if (collide) {
      lower += start_time_1;
      upper += start_time_1;
    }
    return timer_off(collide);
  }

  bool CollisionCheck2(const int source_vertex_0, const double start_time_0,
                       const int target_vertex_0, const int source_vertex_1,
                       const double start_time_1,
                       const int target_vertex_1) override {
    number_of_collision_checks++;
    timer_on();
    auto &start_point_0 = coordinates[source_vertex_0];
    auto &target_point_0 = coordinates[target_vertex_0];
    auto &start_point_1 = coordinates[source_vertex_1];
    auto &target_point_1 = coordinates[target_vertex_1];
    return timer_off(Geometry::CollisionCheck(
        start_point_0, start_time_0, target_point_0,
        start_time_0 + Geometry::distance(start_point_0, target_point_0),
        start_point_1, start_time_1, target_point_1,
        start_time_1 + Geometry::distance(start_point_1, target_point_1),
        agent_geometry, EPS));
  }

  bool CollisionCheck(const int source_vertex_0, const double start_time_0,
                      const int edge_id_0, const int source_vertex_1,
                      const double start_time_1, const int edge_id_1) override {
    number_of_collision_checks++;
    timer_on();
    return timer_off(Geometry::CollisionCheck(
        coordinates[source_vertex_0], start_time_0,
        coordinates[edges[source_vertex_0][edge_id_0]],
        start_time_0 + getEdgeLength(source_vertex_0, edge_id_0),
        coordinates[source_vertex_1], start_time_1,
        coordinates[edges[source_vertex_1][edge_id_1]],
        start_time_1 + getEdgeLength(source_vertex_1, edge_id_1),
        agent_geometry, EPS));
  }

  bool getCollisionInterval(const int source_vertex_0, const int edge_id_0,
                            const int source_vertex_1, const int edge_id_1,
                            double &lower, double &upper) override {
    number_of_interval_calculations++;
    timer_on();
    double length_0 = getEdgeLength(source_vertex_0, edge_id_0);
    double length_1 = getEdgeLength(source_vertex_1, edge_id_1);
    if (length_0 < EPS || length_1 < EPS) {
      return timer_off(false);
    }
    return timer_off(Geometry::getCollisionInterval(
        coordinates[source_vertex_0],
        coordinates[edges[source_vertex_0][edge_id_0]], length_0,
        coordinates[source_vertex_1],
        coordinates[edges[source_vertex_1][edge_id_1]], length_1,
        agent_geometry, lower, upper));
  }

  bool getCollisionInterval2(const int source_vertex_0,
                             const int target_vertex_0,
                             const int source_vertex_1,
                             const int target_vertex_1, double &lower,
                             double &upper) override {
    number_of_interval_calculations++;
    timer_on();
    auto &start_point_0 = coordinates[source_vertex_0];
    auto &target_point_0 = coordinates[target_vertex_0];
    auto &start_point_1 = coordinates[source_vertex_1];
    auto &target_point_1 = coordinates[target_vertex_1];
    return timer_off(Geometry::getCollisionInterval(
        start_point_0, target_point_0,
        Geometry::distance(start_point_0, target_point_0), start_point_1,
        target_point_1, Geometry::distance(start_point_1, target_point_1),
        agent_geometry, lower, upper));
  }

  bool checkPlanValidity(const Plan &plan, double &makespan,
                         double &sum_of_costs) override {
    int n = tasks.size();
    if (plan.size() != n) {
      return false;
    }
    struct Event {
      int agent;
      double time;
      Event(int agent, double time) {
        this->agent = agent;
        this->time = time;
      }
    };
    std::vector<Event> events;
    std::vector<std::vector<std::pair<int, int>>> routes(n);
    std::vector<std::vector<double>> time_stamps(n);
    for (int i = 0; i < n; i++) {
      // Check route validity
      double current_time = 0.0;
      int current_vertex = tasks[i].initial_vertex;
      for (int j = 0; j < plan[i].size(); j++) {
        if (plan[i][j].start_time < current_time - EPS) {
          throw std::runtime_error("agent starts before reach start vertex\n");
        }
        if (getOutDegree(current_vertex) <= plan[i][j].edge_id) {
          throw std::runtime_error("edge id is invalid\n");
        }
        int target_vertex = getTargetVertex(current_vertex, plan[i][j].edge_id);
        double edge_length = getEdgeLength(current_vertex, plan[i][j].edge_id);
        double end_time = plan[i][j].start_time + edge_length;
        if (current_time + EPS < plan[i][j].start_time) {
          time_stamps[i].push_back(current_time);
          routes[i].push_back(std::make_pair(current_vertex, -1));  // wait
          events.push_back(Event(i, current_time));
        }
        time_stamps[i].push_back(plan[i][j].start_time);
        routes[i].push_back(std::make_pair(current_vertex, plan[i][j].edge_id));
        events.push_back(Event(i, plan[i][j].start_time));
        current_vertex = target_vertex;
        current_time = end_time;
      }
      if (current_vertex != tasks[i].goal_vertex) {
        throw std::runtime_error("agent does not reach goal\n");
      }
      time_stamps[i].push_back(current_time);
      routes[i].push_back(std::make_pair(current_vertex, -1));  // wait forever
      events.push_back(Event(i, current_time));
      time_stamps[i].push_back(INF);
    }

    // Collision Check
    std::sort(
        events.begin(), events.end(),
        [](const Event &e0, const Event &e1) { return e0.time < e1.time; });
    std::vector<int> steps(n, 0);
    for (int t = 0; t < events.size(); t++) {
      int i = events[t].agent;
      for (int j = 0; j < n; j++) {
        if (j == i || steps[j] == routes[j].size() ||
            time_stamps[j][steps[j]] + EPS >=
                std::min(time_stamps[i][steps[i] + 1],
                         time_stamps[j][steps[j] + 1]))
          continue;
        bool collision = ProblemInstance::CollisionCheck(
            routes[i][steps[i]].first, time_stamps[i][steps[i]],
            routes[i][steps[i]].second, time_stamps[i][steps[i] + 1],
            routes[j][steps[j]].first, time_stamps[j][steps[j]],
            routes[j][steps[j]].second, time_stamps[j][steps[j] + 1]);
        if (collision) {
          throw std::runtime_error("collision occurs\n");
        }
      }
      steps[i]++;
    }
    // calculate makespan and sum of costs
    makespan = 0.0;
    sum_of_costs = 0.0;
    for (int i = 0; i < n; i++) {
      double cost =
          time_stamps[i][time_stamps[i].size() - 2];  // time for arrive goal
      makespan = std::max(makespan, cost);
      sum_of_costs += cost;
    }
    return true;
  }
  ProblemInstanceWithGeometry(){};
  ProblemInstanceWithGeometry(const std::string &file_path);
  void print(const std::string &file_path);
  void printVertex(FILE *file, const int vertex) const override {
    Geometry::printPoint(file, coordinates[vertex]);
  }
};

template <class Geometry>
using ProblemInstanceWithGeometryPtr =
    std::shared_ptr<ProblemInstanceWithGeometry<Geometry>>;

#endif
