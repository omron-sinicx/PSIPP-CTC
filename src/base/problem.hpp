/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#ifndef MAPF_BENCHMARK_PROBLEM
#define MAPF_BENCHMARK_PROBLEM

#include <algorithm>
#include <chrono>
#include <cmath>
#include <list>
#include <memory>
#include <string>
#include <vector>

class ProblemInstance {
 public:
  std::vector<std::vector<int>> edges;

  const double INF = std::numeric_limits<double>::infinity(), EPS = 1e-8;

  virtual void setNumberOfVertex(const int n) {
    edges.clear();
    edges.resize(n);
  }

  void addEdge(int source, int target) { edges[source].push_back(target); }

  int getNumberOfVertices() const { return edges.size(); }

  int getOutDegree(const int vertex) const { return edges[vertex].size(); }

  int getTargetVertex(const int source, const int edge_id) const {
    return edges[source][edge_id];
  }

  virtual double getEdgeLength(const int source, const int edge_id) const = 0;

  virtual double getCoordinateX(const int vertex) const = 0;
  virtual double getCoordinateY(const int vertex) const = 0;
  virtual double getAgentRadius() const = 0;

  struct AgentTask {
    int initial_vertex, goal_vertex;
    AgentTask() {}
    AgentTask(const int initial_vertex, const int goal_vertex) {
      this->initial_vertex = initial_vertex;
      this->goal_vertex = goal_vertex;
    }
  };

  std::vector<AgentTask> tasks;

  int getNumberOfAgents() { return tasks.size(); }

  int getAgentInitialVertex(int agent) { return tasks[agent].initial_vertex; }

  int getAgentGoalVertex(int agent) { return tasks[agent].goal_vertex; }

  struct Move {
    int edge_id;
    double start_time;
    Move() {}
    Move(int _edge_id, double _start_time)
        : edge_id(_edge_id), start_time(_start_time) {}
  };

  using Plan = std::vector<std::vector<Move>>;

  int number_of_collision_checks = 0, number_of_interval_calculations = 0;
  std::chrono::duration<double> used_time{};
  std::chrono::system_clock::time_point timer_time;
  int timer_switch = 0;

  void timer_on() {
    if (timer_switch++ == 0) {
      timer_time = std::chrono::system_clock::now();
    }
  }

  template <typename T>
  T timer_off(const T ret) {
    if (--timer_switch == 0) {
      used_time += std::chrono::system_clock::now() - timer_time;
    }
    return ret;
  }

  virtual bool CollisionCheck(const int vertex_0, const int vertex_1) = 0;

  virtual bool CollisionCheck(const int vertex_0, const double start_time_0,
                              const double end_time_0,
                              const int source_vertex_1,
                              const double start_time_1,
                              const int edge_id_1) = 0;

  virtual bool CollisionCheck2(const int vertex_0, const double start_time_0,
                               const double end_time_0,
                               const int source_vertex_1,
                               const double start_time_1,
                               const int target_vertex_1) = 0;

  virtual bool CollisionCheck(const int source_vertex_0,
                              const double start_time_0, const int edge_id_0,
                              const int source_vertex_1,
                              const double start_time_1,
                              const int edge_id_1) = 0;

  virtual bool CollisionCheck2(const int source_vertex_0,
                               const double start_time_0, const int edge_id_0,
                               const int source_vertex_1,
                               const double start_time_1,
                               const int target_vertex_1) = 0;

  virtual bool CollisionCheck(const int source_vertex_0,
                              const double start_time_0, const int edge_id_0,
                              const double end_time_0,
                              const int source_vertex_1,
                              const double start_time_1, const int edge_id_1,
                              const double end_time_1) {
    if (edge_id_0 < 0) {
      if (edge_id_1 < 0) {
        return CollisionCheck(source_vertex_0, source_vertex_1);
      } else {
        return CollisionCheck(source_vertex_0, start_time_0, end_time_0,
                              source_vertex_1, start_time_1, edge_id_1);
      }
    } else {
      if (edge_id_1 < 0) {
        return CollisionCheck(source_vertex_1, start_time_1, end_time_1,
                              source_vertex_0, start_time_0, edge_id_0);
      } else {
        return CollisionCheck(source_vertex_0, start_time_0, edge_id_0,
                              source_vertex_1, start_time_1, edge_id_1);
      }
    }
  }

  virtual bool CollisionCheck2(
      const int source_vertex_0, const double start_time_0,
      const int target_vertex_0, const double end_time_0,
      const int source_vertex_1, const double start_time_1,
      const int target_vertex_1, const double end_time_1) {
    if (source_vertex_0 == target_vertex_0) {
      if (source_vertex_1 == target_vertex_1) {
        return CollisionCheck(source_vertex_0, source_vertex_1);
      } else {
        return CollisionCheck2(source_vertex_0, start_time_0, end_time_0,
                               source_vertex_1, start_time_1, target_vertex_1);
      }
    } else {
      if (source_vertex_1 == target_vertex_1) {
        return CollisionCheck2(source_vertex_1, start_time_1, end_time_1,
                               source_vertex_0, start_time_0, target_vertex_0);
      } else {
        return CollisionCheck2(source_vertex_0, start_time_0, target_vertex_0,
                               source_vertex_1, start_time_1, target_vertex_1);
      }
    }
  }

  virtual bool getCollisionInterval(const int vertex_0,
                                    const int source_vertex_1,
                                    const int edge_id_1, double& lower,
                                    double& upper) = 0;

  virtual bool getCollisionInterval2(const int vertex_0,
                                     const int source_vertex_1,
                                     const int target_vertex_1, double& lower,
                                     double& upper) = 0;

  virtual bool getCollisionInterval(const int source_vertex_0,
                                    const int edge_id_0,
                                    const int source_vertex_1,
                                    const int edge_id_1, double& lower,
                                    double& upper) = 0;

  virtual bool getCollisionInterval2(const int source_vertex_0,
                                     const int target_vertex_0,
                                     const int source_vertex_1,
                                     const int target_vertex_1, double& lower,
                                     double& upper) = 0;

  virtual bool getCollisionInterval(const int vertex_0,
                                    const int source_vertex_1,
                                    const double start_time_1,
                                    const int edge_id_1,
                                    const double end_time_1, double& lower,
                                    double& upper) {
    bool collision;
    if (edge_id_1 < 0) {
      collision = CollisionCheck(vertex_0, source_vertex_1);
      if (collision) {
        lower = start_time_1;
        upper = end_time_1;
      }
    } else {
      collision = getCollisionInterval(vertex_0, source_vertex_1, edge_id_1,
                                       lower, upper);
      if (collision) {
        lower += start_time_1;
        upper += start_time_1;
      }
    }
    return collision;
  }

  virtual bool getCollisionInterval2(const int vertex_0,
                                     const int source_vertex_1,
                                     const double start_time_1,
                                     const int target_vertex_1,
                                     const double end_time_1, double& lower,
                                     double& upper) {
    bool collision;
    if (source_vertex_1 == target_vertex_1) {
      collision = CollisionCheck(vertex_0, source_vertex_1);
      if (collision) {
        lower = start_time_1;
        upper = end_time_1;
      }
    } else {
      collision = getCollisionInterval2(vertex_0, source_vertex_1,
                                        target_vertex_1, lower, upper);
      if (collision) {
        lower += start_time_1;
        upper += start_time_1;
      }
    }
    return collision;
  }

  virtual bool getCollisionInterval(
      const int source_vertex_0, const int edge_id_0, const int source_vertex_1,
      const double start_time_1, const int edge_id_1, const double end_time_1,
      double& lower, double& upper) {
    bool collision;
    if (edge_id_1 < 0) {
      collision = getCollisionInterval(source_vertex_1, source_vertex_0,
                                       edge_id_0, upper, lower);
      if (collision) {
        lower = start_time_1 - lower;
        upper = end_time_1 - upper;
      }
    } else {
      collision = getCollisionInterval(
          source_vertex_0, edge_id_0, source_vertex_1, edge_id_1, lower, upper);
      if (collision) {
        lower += start_time_1;
        upper += start_time_1;
      }
    }
    return collision;
  }

  virtual bool getCollisionInterval2(const int source_vertex_0,
                                     const int target_vertex_0,
                                     const int source_vertex_1,
                                     const double start_time_1,
                                     const int target_vertex_1,
                                     const double end_time_1, double& lower,
                                     double& upper) {
    bool collision;
    if (source_vertex_1 == target_vertex_1) {
      collision = getCollisionInterval2(source_vertex_1, source_vertex_0,
                                        target_vertex_0, upper, lower);
      if (collision) {
        lower = start_time_1 - lower;
        upper = end_time_1 - upper;
      }
    } else {
      collision =
          getCollisionInterval2(source_vertex_0, target_vertex_0,
                                source_vertex_1, target_vertex_1, lower, upper);
      if (collision) {
        lower += start_time_1;
        upper += start_time_1;
      }
    }
    return collision;
  }

 private:
  std::vector<int> all_vertices;
  std::vector<std::tuple<int, double, double>> all_collision_vertices;
  std::vector<std::tuple<int, int, double, double>> all_collision_edges;

 public:
  virtual std::vector<int>& getAllVVCollisions(const int vertex) {
    all_vertices.clear();
    for (int vertex_0 = 0; vertex_0 < edges.size(); vertex_0++) {
      if (CollisionCheck(vertex_0, vertex)) {
        all_vertices.push_back(vertex_0);
      }
    }
    return all_vertices;
  }

  virtual std::vector<std::tuple<int, int, double, double>>& getAllVECollisions(
      const int vertex) {
    all_collision_edges.clear();
    for (int vertex_0 = 0; vertex_0 < edges.size(); vertex_0++) {
      for (int edge_id_0 = 0; edge_id_0 < edges[vertex_0].size(); edge_id_0++) {
        double lower, upper;
        if (getCollisionInterval(vertex, vertex_0, edge_id_0, lower, upper)) {
          all_collision_edges.push_back(
              std::make_tuple(vertex_0, edge_id_0, -upper, -lower));
        }
      }
    }
    return all_collision_edges;
  }
  virtual std::vector<std::tuple<int, double, double>>& getAllEVCollisions(
      const int vertex, const int edge_id) {
    all_collision_vertices.clear();
    for (int vertex_0 = 0; vertex_0 < edges.size(); vertex_0++) {
      double lower, upper;
      if (getCollisionInterval(vertex_0, vertex, edge_id, lower, upper)) {
        all_collision_vertices.push_back(
            std::make_tuple(vertex_0, lower, upper));
      }
    }
    return all_collision_vertices;
  }
  virtual std::vector<std::tuple<int, int, double, double>>& getAllEECollisions(
      const int vertex, const int edge_id) {
    all_collision_edges.clear();
    for (int vertex_0 = 0; vertex_0 < edges.size(); vertex_0++) {
      for (int edge_id_0 = 0; edge_id_0 < edges[vertex_0].size(); edge_id_0++) {
        double lower, upper;
        if (getCollisionInterval(vertex_0, edge_id_0, vertex, edge_id, lower,
                                 upper)) {
          all_collision_edges.push_back(
              std::make_tuple(vertex_0, edge_id_0, lower, upper));
        }
      }
    }
    return all_collision_edges;
  }

  virtual bool checkPlanValidity(const Plan& plan, double& makespan,
                                 double& sum_of_costs) {
    int n = plan.size();
    if (tasks.size() != n) {
      return false;
    }
    std::vector<std::vector<std::pair<int, int>>> routes(n);
    std::vector<std::vector<double>> time_stamps(n);
    for (int i = 0; i < n; i++) {
      // Check route validity
      double current_time = 0.0;
      int current_vertex = tasks[i % tasks.size()].initial_vertex;
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
        }
        time_stamps[i].push_back(plan[i][j].start_time);
        routes[i].push_back(std::make_pair(current_vertex, plan[i][j].edge_id));
        current_vertex = target_vertex;
        current_time = end_time;
      }
      if (current_vertex != tasks[i % tasks.size()].goal_vertex) {
        throw std::runtime_error("agent does not reach goal\n");
      }
      time_stamps[i].push_back(current_time);
      // routes[i].push_back(std::make_pair(current_vertex, -1));  // wait
      // forever time_stamps[i].push_back(INF);
    }

    // Collision Check
    for (int i = 0; i < n; i++) {
      for (int step0 = 0; step0 < routes[i].size(); step0++) {
        for (int j = i + 1; j < n; j++) {
          for (int step1 = 0; step1 < routes[j].size(); step1++) {
            bool collision = CollisionCheck(
                routes[i][step0].first, time_stamps[i][step0],
                routes[i][step0].second, time_stamps[i][step0 + 1],
                routes[j][step1].first, time_stamps[j][step1],
                routes[j][step1].second, time_stamps[j][step1 + 1]);
            if (collision) {
              throw std::runtime_error("collision occurs\n");
            }
          }
        }
      }
    }
    // calculate makespan and sum of costs
    makespan = 0.0;
    sum_of_costs = 0.0;
    for (int i = 0; i < n; i++) {
      double cost =
          time_stamps[i][time_stamps[i].size() - 1];  // time for arrive goal
      makespan = std::max(makespan, cost);
      sum_of_costs += cost;
    }
    return true;
  }
  ProblemInstance() {}

  virtual void printVertex(FILE* file, const int vertex) const = 0;
};

using ProblemInstancePtr = std::shared_ptr<ProblemInstance>;

#endif
