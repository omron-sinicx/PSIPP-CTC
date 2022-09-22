/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#ifndef MAPF_BENCHMARK_SPACIAL
#define MAPF_BENCHMARK_SPACIAL

#include "base/problem_with_geometry.hpp"

template <class SpaceGeometry>
class SpatialProblemInstance {
 public:
  using Geometry = typename SpaceGeometry::Geometry;
  typename SpaceGeometry::Space space;
  struct Task {
    typename Geometry::Point initial_point, goal_point;
    Task() {}
    Task(const typename Geometry::Point& initial_point,
         const typename Geometry::Point& goal_point) {
      this->initial_point = initial_point, this->goal_point = goal_point;
    }
  };
  typename Geometry::AgentGeometry agent_geometry;
  std::vector<Task> tasks;

  double EPS = 1e-6;

  bool isInside(const typename SpaceGeometry::Point& point) const {
    return SpaceGeometry::checkVertexInside(space, point, agent_geometry - EPS);
  }

  bool isInside(const typename SpaceGeometry::Point& point_0,
                const typename SpaceGeometry::Point& point_1) const {
    return SpaceGeometry::checkEdgeInside(space, point_0, point_1,
                                          agent_geometry - EPS);
  }

  std::pair<double, double> getBound(const int axis) const {
    return SpaceGeometry::getBound(space, axis);
  }

  bool checkRoadspaceValidity(
      const ProblemInstanceWithGeometry<Geometry>& problem_instance) {
    int n = problem_instance.getNumberOfVertices();
    for (int vertex = 0; vertex < n; vertex++) {
      if (!SpaceGeometry::checkVertexInside(
              space, problem_instance.getPoint(vertex),
              problem_instance.agent_geometry - EPS)) {
        fprintf(stderr, "a vertex is outside of the map:\n");
        Geometry::printPoint(stderr, problem_instance.getPoint(vertex));
        return false;
      }
    }
    for (int vertex = 0; vertex < n; vertex++) {
      int outdegree = problem_instance.getOutDegree(vertex);
      for (int edge_id = 0; edge_id < outdegree; edge_id++) {
        int target = problem_instance.getTargetVertex(vertex, edge_id);
        if (!SpaceGeometry::checkEdgeInside(
                space, problem_instance.getPoint(vertex),
                problem_instance.getPoint(target),
                problem_instance.agent_geometry - EPS)) {
          fprintf(stderr, "an edge is outside of the map\n");
          Geometry::printPoint(stderr, problem_instance.getPoint(vertex));
          Geometry::printPoint(stderr, problem_instance.getPoint(target));
          return false;
        }
      }
    }
    for (int i = 0; i < tasks.size(); i++) {
      if (Geometry::distance(tasks[i].initial_point,
                             problem_instance.getPoint(
                                 problem_instance.tasks[i].initial_vertex)) >=
          EPS) {
        fprintf(stderr, "an initial point is not equal\n");
        return false;
      }
      if (Geometry::distance(tasks[i].goal_point,
                             problem_instance.getPoint(
                                 problem_instance.tasks[i].goal_vertex)) >=
          EPS) {
        fprintf(stderr, "a goal point is not equal\n");
        return false;
      }
    }
    if (abs(problem_instance.agent_geometry - agent_geometry) >= EPS) {
      fprintf(stderr, "an agent geometry is not equal\n");
      return false;
    }
    return true;
  }

  using Plan = std::vector<
      std::vector<std::pair<typename SpaceGeometry::Point, double>>>;

  bool checkPlanValidity(const Plan& plan, double& makespan,
                         double& sum_of_costs) {
    int number_of_agents = tasks.size();
    if (plan.size() != number_of_agents) {
      return false;
    }
    makespan = 0.0;
    sum_of_costs = 0.0;
    struct Event {
      int agent;
      double time;
      Event(int agent, double time) {
        this->agent = agent;
        this->time = time;
      }
    };
    std::vector<Event> events;
    for (int agent = 0; agent < number_of_agents; agent++) {
      assert(plan[agent][0].second <= EPS);
      for (int t = 0; t + 1 < plan[agent].size(); t++) {
        assert(SpaceGeometry::checkEdgeInside(space, plan[agent][t].first,
                                              plan[agent][t + 1].first,
                                              agent_geometry - EPS));
        double length =
            Geometry::distance(plan[agent][t].first, plan[agent][t + 1].first);
        assert(length <=
               plan[agent][t + 1].second - plan[agent][t].second + EPS);
        events.push_back(Event(agent, plan[agent][t + 1].second));
      }
      double goal_time = plan[agent][plan[agent].size() - 1].second;
      makespan = std::max(makespan, goal_time);
      sum_of_costs += goal_time;
    }
    std::sort(
        events.begin(), events.end(),
        [](const Event& e0, const Event& e1) { return e0.time < e1.time; });
    std::vector<int> steps(number_of_agents, 0);
    for (int t = 0; t < events.size(); t++) {
      int i = events[t].agent;
      auto next_i =
          steps[i] + 1 < plan[i].size()
              ? plan[i][steps[i] + 1]
              : std::make_pair(tasks[i].goal_point,
                               std::numeric_limits<double>::infinity());
      for (int j = 0; j < number_of_agents; j++) {
        if (i == j) continue;
        auto next_j =
            steps[j] + 1 < plan[j].size()
                ? plan[j][steps[j] + 1]
                : std::make_pair(tasks[j].goal_point,
                                 std::numeric_limits<double>::infinity());
        assert(!Geometry::CollisionCheck(
            plan[i][steps[i]].first, plan[i][steps[i]].second, next_i.first,
            next_i.second, plan[j][steps[j]].first, plan[j][steps[j]].second,
            next_j.first, next_j.second, agent_geometry, EPS));
      }
      steps[i]++;
    }
    return true;
  }

  SpatialProblemInstance() {}
  void loadTextFile(const std::string& file_path);
  void printTextFile(const std::string& file_path) const;
  void simplifySpace(const double max_distance);
  void loadGripMap(const std::string& file_path);
  void loadScenario(const std::string& file_path);
};

template <class SpaceGeometry>
using SpatialProblemInstancePtr =
    std::shared_ptr<SpatialProblemInstance<SpaceGeometry>>;
#endif
