/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include "planners/PrioritizedSIPP.hpp"

#include <cassert>

void PrioritizedSIPP::addVertexCollisionInterval(
    std::set<IntervalWithID>& safes, const double start, const double end) {
  auto it = safes.lower_bound(IntervalWithID(start - EPS));
  if (it != safes.begin()) {
    auto prev = it;
    --prev;
    if (prev->end > start) {
      double a = prev->start, b = prev->end;
      int id = prev->id;
      safes.erase(prev);
      assert(it == safes.end() || a < it->start);
      safes.insert(IntervalWithID(a, start, id));
      if (end + EPS < b) {
        safes.insert(IntervalWithID(end, b, number_of_intervals++));
        return;
      }
    }
  }
  while (it != safes.end() && it->end <= end + EPS) {
    auto next = it;
    ++next;
    safes.erase(it);
    it = next;
  }
  if (it != safes.end() && it->start < end) {
    double b = it->end;
    int id = it->id;
    safes.erase(it);
    safes.insert(IntervalWithID(end, b, id));
  }
}

void PrioritizedSIPP::addEdgeCollisionInterval(std::set<Interval>& safes,
                                               const double start,
                                               const double end) {
  auto it = safes.lower_bound(Interval(start - EPS));
  if (it != safes.begin()) {
    auto prev = it;
    --prev;
    if (prev->end > start) {
      double a = prev->start, b = prev->end;
      safes.erase(prev);
      assert(it == safes.end() || a < it->start);
      safes.insert(Interval(a, start));
      if (end + EPS < b) {
        safes.insert(Interval(end, b));
        return;
      }
    }
  }
  while (it != safes.end() && it->end <= end + EPS) {
    auto next = it;
    ++next;
    safes.erase(it);
    it = next;
  }
  if (it != safes.end() && it->start < end) {
    double b = it->end;
    safes.erase(it);
    safes.insert(Interval(end, b));
  }
}

double PrioritizedSIPP::heuristic_distance(const int vertex_0,
                                           const int vertex_1) {
  return sqrt(pow(problem_instance_ptr->getCoordinateX(vertex_0) -
                      problem_instance_ptr->getCoordinateX(vertex_1),
                  2) +
              pow(problem_instance_ptr->getCoordinateY(vertex_0) -
                      problem_instance_ptr->getCoordinateY(vertex_1),
                  2));
}

void PrioritizedSIPP::solve() {
  auto start_time = std::chrono::system_clock::now();

  INF = problem_instance_ptr->INF;
  EPS = problem_instance_ptr->EPS;
  using Move = typename ProblemInstance::Move;

  int m = problem_instance_ptr
              ->getNumberOfVertices();  // the number of vertex in map

  int n = problem_instance_ptr->getNumberOfAgents();
  std::vector<int> priorities(n);
  std::iota(priorities.begin(), priorities.end(), 0);
  if (priority_strategy == 1) {
    std::random_device rd;
    std::default_random_engine eng(rd());
    std::shuffle(priorities.begin(), priorities.end(), eng);
  }

  // memorize the planned routes to collision check
  std::vector<std::vector<std::pair<int, int>>> routes(n);
  std::vector<std::vector<double>> time_stamps(n);

  auto& edges = problem_instance_ptr->edges;
  plan.clear();
  plan.resize(n);

  interval_data.clear();

  std::vector<std::set<IntervalWithID>> vertex_safes(m);
  std::vector<std::vector<std::set<Interval>>> edge_safes(m);

  struct CurrentState {
    int visited;
    std::set<Interval>::iterator edge_it;
    std::set<IntervalWithID>::iterator target_it;
    CurrentState() : visited(-1) {}
  };
  std::vector<std::vector<CurrentState>> current_states(m);
  for (int vertex = 0; vertex < m; vertex++) {
    vertex_safes[vertex].insert(IntervalWithID(0.0, INF, vertex));
    edge_safes[vertex] = std::vector<std::set<Interval>>(edges[vertex].size());
    current_states[vertex] = std::vector<CurrentState>(edges[vertex].size());
    for (int edge_id = 0; edge_id < edges[vertex].size(); edge_id++) {
      edge_safes[vertex][edge_id].insert(Interval(0.0, INF));
    }
  }
  number_of_intervals = m;
  interval_data.resize(m);

  for (int priority_order = 0; priority_order < n; priority_order++) {
    int agent = priorities[priority_order];  // agent id to plan

    // Use Dijkstra Algorithm to plan

    if (priority_order > 0) {
      int j = priorities[priority_order - 1];  // agent already planned
      // check collision to j
      for (int step = 0; step + 1 < time_stamps[j].size(); step++) {
        if (routes[j][step].second == -1) {
          auto edge_collision_list =
              problem_instance_ptr->getAllVECollisions(routes[j][step].first);
          for (auto& collision : edge_collision_list) {
            double collision_start =
                time_stamps[j][step] + std::get<2>(collision);
            double collision_end =
                time_stamps[j][step + 1] + std::get<3>(collision);
            if (collision_end > 0.0) {
              addEdgeCollisionInterval(
                  edge_safes[std::get<0>(collision)][std::get<1>(collision)],
                  collision_start, collision_end);
            }
          }
        } else {
          auto vertex_collision_list = problem_instance_ptr->getAllEVCollisions(
              routes[j][step].first, routes[j][step].second);
          for (auto& collision : vertex_collision_list) {
            double collision_start =
                time_stamps[j][step] + std::get<1>(collision);
            double collision_end =
                time_stamps[j][step] + std::get<2>(collision);
            if (collision_end > 0.0) {
              addVertexCollisionInterval(vertex_safes[std::get<0>(collision)],
                                         collision_start, collision_end);
            }
          }
          auto edge_collision_list = problem_instance_ptr->getAllEECollisions(
              routes[j][step].first, routes[j][step].second);
          for (auto& collision : edge_collision_list) {
            double collision_start =
                time_stamps[j][step] + std::get<2>(collision);
            double collision_end =
                time_stamps[j][step] + std::get<3>(collision);
            if (collision_end > 0.0) {
              addEdgeCollisionInterval(
                  edge_safes[std::get<0>(collision)][std::get<1>(collision)],
                  collision_start, collision_end);
            }
          }
        }
      }
    }
    interval_data.resize(number_of_intervals);

    int initial_vertex = problem_instance_ptr->getAgentInitialVertex(agent),
        goal_vertex = problem_instance_ptr->getAgentGoalVertex(agent);
    int initial_interval_id = vertex_safes[initial_vertex].begin()->id;
    int goal_interval_id = vertex_safes[goal_vertex].rbegin()->id;
    struct Node {
      int vertex;
      std::set<IntervalWithID>::iterator it;
      double f_value;
      Node(const int _vertex, const std::set<IntervalWithID>::iterator _it,
           const double _f_value)
          : vertex(_vertex), it(_it), f_value(_f_value) {}
    } initial_node(initial_vertex, vertex_safes[initial_vertex].begin(),
                   heuristic_distance(initial_vertex, goal_vertex));

    auto node_compare = [](const Node& node0, const Node& node1) {
      return node0.f_value > node1.f_value;
    };
    std::priority_queue<Node, std::vector<Node>, decltype(node_compare)> Q(
        node_compare);
    Q.push(initial_node);
    interval_data[initial_interval_id].open_listed = priority_order;
    interval_data[initial_interval_id].best_time = 0.0;
    while (!Q.empty()) {
      int vertex = Q.top().vertex;
      auto it = Q.top().it;
      Q.pop();
      int interval_id = it->id;
      if (interval_data[interval_id].close_listed == priority_order) continue;
      interval_data[interval_id].close_listed = priority_order;
      if (interval_id == goal_interval_id) {
        break;
      }
      double current_time = interval_data[interval_id].best_time;
      double deadline = it->end;

      for (int edge_id = 0; edge_id < edges[vertex].size(); edge_id++) {
        int target = edges[vertex][edge_id];
        double length = problem_instance_ptr->getEdgeLength(vertex, edge_id);
        auto& current = current_states[vertex][edge_id];
        auto& edge_it = current.edge_it;
        auto& target_it = current.target_it;
        if (current.visited != priority_order) {
          current.visited = priority_order;
          edge_it = edge_safes[vertex][edge_id].begin();
          target_it = vertex_safes[target].begin();
        }
        for (; edge_it != edge_safes[vertex][edge_id].end() &&
               edge_it->start <= deadline + EPS;
             ++edge_it) {
          if (edge_it->end < current_time - EPS) {
            continue;
          }
          double arrival_start =
              std::max(current_time, edge_it->start) + length;
          double arrival_end = std::min(deadline, edge_it->end) + length;
          for (; target_it != vertex_safes[target].end() &&
                 target_it->start <= arrival_end + EPS;
               ++target_it) {
            if (target_it->end < arrival_start - EPS) {
              continue;
            }
            double arrival_time = std::max(arrival_start, target_it->start);
            int target_interval_id = target_it->id;
            if (interval_data[target_interval_id].open_listed !=
                    priority_order ||
                interval_data[target_interval_id].best_time - EPS >
                    arrival_time) {
              interval_data[target_interval_id].open_listed = priority_order;
              interval_data[target_interval_id].best_time = arrival_time;
              Q.push(
                  Node(target, target_it,
                       arrival_time + heuristic_distance(target, goal_vertex)));
              interval_data[target_interval_id].previous = interval_id;
              interval_data[target_interval_id].prev_vertex = vertex;
              interval_data[target_interval_id].prev_edge_id = edge_id;
            }
          }
        }
      }
    }
    if (interval_data[goal_interval_id].close_listed != priority_order) {
      success = false;
      return;
    }
    int vertex = goal_vertex, interval_id = goal_interval_id;
    time_stamps[agent].push_back(INF);
    routes[agent].push_back(std::make_pair(vertex, -1));
    time_stamps[agent].push_back(interval_data[interval_id].best_time);
    while (interval_id != initial_interval_id) {
      int previous_vertex = interval_data[interval_id].prev_vertex;
      int previous_edge_id = interval_data[interval_id].prev_edge_id;
      int previous_interval_id = interval_data[interval_id].previous;
      double start_time = interval_data[interval_id].best_time -
                          problem_instance_ptr->getEdgeLength(previous_vertex,
                                                              previous_edge_id);
      routes[agent].push_back(
          std::make_pair(previous_vertex, previous_edge_id));
      time_stamps[agent].push_back(start_time);
      plan[agent].push_back(Move(previous_edge_id, start_time));
      if (start_time > interval_data[previous_interval_id].best_time + EPS) {
        routes[agent].push_back(std::make_pair(previous_vertex, -1));
        time_stamps[agent].push_back(
            interval_data[previous_interval_id].best_time);
      }
      vertex = previous_vertex;
      interval_id = previous_interval_id;
    }
    std::reverse(plan[agent].begin(), plan[agent].end());
    std::reverse(routes[agent].begin(), routes[agent].end());
    std::reverse(time_stamps[agent].begin(), time_stamps[agent].end());

    auto current_duration = std::chrono::system_clock::now() - start_time;
    if (std::chrono::duration_cast<std::chrono::seconds>(current_duration)
            .count() >= time_limit) {
      success = false;
      return;
    }
  }
  success = true;
}
