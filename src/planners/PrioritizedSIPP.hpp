/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include <queue>
#include <random>
#include <set>

#include "base/planner.hpp"

class PrioritizedSIPP : public Planner {
  ProblemInstancePtr problem_instance_ptr;
  typename ProblemInstance::Plan plan;

  double INF, EPS;
  double time_limit;

  struct SafeIntervalData {
    int open_listed = -1, close_listed = -1;

    double best_time;                         // data for A*
    int previous, prev_vertex, prev_edge_id;  // data for route reconstruction
  };

  int number_of_intervals;
  std::vector<SafeIntervalData> interval_data;

  struct IntervalWithID {
    double start, end;
    int id;
    IntervalWithID(const double _start) : start(_start) {}
    IntervalWithID(const double _start, const double _end, const int _id)
        : start(_start), end(_end), id(_id) {}
    bool operator<(const IntervalWithID& another) const {
      return start < another.start;
    }
  };

  struct Interval {
    double start, end;
    Interval(const double _start) : start(_start) {}
    Interval(const double _start, const double _end)
        : start(_start), end(_end) {}
    bool operator<(const Interval& another) const {
      return start < another.start;
    }
  };

  void addVertexCollisionInterval(std::set<IntervalWithID>& safes,
                                  const double start, const double end);

  void addEdgeCollisionInterval(std::set<Interval>& safes, const double start,
                                const double end);

  double heuristic_distance(const int vertex_0, const int vertex_1);

  int priority_strategy;

 public:
  PrioritizedSIPP(const int priority_strategy = 0,
                  const double time_limit = 30.) {
    this->priority_strategy = priority_strategy;
    this->time_limit = time_limit;
  }

  void setInstance(ProblemInstancePtr problem_instance_ptr) override {
    this->problem_instance_ptr = problem_instance_ptr;
  }

  bool success;
  void solve() override;
  bool succeeded() override { return success; }
  typename ProblemInstance::Plan getPlan() override { return plan; }
};
