/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <numeric>

#include "base/geometry_2D_for_generator.hpp"
#include "base/memory_usage.hpp"
#include "base/planner.hpp"
#include "base/problem_with_explicit_collisions.hpp"
#include "base/roadmap_generator.hpp"
#include "base/spatial_planner.hpp"

class RunPlanner {
 public:
  static int duration_to_int(const std::chrono::duration<double>& duration) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration)
        .count();
  }
  bool success;
  int used_time, used_time_for_collisions;
  double used_memory, makespan, sum_of_costs;
  int number_of_CC, number_of_IC;

  RunPlanner(PlannerPtr planner_ptr,
             const ProblemInstancePtr problem_instance_ptr) {
    auto start_time = std::chrono::system_clock::now();
    auto current_used_time_for_collisions = problem_instance_ptr->used_time;
    int current_number_of_CC = problem_instance_ptr->number_of_collision_checks;
    int current_number_of_IC =
        problem_instance_ptr->number_of_interval_calculations;
    try {
      planner_ptr->solve();
    } catch (std::runtime_error& e) {
      fprintf(stderr, "runtime error when planning: %s\n", e.what());
      success = false;
      return;
    }
    used_time = duration_to_int(std::chrono::system_clock::now() - start_time);
    used_time_for_collisions = duration_to_int(
        problem_instance_ptr->used_time - current_used_time_for_collisions);
    used_memory = getUsedMemory();
    number_of_CC =
        problem_instance_ptr->number_of_collision_checks - current_number_of_CC;
    number_of_IC = problem_instance_ptr->number_of_interval_calculations -
                   current_number_of_IC;
    if (!planner_ptr->succeeded()) {
      success = false;
      return;
    }
    auto plan = planner_ptr->getPlan();
    if (!problem_instance_ptr->checkPlanValidity(plan, makespan,
                                                 sum_of_costs)) {
      fprintf(stderr, "Planner output is invalid\n");
      success = false;
      return;
    }
    success = true;
  }
};

class RunSpatialPlanner {
 public:
  bool success;
  int used_time;
  double used_memory, makespan, sum_of_costs;

  RunSpatialPlanner(
      SpatialPlannerPtr<Space2D> planner_ptr,
      const SpatialProblemInstancePtr<Space2D> problem_instance_ptr) {
    auto start_time = std::chrono::system_clock::now();
    try {
      planner_ptr->solve();
    } catch (std::runtime_error& e) {
      fprintf(stderr, "runtime error when planning: %s\n", e.what());
      success = false;
      return;
    }
    used_time = RunPlanner::duration_to_int(std::chrono::system_clock::now() -
                                            start_time);
    used_memory = getUsedMemory();
    if (!planner_ptr->succeeded()) {
      success = false;
      return;
    }
    auto plan = planner_ptr->getPlan();
    if (!problem_instance_ptr->checkPlanValidity(plan, makespan,
                                                 sum_of_costs)) {
      fprintf(stderr, "Planner output is invalid\n");
      success = false;
      return;
    }
    success = true;
  }
};

class Benchmark {
 public:
  struct Result {
    std::string problem_instance_name, planner_name;
    int iterations, success_count = 0, number_of_CC = 0, number_of_IC = 0;
    double used_time = 0., used_memory = 0., used_time_for_collisions = 0.,
           makespan = 0., sum_of_costs = 0.;
  };

  int iterations = 0;
  double time_limit = 0.0;

  std::map<std::string, ProblemInstancePtr> problem_instances;
  std::map<std::string, PlannerPtr> planners;

  void addProblemInstance(const ProblemInstancePtr& problem_instance_ptr,
                          const std::string& name) {
    problem_instances[name] = problem_instance_ptr;
  }
  void addPlanner(const PlannerPtr& planner_ptr, const std::string& name) {
    planners[name] = planner_ptr;
  }

  std::vector<Result> results;

  void printPlan(const ProblemInstancePtr& problem_instance_ptr,
                 const ProblemInstance::Plan& plan,
                 const std::string& file_name) {
    FILE* out = file_name == "stdout" ? stdout : fopen(file_name.c_str(), "a");
    fprintf(out, "%d %lf\n", (int)plan.size(),
            problem_instance_ptr->getAgentRadius());
    for (int agent = 0; agent < plan.size(); agent++) {
      int vertex = problem_instance_ptr->tasks[agent].initial_vertex;
      double time = 0.0;
      std::vector<std::pair<int, double>> route(1,
                                                std::make_pair(vertex, time));
      for (int move = 0; move < plan[agent].size(); move++) {
        if (time < plan[agent][move].start_time) {
          time = plan[agent][move].start_time;
          route.push_back(std::make_pair(vertex, time));
        }
        time += problem_instance_ptr->getEdgeLength(vertex,
                                                    plan[agent][move].edge_id);
        vertex = problem_instance_ptr->getTargetVertex(
            vertex, plan[agent][move].edge_id);
        route.push_back(std::make_pair(vertex, time));
      }
      fprintf(out, "%d\n", (int)route.size());
      for (int t = 0; t < route.size(); t++) {
        fprintf(out, "%lf\n", route[t].second);
        problem_instance_ptr->printVertex(out, route[t].first);
      }
    }
    if (out != stdout) {
      fclose(out);
    }
  }

  void run(const typename std::map<std::string, ProblemInstancePtr>::iterator&
               problem_instance_it,
           typename std::map<std::string, PlannerPtr>::iterator& planner_it,
           const std::string output_file = "") {
    auto& problem_instance_ptr = problem_instance_it->second;
    auto& planner_ptr = planner_it->second;
    planner_ptr->setInstance(problem_instance_ptr);
    int count = 0, success_count = 0, number_of_CC = 0, number_of_IC = 0;
    double used_time = 0.0, used_time_for_collisions = 0.0, used_memory = 0.0,
           makespan = 0.0, sum_of_costs = 0.0, all_used_time = 0.0;
    while (count < iterations ||
           (iterations <= 0 && all_used_time < time_limit)) {
      count++;
      RunPlanner single_run(planner_ptr, problem_instance_ptr);
      all_used_time += used_time;
      if (single_run.success) {
        used_time += single_run.used_time;
        used_memory += single_run.used_memory;
        makespan += single_run.makespan;
        sum_of_costs += single_run.sum_of_costs;
        number_of_CC += single_run.number_of_CC;
        number_of_IC += single_run.number_of_IC;
        used_time_for_collisions += single_run.used_time_for_collisions;
        success_count++;

        if (output_file != "") {
          printPlan(problem_instance_ptr, planner_ptr->getPlan(), output_file);
        }
      }
    }
    Result result;
    result.problem_instance_name = problem_instance_it->first;
    result.planner_name = planner_it->first;
    result.iterations = count;
    result.success_count = success_count;
    if (success_count > 0) {
      result.used_time = used_time / success_count;
      result.used_memory = used_memory / success_count;
      result.makespan = makespan / success_count;
      result.sum_of_costs = sum_of_costs / success_count;
      result.number_of_CC = number_of_CC / success_count;
      result.number_of_IC = number_of_IC / success_count;
      result.used_time_for_collisions =
          used_time_for_collisions / success_count;
    }
    results.push_back(result);
  }

  void runSpecified(const std::string& problem_instance_name,
                    const std::string& planner_name, const int agents_limit = 0,
                    const std::string output_file = "") {
    auto problem_instance_it = problem_instances.find(problem_instance_name);
    if (problem_instance_it == problem_instances.end()) {
      fprintf(stderr, "Unknown problem instance name");
      return;
    }
    auto planner_it = planners.find(planner_name);
    if (planner_it == planners.end()) {
      fprintf(stderr, "Unknown planner name");
      return;
    }
    if (agents_limit > 0 &&
        problem_instance_it->second->tasks.size() > agents_limit) {
      problem_instance_it->second->tasks.resize(agents_limit);
    }
    run(problem_instance_it, planner_it, output_file);
  }

  std::set<std::pair<std::string, std::string>> pairs_to_run;

  void addPairToRun(const std::string& problem_instance_name,
                    const std::string& planner_name) {
    assert(planners.find(planner_name) != planners.end());
    pairs_to_run.insert(std::make_pair(problem_instance_name, planner_name));
  }

  void runIncrementallyAllPairs(
      const int iterations,
      std::vector<std::vector<std::pair<int, int>>> permutations) {
    for (auto& pair : pairs_to_run) {
      for (int count = 0; count < iterations; count++) {
        assert(problem_instances.find(pair.first + std::to_string(count)) !=
               problem_instances.end());
      }
    }
    std::map<std::string, std::vector<ProblemInstance::AgentTask>> all_tasks;
    for (auto& pair : problem_instances) {
      all_tasks[pair.first] = pair.second->tasks;
    }
    int max_number_of_agents = all_tasks.begin()->second.size();
    if (permutations.size() == 0) {
      std::vector<std::pair<int, int>> permutation(max_number_of_agents);
      for (int i = 0; i < max_number_of_agents; i++) {
        permutation[i].first = permutation[i].second = i;
      }
      permutations.push_back(permutation);
    }
    int number_of_perms = permutations.size();
    std::vector<decltype(pairs_to_run)> pairs_to_runs(iterations *
                                                      number_of_perms);
    for (int count = 0; count < pairs_to_runs.size(); count++) {
      pairs_to_runs[count] = pairs_to_run;
    }
    for (int number_of_agents = 1; number_of_agents <= max_number_of_agents;
         number_of_agents++) {
      for (int perm_id = 0; perm_id < number_of_perms; perm_id++) {
        for (int count = 0; count < iterations; count++) {
          int prob_id = perm_id * iterations + count;
          auto permutation = permutations[perm_id];
          for (auto pair_it = pairs_to_runs[prob_id].begin();
               pair_it != pairs_to_runs[prob_id].end();) {
            std::string problem_name = pair_it->first + std::to_string(count);
            auto problem_instance = problem_instances[problem_name];
            auto planner = planners[pair_it->second];
            problem_instance->tasks.resize(number_of_agents);
            for (int i = 0; i < number_of_agents; i++) {
              problem_instance->tasks[i].initial_vertex =
                  all_tasks[problem_name][permutation[i].first].initial_vertex;
              problem_instance->tasks[i].goal_vertex =
                  all_tasks[problem_name][permutation[i].second].goal_vertex;
            }
            planner->setInstance(problem_instance);
            RunPlanner run(planner, problem_instance);
            if (run.success) {
              printf("%11s,%6s,%4d,%4d,%4d,%10.3lf,%10.3lf,%10d\n",
                     pair_it->first.c_str(), pair_it->second.c_str(),
                     number_of_agents, perm_id, count, run.makespan,
                     run.sum_of_costs, run.used_time);
              ++pair_it;
            } else {
              printf("%11s,%6s,%4d,%4d,%4d,%10.3lf,%10.3lf,%10d\n",
                     pair_it->first.c_str(), pair_it->second.c_str(),
                     number_of_agents, perm_id, count, -1.0, -1.0,
                     run.used_time);
              auto temp = std::next(pair_it);
              pairs_to_runs[prob_id].erase(pair_it);
              pair_it = temp;
            }
          }
        }
      }
    }
  }

  void runAll(const std::string output_file = "") {
    for (auto planner_it = planners.begin(); planner_it != planners.end();
         planner_it++) {
      for (auto problem_instance_it = problem_instances.begin();
           problem_instance_it != problem_instances.end();
           problem_instance_it++) {
        run(problem_instance_it, planner_it, output_file);
      }
    }
  }

  void displayAllResults() {
    fprintf(
        stderr,
        "   planner    problem    success   makespan sumofcosts  ptime(ms)  "
        "ctime(ms) "
        "memory(kB)    countCC    countIC iterations\n");
    for (auto& result : results) {
      fprintf(
          stderr,
          "%10s %10s %10lf %10.5lf %10.5lf %10.0lf %10.0lf %10.0lf %10d %10d "
          "%10d\n",
          result.planner_name.c_str(), result.problem_instance_name.c_str(),
          (double)result.success_count / result.iterations, result.makespan,
          result.sum_of_costs,
          result.used_time - result.used_time_for_collisions,
          result.used_time_for_collisions, result.used_memory,
          result.number_of_CC, result.number_of_IC, result.iterations);
    }
  }

  using ProblemInstance2D = ProblemInstanceWithGeometry<Geometry2D>;
  using RoadmapGeneratorPtr = std::shared_ptr<RoadmapGenerator<Space2D>>;
  std::map<std::string, RoadmapGeneratorPtr> generators;
  void addGenerator(const std::string& name,
                    const RoadmapGeneratorPtr& generator) {
    generators[name] = generator;
  }
  std::map<std::string, SpatialPlannerPtr<Space2D>> spatial_planners;
  void addSpatialPlanner(const std::string& name,
                         const SpatialPlannerPtr<Space2D> planner) {
    spatial_planners[name] = planner;
  }
  void RoadmapBenchmarkAll(
      const SpatialProblemInstancePtr<Space2D>& spatial_problem_instance,
      const std::vector<std::vector<int>>& subinstances, PlannerPtr planner,
      const bool precalc = false, const int iterations = 1) {
    printf("%d,%d,%d\n", (int)generators.size(), (int)subinstances.size(),
           iterations);
    printf(
        "generator_name, generation_time, precalc_time, subinstance_id, "
        "planning_time, "
        "makespan, sumofcosts\n");
    for (auto& pair : generators) {
      ProblemInstancePtr problem_instance;
      auto start_time = std::chrono::system_clock::now();
      try {
        pair.second->generate(*spatial_problem_instance);
      } catch (...) {
        printf("%s,%d,%d\n", pair.first.c_str(), -1, -1);
        continue;
      }
      int generation_time = RunPlanner::duration_to_int(
          std::chrono::system_clock::now() - start_time);
      int precalc_time;
      if (!precalc) {
        problem_instance =
            std::make_shared<ProblemInstance2D>(pair.second->problem_instance);
        precalc_time = 0;
      } else {
        auto start_time = std::chrono::system_clock::now();
        try {
          problem_instance = ProblemInstanceWithExplicitCollisionsPtr(
              new ProblemInstanceWithExplicitCollisions(
                  pair.second->problem_instance));
        } catch (...) {
          printf("%s,%d,%d\n", pair.first.c_str(), generation_time, -1);
          continue;
        }
        precalc_time = RunPlanner::duration_to_int(
            std::chrono::system_clock::now() - start_time);
      }

      printf("%s,%d,%d\n", pair.first.c_str(), generation_time, precalc_time);
      auto original_tasks = problem_instance->tasks;
      for (int subinstance_id = 0; subinstance_id < subinstances.size();
           subinstance_id++) {
        auto& subinstance = subinstances[subinstance_id];
        problem_instance->tasks.clear();
        for (auto agent : subinstance) {
          problem_instance->tasks.push_back(original_tasks[agent]);
        }
        planner->setInstance(problem_instance);
        for (int it = 0; it < iterations; it++) {
          RunPlanner run(planner, problem_instance);
          if (run.success) {
            printf(",,,%d,%d,%lf,%lf\n", subinstance_id, run.used_time,
                   run.makespan, run.sum_of_costs);
          } else {
            printf(",,,%d,%lf,%lf,%lf\n", subinstance_id, -1., -1., -1.);
          }
        }
      }
    }
    auto original_spatial_tasks = spatial_problem_instance->tasks;
    for (auto& pair : spatial_planners) {
      printf("%s\n", pair.first.c_str());
      pair.second->setInstanceMap(spatial_problem_instance);
      for (int subinstance_id = 0; subinstance_id < subinstances.size();
           subinstance_id++) {
        auto& subinstance = subinstances[subinstance_id];
        spatial_problem_instance->tasks.clear();
        for (auto agent : subinstance) {
          spatial_problem_instance->tasks.push_back(
              original_spatial_tasks[agent]);
        }
        pair.second->setInstanceTasks(spatial_problem_instance);
        for (int it = 0; it < iterations; it++) {
          RunSpatialPlanner run(pair.second, spatial_problem_instance);
          if (run.success) {
            printf(",,,%d,%d,%lf,%lf\n", subinstance_id, run.used_time,
                   run.makespan, run.sum_of_costs);
          } else {
            printf(",,,%d,%lf,%lf,%lf\n", subinstance_id, -1., -1., -1.);
          }
        }
      }
    }
  }
};
