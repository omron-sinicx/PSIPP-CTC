/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura

This file contains modified copies from
https://github.com/thaynewalker/CCBS, which has the following license:

MIT License

Copyright (c) 2022 Anton Andreychuk

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "planners/WrappedContinuousCBS2.hpp"

#include "Continuous-CBS/config.h"
#include "Continuous-CBS/heuristic.h"
#include "Continuous-CBS/simplex/pilal.h"
#include "Continuous-CBS/simplex/simplex.h"
#include "Continuous-CBS/sipp.h"
#include "Continuous-CBS/structs.h"
#include "Continuous-CBS/task.h"

namespace CCBS {

Solution find_solution();
bool init_root(const Map &map, const Task &task);
std::list<Constraint> get_constraints(CBS_Node *node, int agent_id = -1);
// std::list<Constraint> merge_constraints(std::list<Constraint> constraints);
bool validate_constraints(std::list<Constraint> constraints, int agent);
bool check_positive_constraints(std::list<Constraint> constraints,
                                Constraint constraint);
Conflict check_paths(const sPath &pathA, const sPath &pathB);
bool check_conflict(Move move1, Move move2);
double get_hl_heuristic(const std::list<Conflict> &conflicts);
std::vector<Conflict> get_all_conflicts(const std::vector<sPath> &paths,
                                        int id);
Constraint get_constraint(int agent, Move move1, Move move2);
Constraint get_wait_constraint(int agent, Move move1, Move move2);
void find_new_conflicts(const Map &map, const Task &task, CBS_Node &node,
                        std::vector<sPath> &paths, const sPath &path,
                        const std::list<Conflict> &conflicts,
                        const std::list<Conflict> &semicard_conflicts,
                        const std::list<Conflict> &cardinal_conflicts,
                        int &low_level_searches, int &low_level_expanded);
double get_cost(CBS_Node node, int agent_id);
std::vector<sPath> get_paths(CBS_Node *node, unsigned int agents_size);
Conflict get_conflict(std::list<Conflict> &conflicts);
CBS_Tree tree;
SIPP planner;
Solution solution;
Heuristic h_values;
Config config;
std::unique_ptr<Map> map;
std::unique_ptr<Task> task;

ProblemInstancePtr prob;

bool check_conflict(Move move1, Move move2) {
  return prob->CollisionCheck2(move1.id1, move1.t1, move1.id2, move1.t2,
                               move2.id1, move2.t1, move2.id2, move2.t2);
}

Constraint get_wait_constraint(int agent, Move move1, Move move2) {
  double lower, upper;
  prob->getCollisionInterval2(move1.id1, move2.id1, move2.t1, move2.id2,
                              move2.t2, lower, upper);
  return Constraint(agent, lower, upper, move1.id1, move1.id1);
}

Constraint get_constraint(int agent, Move move1, Move move2) {
  if (move1.id1 == move1.id2) return get_wait_constraint(agent, move1, move2);
  double lower, upper;
  prob->getCollisionInterval2(move1.id1, move1.id2, move2.id1, move2.t1,
                              move2.id2, move2.t2, lower, upper);
  return Constraint(agent, lower, upper, move1.id1, move1.id2);
}

bool init_root(const Map &map, const Task &task) {
  CBS_Node root;
  tree.set_focal_weight(config.focal_weight);
  sPath path;
  for (int i = 0; i < int(task.get_agents_size()); i++) {
    Agent agent = task.get_agent(i);
    path = planner.find_path(agent, map, {}, h_values);
    if (path.cost < 0) return false;
    root.paths.push_back(path);
    root.cost += path.cost;
  }
  root.low_level_expanded = 0;
  root.parent = nullptr;
  root.id = 1;
  root.id_str = "1";
  auto conflicts = get_all_conflicts(root.paths, -1);
  root.conflicts_num = conflicts.size();

  for (auto conflict : conflicts)
    if (!config.use_cardinal)
      root.conflicts.push_back(conflict);
    else {
      auto pathA = planner.find_path(
          task.get_agent(conflict.agent1), map,
          {get_constraint(conflict.agent1, conflict.move1, conflict.move2)},
          h_values);
      auto pathB = planner.find_path(
          task.get_agent(conflict.agent2), map,
          {get_constraint(conflict.agent2, conflict.move2, conflict.move1)},
          h_values);
      // conflict.path1 = pathA;
      // conflict.path2 = pathB;
      if (pathA.cost > root.paths[conflict.agent1].cost &&
          pathB.cost > root.paths[conflict.agent2].cost) {
        conflict.overcost =
            std::min(pathA.cost - root.paths[conflict.agent1].cost,
                     pathB.cost - root.paths[conflict.agent2].cost);
        root.cardinal_conflicts.push_back(conflict);
      } else if (pathA.cost > root.paths[conflict.agent1].cost ||
                 pathB.cost > root.paths[conflict.agent2].cost)
        root.semicard_conflicts.push_back(conflict);
      else
        root.conflicts.push_back(conflict);
    }
  solution.init_cost = root.cost;
  tree.add_node(root);
  return true;
}

double get_hl_heuristic(const std::list<Conflict> &conflicts) {
  if (conflicts.empty() || config.hlh_type == 0)
    return 0;
  else if (config.hlh_type == 1) {
    optimization::Simplex simplex("simplex");
    std::map<int, int> colliding_agents;
    for (auto c : conflicts) {
      colliding_agents.insert({c.agent1, colliding_agents.size()});
      colliding_agents.insert({c.agent2, colliding_agents.size()});
    }

    pilal::Matrix coefficients(conflicts.size(), colliding_agents.size(), 0);
    std::vector<double> overcosts(conflicts.size());
    int i(0);
    for (auto c : conflicts) {
      coefficients.at(i, colliding_agents.at(c.agent1)) = 1;
      coefficients.at(i, colliding_agents.at(c.agent2)) = 1;
      overcosts[i] = c.overcost;
      i++;
    }
    simplex.set_problem(coefficients, overcosts);
    simplex.solve();
    return simplex.get_solution();
  } else {
    double h_value(0);
    std::vector<std::tuple<double, int, int>> values;
    values.reserve(conflicts.size());
    std::set<int> used;
    for (auto c : conflicts)
      values.push_back(std::make_tuple(c.overcost, c.agent1, c.agent2));
    std::sort(values.begin(), values.end(),
              std::greater<std::tuple<double, int, int>>());
    for (auto v : values) {
      if (used.find(get<1>(v)) != used.end() ||
          used.find(get<2>(v)) != used.end())
        continue;
      h_value += get<0>(v);
      used.insert(get<1>(v));
      used.insert(get<2>(v));
    }
    return h_value;
  }
}

Conflict get_conflict(std::list<Conflict> &conflicts) {
  auto best_it = conflicts.begin();
  for (auto it = conflicts.begin(); it != conflicts.end(); it++) {
    if (it->overcost > 0) {
      if (best_it->overcost < it->overcost ||
          (fabs(best_it->overcost - it->overcost) < CN_EPSILON &&
           best_it->t < it->t))
        best_it = it;
    } else if (best_it->t < it->t)
      best_it = it;
  }

  Conflict conflict = *best_it;
  conflicts.erase(best_it);
  return conflict;
}

Solution find_solution() {
  h_values.init(map->get_size(), task->get_agents_size());
  for (int i = 0; i < int(task->get_agents_size()); i++) {
    Agent agent = task->get_agent(i);
    h_values.count(*map, agent);
  }
  auto t = std::chrono::high_resolution_clock::now();
  int cardinal_solved = 0, semicardinal_solved = 0;
  if (!init_root(*map, *task)) return solution;
  solution.init_time =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::high_resolution_clock::now() - t);
  solution.found = true;
  CBS_Node node;
  std::chrono::duration<double> time_spent;
  int expanded(1);
  double time(0);
  std::list<Conflict> conflicts;
  Conflict conflict;
  std::vector<int> conflicting_agents;
  std::vector<std::pair<int, int>> conflicting_pairs;
  int low_level_searches(0);
  int low_level_expanded(0);
  int id = 2;
  do {
    auto parent = tree.get_front();
    node = *parent;
    node.cost -= node.h;
    parent->conflicts.clear();
    parent->cardinal_conflicts.clear();
    parent->semicard_conflicts.clear();
    auto paths = get_paths(&node, task->get_agents_size());

    auto time_now = std::chrono::high_resolution_clock::now();
    conflicts = node.conflicts;
    auto cardinal_conflicts = node.cardinal_conflicts;
    auto semicard_conflicts = node.semicard_conflicts;
    if (conflicts.empty() && semicard_conflicts.empty() &&
        cardinal_conflicts.empty()) {
      break;  // i.e. no conflicts => solution found
    }
    if (!cardinal_conflicts.empty()) {
      conflict = get_conflict(cardinal_conflicts);
      cardinal_solved++;
    } else if (!semicard_conflicts.empty()) {
      conflict = get_conflict(semicard_conflicts);
      semicardinal_solved++;
    } else
      conflict = get_conflict(conflicts);
    time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::high_resolution_clock::now() - time_now);
    time += time_spent.count();
    expanded++;

    std::list<Constraint> constraintsA =
        get_constraints(&node, conflict.agent1);
    Constraint constraintA(
        get_constraint(conflict.agent1, conflict.move1, conflict.move2));
    constraintsA.push_back(constraintA);
    sPath pathA;
    // if(!config.use_cardinal || !config.cache_paths)
    {
      pathA = planner.find_path(task->get_agent(conflict.agent1), *map,
                                constraintsA, h_values);
      low_level_searches++;
      low_level_expanded += pathA.expanded;
    }
    // else
    //    pathA = conflict.path1;

    std::list<Constraint> constraintsB =
        get_constraints(&node, conflict.agent2);
    Constraint constraintB =
        get_constraint(conflict.agent2, conflict.move2, conflict.move1);
    constraintsB.push_back(constraintB);
    sPath pathB;
    // if(!config.use_cardinal || !config.cache_paths)
    {
      pathB = planner.find_path(task->get_agent(conflict.agent2), *map,
                                constraintsB, h_values);
      low_level_searches++;
      low_level_expanded += pathB.expanded;
    }
    // else
    //    pathB = conflict.path2;

    CBS_Node right({pathA}, parent, constraintA,
                   node.cost + pathA.cost - get_cost(node, conflict.agent1), 0,
                   node.total_cons + 1);
    CBS_Node left({pathB}, parent, constraintB,
                  node.cost + pathB.cost - get_cost(node, conflict.agent2), 0,
                  node.total_cons + 1);
    Constraint positive;
    bool inserted = false;
    bool left_ok = true, right_ok = true;
    if (config.use_disjoint_splitting) {
      int agent1positives(0), agent2positives(0);
      for (auto c : constraintsA)
        if (c.positive) agent1positives++;
      for (auto c : constraintsB)
        if (c.positive) agent2positives++;
      if (conflict.move1.id1 != conflict.move1.id2 &&
          agent2positives > agent1positives && pathA.cost > 0) {
        positive = Constraint(conflict.agent1, constraintA.t1, constraintA.t2,
                              conflict.move1.id1, conflict.move1.id2, true);
        if (check_positive_constraints(constraintsA, positive)) {
          left.positive_constraint = positive;
          left.total_cons++;
          constraintsB.push_back(left.positive_constraint);
          inserted = true;
          // std::cout<<"added positive to "<<positive.agent<<"\n\n";
        }
        // else
        //    right_ok = false;
      }
      if (conflict.move2.id1 != conflict.move2.id2 && !inserted &&
          pathB.cost > 0) {
        positive = Constraint(conflict.agent2, constraintB.t1, constraintB.t2,
                              conflict.move2.id1, conflict.move2.id2, true);
        if (check_positive_constraints(constraintsB, positive)) {
          right.positive_constraint = positive;
          right.total_cons++;
          constraintsA.push_back(right.positive_constraint);
          inserted = true;
        }
        // else
        //    left_ok = false;
      }
      if (conflict.move1.id1 != conflict.move1.id2 && !inserted &&
          pathA.cost > 0) {
        positive = Constraint(conflict.agent1, constraintA.t1, constraintA.t2,
                              conflict.move1.id1, conflict.move1.id2, true);
        if (check_positive_constraints(constraintsA, positive)) {
          inserted = true;
          left.positive_constraint = positive;
          left.total_cons++;
          constraintsB.push_back(left.positive_constraint);
        }
        // else
        //    right_ok = false;
      }
    }
    right.id_str = node.id_str + "0";
    left.id_str = node.id_str + "1";
    right.id = id++;
    left.id = id++;
    if (right_ok && pathA.cost > 0 &&
        validate_constraints(constraintsA, pathA.agentID)) {
      time_now = std::chrono::high_resolution_clock::now();
      find_new_conflicts(*map, *task, right, paths, pathA, conflicts,
                         semicard_conflicts, cardinal_conflicts,
                         low_level_searches, low_level_expanded);
      time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::high_resolution_clock::now() - time_now);
      time += time_spent.count();
      if (right.cost > 0) {
        right.h = get_hl_heuristic(right.cardinal_conflicts);
        right.cost += right.h;
        tree.add_node(right);
      }
    }
    if (left_ok && pathB.cost > 0 &&
        validate_constraints(constraintsB, pathB.agentID)) {
      time_now = std::chrono::high_resolution_clock::now();
      find_new_conflicts(*map, *task, left, paths, pathB, conflicts,
                         semicard_conflicts, cardinal_conflicts,
                         low_level_searches, low_level_expanded);
      time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::high_resolution_clock::now() - time_now);
      time += time_spent.count();
      if (left.cost > 0) {
        left.h = get_hl_heuristic(left.cardinal_conflicts);
        left.cost += left.h;
        tree.add_node(left);
      }
    }
    time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::high_resolution_clock::now() - t);
    if (time_spent.count() > config.timelimit) {
      solution.found = false;
      break;
    }
  } while (tree.get_open_size() > 0);
  solution.paths = get_paths(&node, task->get_agents_size());
  solution.flowtime = node.cost;
  solution.low_level_expansions = low_level_searches;
  solution.low_level_expanded =
      double(low_level_expanded) / std::max(low_level_searches, 1);
  solution.high_level_expanded = expanded;
  solution.high_level_generated = int(tree.get_size());
  for (auto path : solution.paths)
    solution.makespan =
        (solution.makespan > path.cost) ? solution.makespan : path.cost;
  solution.time = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::high_resolution_clock::now() - t);
  solution.check_time = time;
  solution.cardinal_solved = cardinal_solved;
  solution.semicardinal_solved = semicardinal_solved;

  return solution;
}

bool check_positive_constraints(std::list<Constraint> constraints,
                                Constraint constraint) {
  std::list<Constraint> positives;
  for (auto c : constraints)
    if (c.positive && c.agent == constraint.agent) positives.push_back(c);

  for (auto p : positives) {
    if (p.id1 == constraint.id1 && p.id2 == constraint.id2 &&
        p.t1 - CN_EPSILON < constraint.t1 &&
        p.t2 + CN_EPSILON >
            constraint.t2)  // agent needs to perform two equal actions
                            // simultaneously => it's impossible
      return false;
    if (p.id1 == constraint.id1 && p.id2 == constraint.id2 &&
        constraint.t1 - CN_EPSILON < p.t1 && constraint.t2 + CN_EPSILON > p.t2)
      return false;
  }
  return true;
}

bool validate_constraints(std::list<Constraint> constraints, int agent) {
  std::list<Constraint> positives;
  for (auto c : constraints)
    if (c.positive && c.agent == agent) positives.push_back(c);
  for (auto p : positives)
    for (auto c : constraints) {
      if (c.positive) continue;
      if (p.agent == c.agent && p.id1 == c.id1 &&
          p.id2 == c.id2)  // if the same action
        if (p.t1 > c.t1 - CN_EPSILON &&
            p.t2 < c.t2 + CN_EPSILON)  // if the whole positive interval is
                                       // inside collision interval
          return false;
    }
  return true;
}

void find_new_conflicts(const Map &map, const Task &task, CBS_Node &node,
                        std::vector<sPath> &paths, const sPath &path,
                        const std::list<Conflict> &conflicts,
                        const std::list<Conflict> &semicard_conflicts,
                        const std::list<Conflict> &cardinal_conflicts,
                        int &low_level_searches, int &low_level_expanded) {
  auto oldpath = paths[path.agentID];
  paths[path.agentID] = path;
  auto new_conflicts = get_all_conflicts(paths, path.agentID);
  paths[path.agentID] = oldpath;
  std::list<Conflict> conflictsA({}), semicard_conflictsA({}),
      cardinal_conflictsA({});
  for (auto c : conflicts)
    if (c.agent1 != path.agentID && c.agent2 != path.agentID)
      conflictsA.push_back(c);
  for (auto c : semicard_conflicts)
    if (c.agent1 != path.agentID && c.agent2 != path.agentID)
      semicard_conflictsA.push_back(c);
  for (auto c : cardinal_conflicts)
    if (c.agent1 != path.agentID && c.agent2 != path.agentID)
      cardinal_conflictsA.push_back(c);
  if (!config.use_cardinal) {
    node.conflicts = conflictsA;
    for (auto n : new_conflicts) node.conflicts.push_back(n);
    node.cardinal_conflicts.clear();
    node.semicard_conflicts.clear();
    node.conflicts_num = node.conflicts.size();
    return;
  }
  for (auto c : new_conflicts) {
    std::list<Constraint> constraintsA, constraintsB;
    if (path.agentID == c.agent1) {
      constraintsA = get_constraints(&node, c.agent1);
      constraintsA.push_back(get_constraint(c.agent1, c.move1, c.move2));
      auto new_pathA = planner.find_path(task.get_agent(c.agent1), map,
                                         constraintsA, h_values);
      constraintsB = get_constraints(&node, c.agent2);
      constraintsB.push_back(get_constraint(c.agent2, c.move2, c.move1));
      auto new_pathB = planner.find_path(task.get_agent(c.agent2), map,
                                         constraintsB, h_values);
      double old_cost = get_cost(node, c.agent2);
      // c.path1 = new_pathA;
      // c.path2 = new_pathB;
      if (new_pathA.cost < 0 && new_pathB.cost < 0) {
        node.cost = -1;
        return;
      } else if (new_pathA.cost < 0) {
        c.overcost = new_pathB.cost - old_cost;
        cardinal_conflictsA.push_back(c);
      } else if (new_pathB.cost < 0) {
        c.overcost = new_pathA.cost - path.cost;
        cardinal_conflictsA.push_back(c);
      } else if (new_pathA.cost > path.cost && new_pathB.cost > old_cost) {
        c.overcost =
            std::min(new_pathA.cost - path.cost, new_pathB.cost - old_cost);
        cardinal_conflictsA.push_back(c);
      } else if (new_pathA.cost > path.cost || new_pathB.cost > old_cost)
        semicard_conflictsA.push_back(c);
      else
        conflictsA.push_back(c);
      low_level_searches += 2;
      low_level_expanded += (new_pathA.expanded + new_pathB.expanded);
    } else {
      constraintsA = get_constraints(&node, c.agent2);
      constraintsA.push_back(get_constraint(c.agent2, c.move2, c.move1));
      auto new_pathA = planner.find_path(task.get_agent(c.agent2), map,
                                         constraintsA, h_values);
      constraintsB = get_constraints(&node, c.agent1);
      constraintsB.push_back(get_constraint(c.agent1, c.move1, c.move2));
      auto new_pathB = planner.find_path(task.get_agent(c.agent1), map,
                                         constraintsB, h_values);
      double old_cost = get_cost(node, c.agent1);
      // c.path1 = new_pathB;
      // c.path2 = new_pathA;
      if (new_pathA.cost < 0 && new_pathB.cost < 0) {
        node.cost = -1;
        return;
      } else if (new_pathA.cost < 0) {
        c.overcost = new_pathB.cost - old_cost;
        cardinal_conflictsA.push_back(c);
      } else if (new_pathB.cost < 0) {
        c.overcost = new_pathA.cost - path.cost;
        cardinal_conflictsA.push_back(c);
      } else if (new_pathA.cost > path.cost && new_pathB.cost > old_cost) {
        c.overcost =
            std::min(new_pathA.cost - path.cost, new_pathB.cost - old_cost);
        cardinal_conflictsA.push_back(c);
      } else if (new_pathA.cost > path.cost || new_pathB.cost > old_cost)
        semicard_conflictsA.push_back(c);
      else
        conflictsA.push_back(c);
      low_level_searches += 2;
      low_level_expanded += (new_pathA.expanded + new_pathB.expanded);
    }
  }

  node.conflicts = conflictsA;
  node.semicard_conflicts = semicard_conflictsA;
  node.cardinal_conflicts = cardinal_conflictsA;
  node.conflicts_num = conflictsA.size() + semicard_conflictsA.size() +
                       cardinal_conflictsA.size();
  return;
}

std::list<Constraint> get_constraints(CBS_Node *node, int agent_id) {
  CBS_Node *curNode = node;
  std::list<Constraint> constraints(0);
  while (curNode->parent != nullptr) {
    if (agent_id < 0 || curNode->constraint.agent == agent_id)
      constraints.push_back(curNode->constraint);
    if (curNode->positive_constraint.agent == agent_id)
      constraints.push_back(curNode->positive_constraint);
    curNode = curNode->parent;
  }
  return constraints;
}

Conflict check_paths(const sPath &pathA, const sPath &pathB) {
  unsigned int a(0), b(0);
  auto nodesA = pathA.nodes;
  auto nodesB = pathB.nodes;
  while (a < nodesA.size() - 1 || b < nodesB.size() - 1) {
    double dist =
        sqrt(pow(map->get_i(nodesA[a].id) - map->get_i(nodesB[b].id), 2) +
             pow(map->get_j(nodesA[a].id) - map->get_j(nodesB[b].id), 2)) -
        CN_EPSILON - 2 * config.agent_size;
    if (a < nodesA.size() - 1 &&
        b < nodesB.size() -
                1)  // if both agents have not reached their goals yet
    {
      if (dist <
          (nodesA[a + 1].g - nodesA[a].g) + (nodesB[b + 1].g - nodesB[b].g))
        if (check_conflict(Move(nodesA[a], nodesA[a + 1]),
                           Move(nodesB[b], nodesB[b + 1])))
          return Conflict(pathA.agentID, pathB.agentID,
                          Move(nodesA[a], nodesA[a + 1]),
                          Move(nodesB[b], nodesB[b + 1]),
                          std::min(nodesA[a].g, nodesB[b].g));
    } else if (a ==
               nodesA.size() - 1)  // if agent A has already reached the goal
    {
      if (dist < (nodesB[b + 1].g - nodesB[b].g))
        if (check_conflict(
                Move(nodesA[a].g, CN_INFINITY, nodesA[a].id, nodesA[a].id),
                Move(nodesB[b], nodesB[b + 1])))
          return Conflict(
              pathA.agentID, pathB.agentID,
              Move(nodesA[a].g, CN_INFINITY, nodesA[a].id, nodesA[a].id),
              Move(nodesB[b], nodesB[b + 1]),
              std::min(nodesA[a].g, nodesB[b].g));
    } else if (b ==
               nodesB.size() - 1)  // if agent B has already reached the goal
    {
      if (dist < (nodesA[a + 1].g - nodesA[a].g))
        if (check_conflict(
                Move(nodesA[a], nodesA[a + 1]),
                Move(nodesB[b].g, CN_INFINITY, nodesB[b].id, nodesB[b].id)))
          return Conflict(
              pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a + 1]),
              Move(nodesB[b].g, CN_INFINITY, nodesB[b].id, nodesB[b].id),
              std::min(nodesA[a].g, nodesB[b].g));
    }
    if (a == nodesA.size() - 1)
      b++;
    else if (b == nodesB.size() - 1)
      a++;
    else if (fabs(nodesA[a + 1].g - nodesB[b + 1].g) < CN_EPSILON) {
      a++;
      b++;
    } else if (nodesA[a + 1].g < nodesB[b + 1].g)
      a++;
    else if (nodesB[b + 1].g - CN_EPSILON < nodesA[a + 1].g)
      b++;
  }
  return Conflict();
}

std::vector<Conflict> get_all_conflicts(const std::vector<sPath> &paths,
                                        int id) {
  std::vector<Conflict> conflicts;
  // check all agents
  if (id < 0)
    for (unsigned int i = 0; i < paths.size(); i++)
      for (unsigned int j = i + 1; j < paths.size(); j++) {
        Conflict conflict = check_paths(paths[i], paths[j]);
        if (conflict.agent1 >= 0) conflicts.push_back(conflict);
      }
  else {
    for (unsigned int i = 0; i < paths.size(); i++) {
      if (int(i) == id) continue;
      Conflict conflict = check_paths(paths[i], paths[id]);
      if (conflict.agent1 >= 0) conflicts.push_back(conflict);
    }
  }
  return conflicts;
}

double get_cost(CBS_Node node, int agent_id) {
  while (node.parent != nullptr) {
    if (node.paths.begin()->agentID == agent_id)
      return node.paths.begin()->cost;
    node = *node.parent;
  }
  return node.paths.at(agent_id).cost;
}

std::vector<sPath> get_paths(CBS_Node *node, unsigned int agents_size) {
  CBS_Node *curNode = node;
  std::vector<sPath> paths(agents_size);
  while (curNode->parent != nullptr) {
    if (paths.at(curNode->paths.begin()->agentID).cost < 0)
      paths.at(curNode->paths.begin()->agentID) = *curNode->paths.begin();
    curNode = curNode->parent;
  }
  for (unsigned int i = 0; i < agents_size; i++)
    if (paths.at(i).cost < 0) paths.at(i) = curNode->paths.at(i);
  return paths;
}

};  // namespace CCBS

void WrappedContinuousCBS2::setInstance(
    ProblemInstancePtr problem_instance_ptr) {
  using namespace CCBS;
  // convert problem_instance to map and task here
  config.agent_size = problem_instance_ptr->getAgentRadius();

  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement *root = doc.NewElement("graphml");
  doc.InsertFirstChild(root);
  tinyxml2::XMLElement *graph = doc.NewElement("graph");
  root->InsertEndChild(graph);
  int n = problem_instance_ptr->getNumberOfVertices();
  for (int i = 0; i < n; i++) {
    double x = problem_instance_ptr->getCoordinateX(i),
           y = problem_instance_ptr->getCoordinateY(i);
    tinyxml2::XMLElement *node = doc.NewElement("node");
    node->SetAttribute("id", ("n" + std::to_string(i)).c_str());
    graph->InsertEndChild(node);
    tinyxml2::XMLElement *data = doc.NewElement("data");
    char buffer[99];
    sprintf(buffer, "%030.15lf,%030.15lf", x, y);
    data->SetText(buffer);
    node->InsertEndChild(data);
  }
  for (int i = 0; i < n; i++) {
    int outdegree = problem_instance_ptr->getOutDegree(i);
    for (int x = 0; x < outdegree; x++) {
      int j = problem_instance_ptr->getTargetVertex(i, x);
      tinyxml2::XMLElement *edge = doc.NewElement("edge");
      edge->SetAttribute("source", ("n" + std::to_string(i)).c_str());
      edge->SetAttribute("target", ("n" + std::to_string(j)).c_str());
      graph->InsertEndChild(edge);
    }
  }
  char temp_file_name[] = "temp_file.xml";
  doc.SaveFile(temp_file_name);

  map.reset(new Map(config.agent_size, config.connectdness));
  map->get_map(temp_file_name);

  doc.Clear();
  root = doc.NewElement("root");
  doc.InsertFirstChild(root);
  for (int i = 0; i < problem_instance_ptr->tasks.size(); i++) {
    auto &task_i = problem_instance_ptr->tasks[i];
    tinyxml2::XMLElement *agent = doc.NewElement("agent");
    agent->SetAttribute("start_id", task_i.initial_vertex);
    agent->SetAttribute("goal_id", task_i.goal_vertex);
    root->InsertEndChild(agent);
  }
  task.reset(new Task());
  doc.SaveFile(temp_file_name);
  task->get_task(temp_file_name);
  task->make_ij(*map);

  remove(temp_file_name);

  prob = problem_instance_ptr;
}

void WrappedContinuousCBS2::solve() {
  using namespace CCBS;
  tree = CBS_Tree();
  planner = SIPP();
  h_values = Heuristic();
  solution = Solution();
  solution = find_solution();
}
bool WrappedContinuousCBS2::succeeded() { return CCBS::solution.found; }

ProblemInstance::Plan WrappedContinuousCBS2::getPlan() {
  using namespace CCBS;
  // convert solution to plan
  ProblemInstance::Plan plan(solution.paths.size());
  for (int i = 0; i < solution.paths.size(); i++) {
    auto &path = solution.paths[i];
    int agent = path.agentID;
    for (int t = 0; t < path.nodes.size() - 1; t++) {
      ProblemInstance::Move move;
      int source_vertex = path.nodes[t].id;
      int target_vertex = path.nodes[t + 1].id;
      if (source_vertex == target_vertex) {
        continue;
      }
      for (int x = 0; x < prob->getOutDegree(source_vertex); x++) {
        if (prob->getTargetVertex(source_vertex, x) == target_vertex) {
          move.edge_id = x;
        }
      }
      move.start_time = path.nodes[t].g;
      double distance = prob->getEdgeLength(source_vertex, move.edge_id);
      if (abs((path.nodes[t + 1].g - path.nodes[t].g) - distance) > 1e-9) {
        throw std::runtime_error("CCBS solution is invalid");
      }
      plan[agent].push_back(move);
    }
  }
  return plan;
}

WrappedContinuousCBS2::WrappedContinuousCBS2(const double time_limit) {
  using namespace CCBS;
  // setting config
  config.use_cardinal = true;
  config.use_disjoint_splitting = true;
  config.hlh_type = 2;
  config.connectdness = 3;
  config.focal_weight = 1.0;
  config.timelimit = time_limit;
  config.precision = 0.0000001;
}

WrappedContinuousCBS2::~WrappedContinuousCBS2() {
  using namespace CCBS;
  task.release();
  map.release();
}
