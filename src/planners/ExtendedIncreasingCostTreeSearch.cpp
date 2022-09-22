/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include <queue>
#include <set>

#include "base/planner.hpp"

class ExtendedIncreasingCostTreeSearch : public Planner {
  ProblemInstancePtr prob;

  double delta, Delta;
  double time_limit;

 public:
  ExtendedIncreasingCostTreeSearch(const double Delta, const double delta,
                                   const double time_limit) {
    this->Delta = Delta;
    this->delta = delta;
    this->time_limit = time_limit;
  }

  void setInstance(ProblemInstancePtr problem_instance_ptr) override {
    cost_to_goal.clear();
    best_cost.clear();
    this->prob = problem_instance_ptr;
  }

  int number_of_agents;
  std::vector<std::vector<double>> cost_to_goal;
  std::vector<double> best_cost;
  bool success;

  class MDD {
   public:
    std::vector<std::vector<std::pair<int, int>>>
        edges;  // pair of target and graph edge
    std::vector<std::pair<int, double>>
        vertices;  // pair of graph vertex and time

    int agent;
    double redundance;
    MDD() {}
  };

  double bounded_best(const MDD& mdd) {
    double best = prob->INF;
    for (int vertex = 2; vertex < mdd.vertices.size(); vertex++) {
      if (mdd.vertices[vertex].first == mdd.vertices[1].first &&
          best > mdd.vertices[vertex].second &&
          mdd.vertices[vertex].second >
              best_cost[mdd.agent] + mdd.redundance - delta) {
        best = mdd.vertices[vertex].second;
      }
    }
    return best;
  }

  MDD make_MDD(int agent, double redundance) {
    MDD mdd;
    mdd.agent = agent;
    mdd.redundance = redundance;
    int initial_vertex = prob->getAgentInitialVertex(agent);
    int goal_vertex = prob->getAgentGoalVertex(agent);
    mdd.vertices.push_back(std::make_pair(initial_vertex, -prob->INF));
    mdd.edges.push_back(
        std::vector<std::pair<int, int>>(1, std::make_pair(2, -1)));
    mdd.vertices.push_back(std::make_pair(goal_vertex, prob->INF));
    mdd.edges.push_back(std::vector<std::pair<int, int>>(0));
    mdd.vertices.push_back(std::make_pair(initial_vertex, 0));
    for (int i = 2; i < mdd.vertices.size(); i++) {
      std::vector<std::pair<int, int>> edge_list;
      int vertex = mdd.vertices[i].first;
      double time = mdd.vertices[i].second;
      if (time + Delta + cost_to_goal[agent][vertex] <=
          best_cost[agent] + redundance) {
        int id = mdd.vertices.size();
        edge_list.push_back(std::make_pair(id, -1));
        mdd.vertices.push_back(std::make_pair(vertex, time + Delta));
      }
      int outdegree = prob->getOutDegree(vertex);
      for (int edge = 0; edge < outdegree; edge++) {
        int target_vertex = prob->getTargetVertex(vertex, edge);
        if (vertex == target_vertex) continue;
        double length = prob->getEdgeLength(vertex, edge);
        if (time + length + cost_to_goal[agent][target_vertex] <
            best_cost[agent] + redundance) {
          int id = mdd.vertices.size();
          edge_list.push_back(std::make_pair(id, edge));
          mdd.vertices.push_back(std::make_pair(target_vertex, time + length));
        }
      }
      if (vertex == goal_vertex) {
        edge_list.push_back(std::make_pair(1, -1));
      }
      mdd.edges.push_back(edge_list);
    }
    return mdd;
  }

  struct ICTNode {
    std::vector<int> redundances;
    bool operator<(const ICTNode& node_0) const {
      for (int agent = 0; agent < this->redundances.size(); agent++) {
        if (this->redundances[agent] != node_0.redundances[agent]) {
          return this->redundances[agent] < node_0.redundances[agent];
        }
      }
      return false;
    }
  };

  struct JointState {
    std::vector<std::pair<int, int>> states;
    bool operator<(const JointState& node_0) const {
      for (int agent = 0; agent < this->states.size(); agent++) {
        if (this->states[agent] != node_0.states[agent]) {
          return this->states[agent] < node_0.states[agent];
        }
      }
      return false;
    }
  };

  int number_of_vertices;
  std::vector<std::vector<std::pair<int, double>>> reverse_edges;

  void construct_reverse_graph() {
    number_of_vertices = prob->getNumberOfVertices();
    reverse_edges.clear();
    reverse_edges.resize(number_of_vertices);
    for (int vertex = 0; vertex < number_of_vertices; vertex++) {
      int outdegree = prob->getOutDegree(vertex);
      for (int edge = 0; edge < outdegree; edge++) {
        int target_vertex = prob->getTargetVertex(vertex, edge);
        double length = prob->getEdgeLength(vertex, edge);
        reverse_edges[target_vertex].push_back(std::make_pair(vertex, length));
      }
    }
  }

  std::vector<double> calculate_distance(const int goal) {
    std::vector<double> distances(number_of_vertices, prob->INF);
    std::vector<bool> visited(number_of_vertices, false);
    using Pair = std::pair<double, int>;
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> queue;
    queue.push(std::make_pair(0.0, goal));
    distances[goal] = 0.0;
    while (!queue.empty()) {
      double vertex = queue.top().second;
      queue.pop();
      if (visited[vertex]) continue;
      visited[vertex] = true;
      for (int edge = 0; edge < reverse_edges[vertex].size(); edge++) {
        int target_vertex = reverse_edges[vertex][edge].first;
        double length = reverse_edges[vertex][edge].second;
        if (!visited[target_vertex] &&
            distances[target_vertex] > distances[vertex] + length) {
          distances[target_vertex] = distances[vertex] + length;
          queue.push(std::make_pair(distances[target_vertex], target_vertex));
        }
      }
    }
    return distances;
  }

  ProblemInstance::Plan plan;

  void solve() override {
    auto start_time = std::chrono::system_clock::now();

    number_of_agents = prob->getNumberOfAgents();
    cost_to_goal.resize(number_of_agents);
    best_cost.resize(number_of_agents);
    construct_reverse_graph();
    for (int agent = 0; agent < number_of_agents; agent++) {
      cost_to_goal[agent] = calculate_distance(prob->getAgentGoalVertex(agent));
      best_cost[agent] =
          cost_to_goal[agent][prob->getAgentInitialVertex(agent)];
    }
    success = false;
    std::vector<ICTNode> ICT_nodes(1);
    ICT_nodes[0].redundances = std::vector<int>(number_of_agents, 0);
    std::set<ICTNode> all_ICT_nodes;
    all_ICT_nodes.insert(ICT_nodes[0]);
    using Pair = std::pair<double, int>;
    using PriorityQueue =
        std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>>;
    PriorityQueue ICTQueue;
    double sum_of_bests = 0.0;

    std::vector<std::vector<MDD>> MDDs(number_of_agents);
    for (int agent = 0; agent < number_of_agents; agent++) {
      MDDs[agent].push_back(make_MDD(agent, prob->EPS));
      sum_of_bests += bounded_best(MDDs[agent][0]);
    }

    ICTQueue.push(std::make_pair(sum_of_bests, 0));
    double incumbent = prob->INF;

    while (!ICTQueue.empty()) {
      int ICT_node_id = ICTQueue.top().second;
      double sum_of_bests = ICTQueue.top().first;
      ICTQueue.pop();
      if (sum_of_bests + prob->EPS >= incumbent) {
        break;
      }
      auto current_time = std::chrono::system_clock::now();
      double used_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                             current_time - start_time)
                             .count();
      if (used_time > 1000. * time_limit) {
        break;
      }
      auto redundances = ICT_nodes[ICT_node_id].redundances;
      std::vector<JointState> joint_states;
      std::set<JointState> all_joint_states;
      JointState initial_state;
      initial_state.states = std::vector<std::pair<int, int>>(
          number_of_agents, std::make_pair(0, 0));
      joint_states.push_back(initial_state);
      all_joint_states.insert(initial_state);
      std::vector<int> previous(1, -1), moved_agent(1, -1);
      PriorityQueue Q;
      Q.push(std::make_pair(0.0, 0));

      int goal = -1;
      double goal_cost;

      while (!Q.empty()) {
        int id = Q.top().second;
        double cost = Q.top().first;
        Q.pop();

        bool finished = true;

        auto states = joint_states[id].states;
        int move_agent = -1, start_vertex = -1;
        double start_time = prob->INF;
        for (int agent = 0; agent < number_of_agents; agent++) {
          auto& MDD = MDDs[agent][redundances[agent]];
          int target_vertex =
              MDD.edges[states[agent].first][states[agent].second].first;
          if (target_vertex != 1) {
            finished = false;
            if (MDD.vertices[target_vertex].second < start_time) {
              start_time = MDD.vertices[target_vertex].second;
              move_agent = agent;
              start_vertex = target_vertex;
            }
          }
        }
        if (finished) {
          goal = id;
          goal_cost = cost;
          break;
        }
        auto& MDD = MDDs[move_agent][redundances[move_agent]];
        for (int MDD_edge = 0; MDD_edge < MDD.edges[start_vertex].size();
             MDD_edge++) {
          bool collision;
          int target_vertex = MDD.edges[start_vertex][MDD_edge].first;
          int moving_edge = MDD.edges[start_vertex][MDD_edge].second;
          if (moving_edge == -1) {
            for (int agent = 0; agent < number_of_agents; agent++) {
              if (agent == move_agent) continue;
              auto& agent_MDD = MDDs[agent][redundances[agent]];
              auto& agent_current_vertex =
                  agent_MDD.vertices[states[agent].first];
              auto& agent_current_edge =
                  agent_MDD.edges[states[agent].first][states[agent].second];
              if (agent_current_edge.second == -1) {
                collision =
                    prob->CollisionCheck(MDD.vertices[start_vertex].first,
                                         agent_current_vertex.first);
              } else {
                collision = prob->CollisionCheck(
                    MDD.vertices[start_vertex].first, start_time,
                    start_time + Delta, agent_current_vertex.first,
                    agent_current_vertex.second, agent_current_edge.second);
              }
            }
          } else {
            for (int agent = 0; agent < number_of_agents; agent++) {
              if (agent == move_agent) continue;
              auto& agent_MDD = MDDs[agent][redundances[agent]];
              auto& agent_current_vertex =
                  agent_MDD.vertices[states[agent].first];
              auto& agent_current_edge =
                  agent_MDD.edges[states[agent].first][states[agent].second];
              if (agent_current_edge.second == -1) {
                collision = prob->CollisionCheck(
                    agent_current_vertex.first, agent_current_vertex.second,
                    agent_current_vertex.second + Delta,
                    MDD.vertices[start_vertex].first, start_time, moving_edge);
              } else {
                collision = prob->CollisionCheck(
                    MDD.vertices[start_vertex].first, start_time, moving_edge,
                    agent_current_vertex.first, agent_current_vertex.second,
                    agent_current_edge.second);
              }
            }
          }
          if (!collision) {
            JointState new_state;
            new_state.states = states;
            new_state.states[move_agent] =
                std::make_pair(start_vertex, MDD_edge);
            if (all_joint_states.find(new_state) != all_joint_states.end()) {
              continue;
            }
            all_joint_states.insert(new_state);
            int new_id = joint_states.size();
            joint_states.push_back(new_state);
            previous.push_back(id);
            moved_agent.push_back(move_agent);
            double new_cost =
                cost + (target_vertex == 1
                            ? 0.
                            : MDD.vertices[target_vertex].second -
                                  MDD.vertices[start_vertex].second);
            Q.push(std::make_pair(new_cost, new_id));
          }
        }
      }
      if (goal != -1) {
        success = true;
        incumbent = goal_cost;
        // convert to plan
        int joint_state_id = goal;
        plan.clear();
        plan.resize(number_of_agents);
        while (joint_state_id != 0) {
          int agent = moved_agent[joint_state_id];
          joint_state_id = previous[joint_state_id];
          auto& move = joint_states[joint_state_id].states[agent];
          auto& MDD = MDDs[agent][redundances[agent]];
          int edge = MDD.edges[move.first][move.second].second;
          if (edge != -1) {
            double start_time = MDD.vertices[move.first].second;
            plan[agent].push_back(ProblemInstance::Move(edge, start_time));
          }
        }
        for (int agent = 0; agent < number_of_agents; agent++) {
          std::reverse(plan[agent].begin(), plan[agent].end());
        }
        if (sum_of_bests + prob->EPS >= incumbent) {
          break;
        }
      } else {
        for (int agent = 0; agent < number_of_agents; agent++) {
          if (MDDs[agent].size() <= redundances[agent] + 1) {
            MDDs[agent].push_back(
                make_MDD(agent, (redundances[agent] + 1) * delta));
          }
          ICTNode new_ICT_node;
          new_ICT_node.redundances = redundances;
          new_ICT_node.redundances[agent]++;
          if (all_ICT_nodes.find(new_ICT_node) != all_ICT_nodes.end()) {
            continue;
          }
          all_ICT_nodes.insert(new_ICT_node);
          int ICT_id = ICT_nodes.size();
          ICT_nodes.push_back(new_ICT_node);
          double new_sum_of_best =
              sum_of_bests - bounded_best(MDDs[agent][redundances[agent]]) +
              bounded_best(MDDs[agent][redundances[agent] + 1]);
          ICTQueue.push(std::make_pair(new_sum_of_best, ICT_id));
        }
      }
    }
  }
  bool succeeded() override { return success; }
  ProblemInstance::Plan getPlan() { return plan; }
};
