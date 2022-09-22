/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#ifndef MAPF_BENCHMARK_PROBLEM_WITH_GRID
#define MAPF_BENCHMARK_PROBLEM_WITH_GRID

#include "base/geometry_2D.hpp"
#include "base/problem.hpp"

class ProblemInstanceWithGrid : public ProblemInstanceWithGeometry<Geometry2D> {
 private:
  struct Vector2 {
    int i, j;
    Vector2() : i(0), j(0) {}
    Vector2(const int _i, const int _j) : i(_i), j(_j) {}
    Vector2 operator+(const Vector2 add) const {
      return Vector2(i + add.i, j + add.j);
    }
    Vector2 operator-(const Vector2 add) const {
      return Vector2(i - add.i, j - add.j);
    }
    Vector2 operator-() const { return Vector2(-i, -j); }
    int norm() { return i * i + j * j; }
  };
  int K;
  std::vector<Vector2> neighbors;
  std::vector<double> neighbor_length;

  double radius;
  std::vector<Vector2> VV_collisions;
  std::vector<std::vector<std::tuple<Vector2, double, double>>> EV_collisions;
  std::vector<std::tuple<Vector2, int, double, double>> VE_collisions;
  std::vector<std::vector<std::tuple<Vector2, int, double, double>>>
      EE_collisions;

  static Geometry2D::Point convertToPoint(const Vector2& point) {
    return Geometry2D::Point(point.i, point.j);
  }

  void addEVCollision(const int k, const Vector2& point) {
    double lower, upper;
    Geometry2D::getCollisionInterval(
        convertToPoint(point), Geometry2D::Point(0., 0.),
        convertToPoint(neighbors[k]), neighbor_length[k], agent_geometry, lower,
        upper);
    EV_collisions[k].push_back(std::make_tuple(point, lower, upper));
  }
  void addEECollision(const int k, const Vector2& point, const int l) {
    double lower, upper;
    Geometry2D::getCollisionInterval(
        Geometry2D::Point(0., 0.), convertToPoint(neighbors[k]),
        neighbor_length[k], convertToPoint(point), convertToPoint(neighbors[l]),
        neighbor_length[l], agent_geometry, lower, upper);
    EE_collisions[k].push_back(std::make_tuple(point, l, lower, upper));
  }

  void precalculateCollisions() {
    EV_collisions.resize(1 << K);
    EE_collisions.resize(1 << K);
    for (int di = -std::floor(2 * radius); di <= std::floor(2 * radius); di++) {
      int abs_j = std::floor(std::sqrt(4 * radius * radius - (double)di * di));
      for (int dj = -abs_j; dj <= abs_j; dj++) {
        VV_collisions.push_back(Vector2(di, dj));
      }
    }
    for (int k = 0; k < 1 << (K - 2); k++) {
      for (auto point : VV_collisions) {
        addEVCollision(k, point);
        for (int l = 0; l < 1 << K; l++) {
          addEECollision(k, point, l);
          int rl = l ^ (1 << (K - 1));
          if ((point + neighbors[l]).norm() >= 4 * radius * radius) {
            addEECollision(k, point + neighbors[l], rl);
          }
        }
        if ((neighbors[k] + point).norm() >= 4 * radius * radius) {
          addEVCollision(k, neighbors[k] + point);
          for (int l = 0; l < 1 << K; l++) {
            if ((neighbors[k] + point + neighbors[l]).norm() >=
                4 * radius * radius) {
              addEECollision(k, neighbors[k] + point, l);
              int rl = l ^ (1 << (K - 1));
              if ((point + neighbors[l]).norm() >= 4 * radius * radius) {
                addEECollision(k, neighbors[k] + point + neighbors[l], rl);
              }
            }
          }
        }
      }
      Geometry2D::Point vec = convertToPoint(neighbors[k]),
                        u = radius / std::sqrt(Geometry2D::inner(vec, vec)) *
                            Point(-vec.y, vec.x);
      int L = std::ceil(u.x), R = std::floor(vec.x - u.x);
      for (int i = L; i <= R; i++) {
        int lower_j, upper_j;
        if (i < -u.x - EPS) {
          lower_j = std::ceil(u.y / u.x * i);
        } else {
          lower_j = std::ceil(-u.y + vec.y / vec.x * (i + u.x));
        }
        if (i <= vec.x + u.x + EPS) {
          upper_j = std::floor(u.y + vec.y / vec.x * (i - u.x));
        } else {
          upper_j = std::floor(vec.y + u.y / u.x * (i - vec.x));
        }
        for (int j = lower_j; j <= upper_j; j++) {
          Vector2 point(i, j);
          if (point.norm() >= 4 * radius * radius &&
              (point - neighbors[k]).norm() >= 4 * radius * radius) {
            addEVCollision(k, point);
          }
        }
      }
      for (int l = 0; l < 1 << K; l++) {
        Vector2 B, T, L, R;
        if (l <= k) {
          B = -neighbors[l];
          L = neighbors[k] - neighbors[l];
          T = neighbors[k];
        } else if (l < 1 << (K - 1)) {
          B = -neighbors[l];
          R = neighbors[k] - neighbors[l];
          T = neighbors[k];
        } else if (l <= k + (1 << (K - 1))) {
          L = neighbors[k];
          R = -neighbors[l];
          T = neighbors[k] - neighbors[l];
        } else {
          R = neighbors[k];
          L = -neighbors[l];
          T = neighbors[k] - neighbors[l];
        }
        for (int j = B.j; j <= T.j; j++) {
          int left_i, right_i;
          if (j < L.j) {
            left_i = B.i + std::div((L.i - B.i) * (j - B.j) + L.j - B.j - 1,
                                    L.j - B.j)
                               .quot;
          } else if (j == L.j) {
            left_i = L.i;
          } else {
            left_i = L.i + std::div((T.i - L.i) * (j - L.j) + T.j - L.j - 1,
                                    T.j - L.j)
                               .quot;
          }
          if (j < R.j) {
            right_i = B.i + std::div((R.i - B.i) * (j - B.j), R.j - B.j).quot;
          } else if (j == R.j) {
            right_i = R.i;
          } else {
            right_i = R.i + std::div((T.i - R.i) * (j - R.j), T.j - R.j).quot;
          }
          for (int i = left_i; i <= right_i; i++) {
            Vector2 point(i, j);
            if (point.norm() >= 4 * radius * radius &&
                (point - neighbors[k]).norm() >= 4 * radius * radius &&
                (point + neighbors[l]).norm() >= 4 * radius * radius &&
                (point + neighbors[l] - neighbors[k]).norm() <=
                    4 * radius * radius) {
              addEECollision(k, point, l);
            }
          }
        }
      }
      for (auto& collision : EV_collisions[k]) {
        Vector2& point = std::get<0>(collision);
        double lower = std::get<1>(collision), upper = std::get<2>(collision);
        EV_collisions[k + (1 << (K - 2))].push_back(
            std::make_tuple(Vector2(-point.j, point.i), lower, upper));
        EV_collisions[k + (1 << (K - 1))].push_back(
            std::make_tuple(Vector2(-point.i, -point.j), lower, upper));
        EV_collisions[k + (3 << (K - 2))].push_back(
            std::make_tuple(Vector2(point.j, -point.i), lower, upper));
      }
      for (auto& collision : EE_collisions[k]) {
        Vector2& point = std::get<0>(collision);
        int l = std::get<1>(collision);
        double lower = std::get<2>(collision), upper = std::get<3>(collision);
        EE_collisions[k + (1 << (K - 2))].push_back(
            std::make_tuple(Vector2(-point.j, point.i),
                            (l + (1 << (K - 2))) % (1 << K), lower, upper));
        EE_collisions[k + (1 << (K - 1))].push_back(
            std::make_tuple(Vector2(-point.i, -point.j),
                            (l + (1 << (K - 1))) % (1 << K), lower, upper));
        EE_collisions[k + (3 << (K - 2))].push_back(
            std::make_tuple(Vector2(point.j, -point.i),
                            (l + (3 << (K - 2))) % (1 << K), lower, upper));
      }
    }
    for (int k = 0; k < 1 << K; k++) {
      for (auto& collision : EV_collisions[k]) {
        Vector2& point = std::get<0>(collision);
        double lower = std::get<1>(collision), upper = std::get<2>(collision);
        VE_collisions.push_back(std::make_tuple(-point, k, -upper, -lower));
      }
    }
  }

  std::vector<std::vector<bool>> grid;
  int height, width;
  std::vector<Vector2> vertices;
  std::vector<std::vector<int>> vertex_ids;
  std::vector<std::vector<int>> edge_ids, edge_directions;

  // TODO, current codes are temporal
  bool isCellInside(const Vector2& cell) const { return grid[cell.i][cell.j]; }

  int getID(const Vector2& point) const {
    return point.i < 0 || point.i >= height || point.j < 0 || point.j >= width
               ? -1
               : vertex_ids[point.i][point.j];
  }

  bool isEdgeInside(const Vector2& cell, const int k) const {
    return getID(cell) != -1 && getID(cell + neighbors[k]) != -1;
  }

 public:
  ProblemInstanceWithGrid(
      const int K, const std::vector<std::vector<bool>>& grid,
      const double agent_geometry,
      const std::vector<std::tuple<int, int, int, int>>& task_tuples) {
    this->K = K;
    neighbors = std::vector<Vector2>{Vector2(1, 0), Vector2(0, 1),
                                     Vector2(-1, 0), Vector2(0, -1)};
    for (int k = 3; k <= K; k++) {
      std::vector<Vector2> new_neighbors;
      for (int i = 0; i < neighbors.size(); i++) {
        new_neighbors.push_back(neighbors[i]);
        int j = (i + 1) % neighbors.size();
        new_neighbors.push_back(neighbors[i] + neighbors[j]);
      }
      neighbors = new_neighbors;
    }
    neighbor_length.resize(1 << K);
    for (int k = 0; k < 1 << K; k++) {
      neighbor_length[k] = std::sqrt((double)neighbors[k].norm());
    }
    this->grid = grid;
    height = grid.size();
    width = grid[0].size();
    this->agent_geometry = agent_geometry;
    radius = agent_geometry - EPS;
    precalculateCollisions();
    vertex_ids.resize(height);
    for (int i = 0; i < height; i++) {
      vertex_ids[i].resize(width);
      for (int j = 0; j < width; j++) {
        if (isCellInside(Vector2(i, j))) {
          vertex_ids[i][j] = vertices.size();
          vertices.push_back(Vector2(i, j));
          coordinates.push_back(Geometry2D::Point(j, i));
        } else {
          vertex_ids[i][j] = -1;
        }
      }
    }
    edges.resize(vertices.size());
    edge_ids.resize(vertices.size());
    edge_directions.resize(vertices.size());
    for (int vertex = 0; vertex < vertices.size(); vertex++) {
      edge_ids[vertex].resize(1 << K);
      int outdegree = 0;
      for (int k = 0; k < 1 << K; k++) {
        if (isEdgeInside(vertices[vertex], k)) {
          auto target = vertices[vertex] + neighbors[k];
          edge_ids[vertex][k] = edges[vertex].size();
          edges[vertex].push_back(vertex_ids[target.i][target.j]);
          edge_directions[vertex].push_back(k);
        } else {
          edge_ids[vertex][k] = -1;
        }
      }
    }
    tasks.resize(task_tuples.size());
    for (int agent = 0; agent < task_tuples.size(); agent++) {
      auto& tuple = task_tuples[agent];
      tasks[agent].initial_vertex =
          vertex_ids[std::get<1>(tuple) + 1][std::get<0>(tuple) + 1];
      tasks[agent].goal_vertex =
          vertex_ids[std::get<3>(tuple) + 1][std::get<2>(tuple) + 1];
    }
  }

  std::vector<int> all_vertices;
  std::vector<std::tuple<int, double, double>> all_collision_vertices;
  std::vector<std::tuple<int, int, double, double>> all_collision_edges;

  std::vector<int>& getAllVVCollisions(const int vertex) override {
    Vector2& point = vertices[vertex];
    all_vertices.clear();
    for (auto& collision_vector : VV_collisions) {
      int id = getID(point + collision_vector);
      if (id != -1) {
        all_vertices.push_back(id);
      }
    }
    return all_vertices;
  }

  std::vector<std::tuple<int, int, double, double>>& getAllVECollisions(
      const int vertex) override {
    Vector2& point = vertices[vertex];
    all_collision_edges.clear();
    for (auto& collision : VE_collisions) {
      Vector2 target = point + std::get<0>(collision);
      int target_id = getID(target);
      if (target_id == -1) continue;
      int k = std::get<1>(collision);
      int edge_id = edge_ids[target_id][k];
      if (edge_id != -1) {
        all_collision_edges.push_back(std::make_tuple(target_id, edge_id,
                                                      std::get<2>(collision),
                                                      std::get<3>(collision)));
      }
    }
    return all_collision_edges;
  }
  std::vector<std::tuple<int, double, double>>& getAllEVCollisions(
      const int vertex, const int edge_id) override {
    Vector2& point = vertices[vertex];
    int k = edge_directions[vertex][edge_id];
    all_collision_vertices.clear();
    for (auto& collision : EV_collisions[k]) {
      Vector2 target = point + std::get<0>(collision);
      int target_id = getID(target);
      if (target_id != -1) {
        all_collision_vertices.push_back(std::make_tuple(
            target_id, std::get<1>(collision), std::get<2>(collision)));
      }
    }
    return all_collision_vertices;
  }

  std::vector<std::tuple<int, int, double, double>>& getAllEECollisions(
      const int vertex, const int edge_id) override {
    Vector2& point = vertices[vertex];
    int k = edge_directions[vertex][edge_id];
    for (auto& collision : EE_collisions[k]) {
      Vector2 target = point + std::get<0>(collision);
      int target_id = getID(target);
      if (target_id == -1) continue;
      int l = std::get<1>(collision);
      int edge_id = edge_ids[target_id][l];
      if (edge_id != -1) {
        all_collision_edges.push_back(std::make_tuple(target_id, edge_id,
                                                      std::get<2>(collision),
                                                      std::get<3>(collision)));
      }
    }
    return all_collision_edges;
  }
};

using ProblemInstanceWithGridPtr = std::shared_ptr<ProblemInstanceWithGrid>;
#endif
