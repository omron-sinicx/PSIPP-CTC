/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include "base/input.hpp"
#include "base/problem_with_explicit_collisions.hpp"

int main() {
  std::random_device rd;
  std::default_random_engine eng(rd());
  std::uniform_real_distribution<double> distr(0., 10.);
  using ProblemInstance2D = ProblemInstanceWithGeometry<Geometry2D>;
  ProblemInstance2D problem_0("../problem_instances/roadmaps/generated.txt");
  ProblemInstanceWithExplicitCollisions problem_1(problem_0);
  int number_of_vertices = problem_0.getNumberOfVertices();
  const double EPS = 1e-6;
  for (int vertex_0 = 0; vertex_0 < number_of_vertices; vertex_0++) {
    for (int vertex_1 = 0; vertex_1 < number_of_vertices; vertex_1++) {
      bool collision_0 = problem_0.CollisionCheck(vertex_0, vertex_1);
      bool collision_1 = problem_1.CollisionCheck(vertex_0, vertex_1);
      assert(collision_0 == collision_1);
      for (int edge_1 = 0; edge_1 < problem_0.getOutDegree(vertex_1);
           edge_1++) {
        double lower_0, upper_0, lower_1, upper_1;
        bool collision_0 = problem_0.getCollisionInterval(
            vertex_0, vertex_1, edge_1, lower_0, upper_0);
        bool collision_1 = problem_1.getCollisionInterval(
            vertex_0, vertex_1, edge_1, lower_1, upper_1);
        assert(collision_0 == collision_1);
        if (collision_0) {
          assert(abs(lower_0 - lower_1) < EPS);
          assert(abs(upper_0 - upper_1) < EPS);
        }
        for (int t = 0; t < 3; t++) {
          double start_time_0 = distr(eng), end_time_0 = distr(eng);
          double start_time_1 = distr(eng);
          bool collision_0 =
              problem_0.CollisionCheck(vertex_0, start_time_0, end_time_0,
                                       vertex_1, start_time_1, edge_1);
          bool collision_1 =
              problem_1.CollisionCheck(vertex_0, start_time_0, end_time_0,
                                       vertex_1, start_time_1, edge_1);
          assert(collision_0 == collision_1);
        }
        for (int edge_0 = 0; edge_0 < problem_0.getOutDegree(vertex_0);
             edge_0++) {
          double lower_0, upper_0, lower_1, upper_1;
          bool collision_0 = problem_0.getCollisionInterval(
              vertex_0, edge_0, vertex_1, edge_1, lower_0, upper_0);
          bool collision_1 = problem_1.getCollisionInterval(
              vertex_0, edge_0, vertex_1, edge_1, lower_1, upper_1);
          assert(collision_0 == collision_1);
          if (collision_0) {
            assert(abs(lower_0 - lower_1) < EPS);
            assert(abs(upper_0 - upper_1) < EPS);
          }
          for (int t = 0; t < 3; t++) {
            double start_time_0 = distr(eng);
            double start_time_1 = distr(eng);
            bool collision_0 = problem_0.CollisionCheck(
                vertex_0, start_time_0, edge_0, vertex_1, start_time_1, edge_1);
            bool collision_1 = problem_1.CollisionCheck(
                vertex_0, start_time_0, edge_0, vertex_1, start_time_1, edge_1);
            assert(collision_0 == collision_1);
          }
        }
      }
    }
  }
  return 0;
}
