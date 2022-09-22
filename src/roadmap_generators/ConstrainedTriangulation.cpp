/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include "roadmap_generators/ConstrainedTriangulation.hpp"

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

using K = CGAL::Exact_predicates_exact_constructions_kernel;
using CDT = CGAL::Constrained_Delaunay_triangulation_2<K>;

CDT::Point convertToCGALPoint(const Space2D::Point& point) {
  return CDT::Point(point.x, point.y);
}

Space2D::Point convertFromCGALPoint(const CDT::Point& point) {
  return Space2D::Point(CGAL::to_double(point.x()), CGAL::to_double(point.y()));
}

void ConstrainedTriangulation::generate(
    const SpatialProblemInstance<Space2D>& spatial_problem_instance) {
  auto& space = spatial_problem_instance.space;

  CDT cdt;
  auto& tasks = spatial_problem_instance.tasks;
  for (int agent = 0; agent < tasks.size(); agent++) {
    cdt.insert(convertToCGALPoint(tasks[agent].initial_point));
    cdt.insert(convertToCGALPoint(tasks[agent].goal_point));
  }
  double agent_geometry = spatial_problem_instance.agent_geometry;
  auto additional_points =
      Space2D::getRandomPoints(space, agent_geometry, number_of_additionals);
  for (auto& point : additional_points) {
    cdt.insert(convertToCGALPoint(point));
  }
  for (int t = 0; t < space.inners().size() + 1; t++) {
    Space2D::Space::ring_type ring;
    if (t == 0)
      ring = space.outer();
    else
      ring = space.inners()[t - 1];
    int n = ring.size() - 1;
    std::vector<Space2D::Point> border_0(n), border_1(n);
    for (int i = 0; i < n; i++) {
      Space2D::Point vec0 = ring[i == 0 ? n - 1 : i - 1] - ring[i],
                     u0 = 1. / Geometry2D::length(vec0) * vec0;
      Space2D::Point vec1 = ring[i + 1] - ring[i],
                     u1 = 1. / Geometry2D::length(vec1) * vec1;
      double ccw = u0.x * u1.y - u0.y * u1.x;
      if (ccw > 0) {
        double ip = Geometry2D::inner(u0, u1);
        border_0[i] = border_1[i] =
            ring[i] + agent_geometry * (Space2D::Point(-u0.y, u0.x) +
                                        std::sqrt((1. + ip) / (1. - ip)) * u0);
      } else {
        border_0[i] = ring[i] + agent_geometry * Space2D::Point(-u0.y, u0.x);
        border_1[i] = ring[i] + agent_geometry * Space2D::Point(u1.y, -u1.x);
        if (use_corner_points) {
          double ip = Geometry2D::inner(u0, u1);
          Space2D::Point corner =
              border_0[i] -
              agent_geometry * std::sqrt((1. + ip) / (1. - ip)) * u0;
          if (spatial_problem_instance.isInside(corner)) {
            cdt.insert(convertToCGALPoint(corner));
          }
        }
      }
    }
    for (int i = 0; i < n; i++) {
      int j = (i + 1) % n;
      auto& source = border_1[i];
      auto& target = border_0[j];
      if (Geometry2D::inner(target - source, ring[j] - ring[i]) > 0 &&
          spatial_problem_instance.isInside(source) &&
          spatial_problem_instance.isInside(target) &&
          spatial_problem_instance.isInside(source, target)) {
        cdt.insert_constraint(convertToCGALPoint(source),
                              convertToCGALPoint(target));
      }
    }
  }
  problem_instance.coordinates.clear();
  std::map<std::pair<long long, long long>, int> vertex_ids;
  double EPS = spatial_problem_instance.EPS;
  auto discretize = [EPS](const Space2D::Point& point) {
    return std::make_pair((long long)(point.x / EPS),
                          (long long)(point.y / EPS));
  };
  for (CDT::Vertex_handle vh : cdt.finite_vertex_handles()) {
    auto point = convertFromCGALPoint(vh->point());
    vertex_ids[discretize(point)] = problem_instance.coordinates.size();
    problem_instance.coordinates.push_back(point);
  }
  problem_instance.agent_geometry = agent_geometry;
  problem_instance.edges.clear();
  problem_instance.edges.resize(problem_instance.coordinates.size());
  for (CDT::Edge e : cdt.finite_edges()) {
    auto segment = cdt.segment(e);
    auto p0 = convertFromCGALPoint(segment.source()),
         p1 = convertFromCGALPoint(segment.target());
    if (spatial_problem_instance.isInside(p0, p1)) {
      int id_0 = vertex_ids[discretize(p0)], id_1 = vertex_ids[discretize(p1)];
      problem_instance.edges[id_0].push_back(id_1);
      problem_instance.edges[id_1].push_back(id_0);
    }
  }
  problem_instance.tasks.resize(tasks.size());
  for (int agent = 0; agent < tasks.size(); agent++) {
    problem_instance.tasks[agent].initial_vertex = vertex_ids[discretize(
        spatial_problem_instance.tasks[agent].initial_point)];
    problem_instance.tasks[agent].goal_vertex = vertex_ids[discretize(
        spatial_problem_instance.tasks[agent].goal_point)];
  }
}
