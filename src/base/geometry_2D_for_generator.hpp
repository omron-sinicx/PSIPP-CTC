/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#ifndef MAPF_BENCHMARK_GEOMETRY_FOR_GENERATOR
#define MAPF_BENCHMARK_GEOMETRY_FOR_GENERATOR

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include "base/geometry_2D.hpp"

namespace boost::geometry::traits {
template <>
struct dimension<Geometry2D::Point> : boost::mpl::int_<2> {};
template <>
struct coordinate_system<Geometry2D::Point> {
  typedef cs::cartesian type;
};
template <>
struct coordinate_type<Geometry2D::Point> {
  typedef double type;
};
template <>
struct tag<Geometry2D::Point> {
  typedef point_tag type;
};
template <>
struct access<Geometry2D::Point, 0> {
  static double get(Geometry2D::Point const &p) { return p.x; }
  static void set(Geometry2D::Point &p, double const &value) { p.x = value; }
};
template <>
struct access<Geometry2D::Point, 1> {
  static double get(Geometry2D::Point const &p) { return p.y; }
  static void set(Geometry2D::Point &p, double const &value) { p.y = value; }
};
};  // namespace boost::geometry::traits

class Space2D {
 public:
  static const int Dimension = 2;
  using Geometry = Geometry2D;
  using Point = typename Geometry::Point;
  using AgentGeometry = typename Geometry::AgentGeometry;
  using Space = boost::geometry::model::polygon<Point>;

  static bool checkVertexInside(const Space &space, const Point &point,
                                const AgentGeometry &agent_geometry) {
    bool inside;
    if (!boost::geometry::within(point, space)) {
      inside = false;
    } else {
      inside = true;
      double comparable_limit = agent_geometry * agent_geometry;
      boost::geometry::for_each_segment(
          space, [&point, &inside, &comparable_limit](const auto &segment) {
            if (boost::geometry::comparable_distance(segment, point) <
                comparable_limit) {
              inside = false;
            }
          });
    }
    return inside;
  }

  static bool checkEdgeInside(const Space &space, const Point &point_0,
                              const Point &point_1,
                              const AgentGeometry &agent_geometry) {
    using Segment = boost::geometry::model::segment<Point>;
    Point vec = point_1 - point_0;
    double length = Geometry::length(vec);
    Point u0 = 1. / length * vec, u1 = Point(-u0.y, u0.x);
    for (int j = 0; j < space.inners().size(); j++) {
      auto vertex = space.inners()[j][0] - point_0;
      double ip0 = Geometry::inner(u0, vertex),
             ip1 = Geometry::inner(u1, vertex);
      if (0 < ip0 && ip0 < length && std::abs(ip1) < agent_geometry)
        return false;
    }
    Point d = agent_geometry * u1;
    std::vector<Point> rectangle{point_0 - d, point_1 - d, point_1 + d,
                                 point_0 + d};
    bool inside = true;
    boost::geometry::for_each_segment(
        space, [&rectangle, &inside](const auto &edge) {
          for (int i = 0; i < 4; i++) {
            if (boost::geometry::intersects(
                    edge, Segment(rectangle[i], rectangle[(i + 1) % 4]))) {
              inside = false;
            }
          }
        });
    return inside;
  }

  static std::pair<double, double> getBound(const Space &space,
                                            const int axis) {
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -std::numeric_limits<double>::infinity();
    auto outer_vertice = boost::geometry::exterior_ring(space);
    auto it = boost::begin(outer_vertice);
    for (++it; it != boost::end(outer_vertice); ++it) {
      double coordinate = axis == 0 ? it->x : it->y;
      minimum = std::min(minimum, coordinate);
      maximum = std::max(maximum, coordinate);
    }
    return std::make_pair(minimum, maximum);
  }

  static std::vector<Point> getRandomPoints(const Space &space,
                                            const AgentGeometry &agent_geometry,
                                            const int number) {
    auto outer_vertice = boost::geometry::exterior_ring(space);
    auto it = boost::begin(outer_vertice);
    double min_x = it->x, max_x = min_x;
    double min_y = it->y, max_y = min_y;
    for (++it; it != boost::end(outer_vertice); ++it) {
      min_x = std::min(min_x, it->x);
      max_x = std::max(max_x, it->x);
      min_y = std::min(min_y, it->y);
      max_y = std::max(max_y, it->y);
    }

    std::random_device random_device;
    std::default_random_engine engine(random_device());
    std::uniform_real_distribution<double> x_distribution(
        min_x + agent_geometry, max_x - agent_geometry);
    std::uniform_real_distribution<double> y_distribution(
        min_y + agent_geometry, max_y - agent_geometry);
    std::vector<Point> points(number);
    for (int i = 0; i < number; i++) {
      do {
        points[i] = Point(x_distribution(engine), y_distribution(engine));
      } while (!checkVertexInside(space, points[i], agent_geometry));
    }
    return points;
  }
  static std::vector<Point> getSepareteRandomPoints(
      const Space &space, const AgentGeometry &agent_geometry,
      const int number) {
    auto outer_vertice = boost::geometry::exterior_ring(space);
    auto it = boost::begin(outer_vertice);
    double min_x = it->x, max_x = min_x;
    double min_y = it->y, max_y = min_y;
    for (++it; it != boost::end(outer_vertice); ++it) {
      min_x = std::min(min_x, it->x);
      max_x = std::max(max_x, it->x);
      min_y = std::min(min_y, it->y);
      max_y = std::max(max_y, it->y);
    }

    std::random_device random_device;
    std::default_random_engine engine(random_device());
    std::uniform_real_distribution<double> x_distribution(
        min_x + agent_geometry, max_x - agent_geometry);
    std::uniform_real_distribution<double> y_distribution(
        min_y + agent_geometry, max_y - agent_geometry);
    std::vector<Point> points(number);
    for (int i = 0; i < number; i++) {
      while (true) {
        points[i] = Point(x_distribution(engine), y_distribution(engine));
        if (!checkVertexInside(space, points[i], agent_geometry)) continue;
        bool separeted = true;
        for (int j = 0; j < i; j++) {
          if (Geometry::distance(points[i], points[j]) < 2 * agent_geometry) {
            separeted = false;
            break;
          }
        }
        if (separeted) break;
      }
    }
    return points;
  }

  static constexpr double pi = 4. * atan(1.0);

  static double calculateFreeVolume(const Space &space,
                                    const AgentGeometry &agent_geometry) {
    double area = boost::geometry::area(space);
    boost::geometry::for_each_segment(
        space, [&area, &agent_geometry](const auto &segment) {
          area -= agent_geometry * boost::geometry::length(segment);
        });
    area -= pi * agent_geometry * agent_geometry * (1 + space.inners().size());
    return area;
  }
};

#endif
