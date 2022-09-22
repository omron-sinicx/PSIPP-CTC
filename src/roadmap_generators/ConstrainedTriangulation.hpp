/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include "base/geometry_2D_for_generator.hpp"
#include "base/roadmap_generator.hpp"

class ConstrainedTriangulation : public RoadmapGenerator<Space2D> {
  int number_of_additionals;
  bool use_corner_points;

 public:
  ConstrainedTriangulation(const int number_of_additionals = 0,
                           const bool use_corner_points = false) {
    this->number_of_additionals = number_of_additionals;
    this->use_corner_points = use_corner_points;
  }
  void generate(
      const SpatialProblemInstance<Space2D>& spatial_problem_instance) override;
};
