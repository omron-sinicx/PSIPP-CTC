/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include "base/load_generators.hpp"

#include "roadmap_generators/ConstrainedTriangulation.hpp"
#include "roadmap_generators/Grid.cpp"
#include "roadmap_generators/ProbabilisticRoadmap.cpp"

std::shared_ptr<RoadmapGenerator<Space2D>> makeGenerator(
    const YAML::Node& config) {
  std::shared_ptr<RoadmapGenerator<Space2D>> generator;
  std::string generator_type = config["type"].as<std::string>();
  if (generator_type == "PRM") {
    int number_of_vertices = config["number_of_vertices"].as<int>();
    double radius = config["radius"].as<double>();
    std::shared_ptr<RadiusProbabilisticRoadmap2D> PRM(
        new RadiusProbabilisticRoadmap2D(number_of_vertices, radius));
    generator = PRM;
  } else if (generator_type == "PRMStar") {
    int number_of_vertices = config["number_of_vertices"].as<int>();
    std::shared_ptr<RadiusProbabilisticRoadmap2D> PRM(
        new RadiusProbabilisticRoadmap2D(number_of_vertices));
    generator = PRM;
  } else if (generator_type == "kPRM") {
    int number_of_vertices = config["number_of_vertices"].as<int>();
    int number_of_neighbors = config["number_of_neighbors"].as<int>();
    std::shared_ptr<KNearestProbabilisticRoadmap<Space2D>> PRM(
        new KNearestProbabilisticRoadmap<Space2D>(number_of_vertices,
                                                  number_of_neighbors));
    generator = PRM;
  } else if (generator_type == "kPRMStar") {
    int number_of_vertices = config["number_of_vertices"].as<int>();
    std::shared_ptr<KNearestProbabilisticRoadmap<Space2D>> PRM(
        new KNearestProbabilisticRoadmap<Space2D>(number_of_vertices));
    generator = PRM;
  } else if (generator_type == "GRID") {
    double grid_size = config["grid_size"].as<double>();
    int K = config["K"].as<int>();
    std::shared_ptr<GridGenerator> GG(new GridGenerator(grid_size, K));
    generator = GG;
  } else if (generator_type == "CDT") {
    int number_of_additionals = 0;
    bool use_corner_points = true;
    if (config["number_of_additionals"]) {
      number_of_additionals = config["number_of_additionals"].as<int>();
    }
    if (config["use_corner_points"]) {
      use_corner_points = config["use_corner_points"].as<bool>();
    }
    std::shared_ptr<ConstrainedTriangulation> CDT(
        new ConstrainedTriangulation(number_of_additionals, use_corner_points));
    generator = CDT;
  } else {
    throw std::runtime_error("Unknown Generator Type");
  }
  return generator;
}
