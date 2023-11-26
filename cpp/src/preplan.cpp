#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

#include "planner.hpp"

int main(int /*argc*/, char** /*argv*/)
{
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  double q1_start, q2_start, q3_start;
  double goal_x, goal_y, goal_z;
  double box1_x, box1_y, box1_theta, box2_x, box2_y, box2_theta, box3_x, box3_y, box3_theta;

  try
  {
    YAML::Node config = YAML::LoadFile("/home/AnywareInterview/cpp/preplan_config.yaml");
    goal_x = config["goal_x"].as<double>();
    goal_y = config["goal_y"].as<double>();
    goal_z = config["goal_z"].as<double>();
    box1_x = config["box1_x"].as<double>();
    box1_y = config["box1_y"].as<double>();
    box1_theta = config["box1_theta"].as<double>();
    box2_x = config["box2_x"].as<double>();
    box2_y = config["box2_y"].as<double>();
    box2_theta = config["box2_theta"].as<double>();
    box3_x = config["box3_x"].as<double>();
    box3_y = config["box3_y"].as<double>();
    box3_theta = config["box3_theta"].as<double>();
  }
  catch (const YAML::Exception& e)
  {
    std::cerr << "Error parsing YAML file: " << e.what() << std::endl;
    return 1;
  }

  if (goal_y >= 0.0)
  {
    q1_start = 0.0;
    q2_start = -1.57;
    q3_start = 0.0;
  }
  else
  {
    q1_start = 0.0;
    q2_start = 1.57;
    q3_start = 0.0;
  }

  // q1, q2, q3
  std::array<double, 3> q_start = { q1_start, q2_start, q3_start };

  // x, y, z
  std::array<double, 3> goal = { goal_x, goal_y, goal_z };

  // x1, y1, theta1, x2, y2, theta2, x3, y3, theta3
  std::array<double, 9> box_pose = {
    box1_x, box1_y, box1_theta, box2_x, box2_y, box2_theta, box3_x, box3_y, box3_theta
  };

  double threshold = 0.1;

  plan(q_start, goal, box_pose, threshold);

  return 0;
}