#include <iostream>

#include "planner.hpp"

int main(int /*argc*/, char** /*argv*/)
{
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  std::array<double, 3> q_start = { 0.611, 0.215, -0.826 };
  std::array<double, 3> goal = { 0.5, -1.0, 0.05 };

  plan(q_start, goal);

  return 0;
}