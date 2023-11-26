#include "goal_checker.hpp"

CustomGoal::CustomGoal(const ompl::base::SpaceInformationPtr& si) : ompl::base::GoalRegion(si)
{
  setThreshold(0.3);
}

double CustomGoal::distanceGoal(const ompl::base::State* state) const
{
  // cast the state to the RealVectorStateSpace
  const auto* realState = state->as<ompl::base::RealVectorStateSpace::StateType>();

  // access the elements
  double q1 = realState->values[0];
  double q2 = realState->values[1];
  double q3 = realState->values[2];

  sum = std::pow(q1 - 0.0, 2) + std::pow(q2 + M_PI / 2, 2) + std::pow(q3 - 0.0, 2);
  error = std::sqrt(sum);

  return error;
}