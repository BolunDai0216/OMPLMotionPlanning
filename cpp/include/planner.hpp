#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <ompl/base/Goal.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <iostream>
#include "state_checker.hpp"

class CustomGoal : public ompl::base::Goal
{
public:
  CustomGoal(const ompl::base::SpaceInformationPtr& si) : ompl::base::Goal(si)
  {
  }

  virtual bool isSatisfied(const ompl::base::State* state) const override
  {
    // cast the state to the RealVectorStateSpace
    const auto* realState = state->as<ob::RealVectorStateSpace::StateType>();

    // access the elements
    double q1 = realState->values[0];
    double q2 = realState->values[1];
    double q3 = realState->values[2];

    sum = std::pow(q1 - 0.0, 2) + std::pow(q2 + pi / 2, 2) + std::pow(q3 - 0.0, 2);
    error = std::sqrt(sum);

    if (error < 0.1)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
};

void plan()
{
  // construct the state space we are planning in
  auto space(std::make_shared<ob::RealVectorStateSpace>(3));

  // set the bounds for R^3
  ob::RealVectorBounds bounds(3);
  bounds.setLow(0, 0.0);
  bounds.setHigh(0, pi);

  bounds.setLow(1, -pi / 2);
  bounds.setHigh(1, pi / 2);

  bounds.setLow(2, -pi / 2);
  bounds.setHigh(2, pi / 2);

  space->setBounds(bounds);

  // construct an instance of space information from this state space
  auto si(std::make_shared<ob::SpaceInformation>(space));

  // set state validity checking for this space
  si->setStateValidityChecker(std::make_shared<CustomStateValidityChecker>(si));

  // create a random start state
  ob::ScopedState<> start(space);
  start[0] = 0.611;
  start[1] = 0.215;
  start[2] = -0.826;

  // create a random goal state
  // ob::ScopedState<> goal(space);
  // goal[0] = 0.0;
  // goal[1] = -pi / 2;
  // goal[2] = 0.0;

  auto myCustomGoal = std::make_shared<CustomGoal>(si);

  // create a problem instance
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));

  // set the start and goal states
  pdef->setStartAndGoalStates(start, goal);
  pdef->addStartState(start);
  pdef->setGoal(myCustomGoal);

  // create a planner for the defined space
  auto planner(std::make_shared<og::RRTConnect>(si));

  // set the problem we are trying to solve for the planner
  planner->setProblemDefinition(pdef);

  // perform setup steps for the planner
  planner->setup();

  // print the settings for this space
  si->printSettings(std::cout);

  // print the problem settings
  pdef->print(std::cout);

  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

  if (solved)
  {
    // get the goal representation from the problem definition (not the same as the goal state)
    // and inquire about the found path
    ob::PathPtr path = pdef->getSolutionPath();
    std::cout << "Found solution:" << std::endl;

    // print the path to screen
    path->print(std::cout);
  }
  else
    std::cout << "No solution found" << std::endl;
}

#endif  // PLANNER_HPP