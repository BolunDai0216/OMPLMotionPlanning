#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <iostream>
#include "state_checker.hpp"

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
  ob::ScopedState<> goal(space);
  goal[0] = 0.0;
  goal[1] = -pi / 2;
  goal[2] = 0.0;

  // create a problem instance
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));

  // set the start and goal states
  pdef->setStartAndGoalStates(start, goal);

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