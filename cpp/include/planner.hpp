#pragma once

#include <ompl/base/goals/GoalRegion.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <iostream>
#include <array>
#include "state_checker.hpp"
#include "goal_checker.hpp"

void plan(std::array<double, 3> q_start, std::array<double, 3> goal)
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
  start[0] = q_start[0];
  start[1] = q_start[1];
  start[2] = q_start[2];

  auto myCustomGoal = std::make_shared<CustomGoal>(si);
  myCustomGoal->goal_x = goal[0];
  myCustomGoal->goal_y = goal[1];
  myCustomGoal->goal_z = goal[2];

  // create a problem instance
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));

  // set the start and goal states
  pdef->addStartState(start);
  pdef->setGoal(myCustomGoal);

  // create a planner for the defined space
  auto planner(std::make_shared<og::RRTstar>(si));

  // set the problem we are trying to solve for the planner
  planner->setProblemDefinition(pdef);

  // perform setup steps for the planner
  planner->setup();

  // print the settings for this space
  si->printSettings(std::cout);

  // print the problem settings
  pdef->print(std::cout);

  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = planner->ob::Planner::solve(10.0);

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