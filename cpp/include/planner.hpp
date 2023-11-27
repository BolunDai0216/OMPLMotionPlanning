#pragma once

#include <ompl/base/goals/GoalRegion.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <iostream>
#include <array>
#include "state_checker.hpp"
#include "goal_checker.hpp"

void plan(const std::array<double, 3>& q_start, const std::array<double, 3>& goal,
          const std::array<double, 9>& box_pose, const double threshold)
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
  auto customStateChecker = std::make_shared<CustomStateValidityChecker>(si);
  customStateChecker->setBoxPose(box_pose);

  // set state validity checking for this space
  si->setStateValidityChecker(customStateChecker);

  // create a random start state
  ob::ScopedState<> start(space);
  start[0] = q_start[0];
  start[1] = q_start[1];
  start[2] = q_start[2];

  auto customGoal = std::make_shared<CustomGoal>(si);
  customGoal->setThresh(threshold);
  customGoal->setGoal(goal);

  // create a problem instance
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));

  // set the start and goal states
  pdef->addStartState(start);
  pdef->setGoal(customGoal);

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

  // attempt to solve the problem within ten seconds of planning time
  ob::PlannerStatus solved = planner->ob::Planner::solve(20.0);

  if (solved)
  {
    // get the goal representation from the problem definition (not the same as the goal state)
    // and inquire about the found path
    auto path = pdef->getSolutionPath();
    std::cout << "Found solution:" << std::endl;

    std::vector<double> reals;
    for (const auto &state : path.getStates())
    {
        space->copyToReals(reals, state);

        // Or copy out however you want
        const auto &vector = Eigen::Map<Eigen::VectorXd>(reals.data(), dimension);
    }
  }
  else
    std::cout << "No solution found" << std::endl;
}

void planConnect(const std::array<double, 3>& q_start, const std::array<double, 3>& q_goal,
                 const std::array<double, 9>& box_pose, const double threshold)
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
  auto customStateChecker = std::make_shared<CustomStateValidityChecker>(si);
  customStateChecker->setBoxPose(box_pose);

  // set state validity checking for this space
  si->setStateValidityChecker(customStateChecker);

  // create a random start state
  ob::ScopedState<> start(space);
  start[0] = q_start[0];
  start[1] = q_start[1];
  start[2] = q_start[2];

  ob::ScopedState<> goal(space);
  goal[0] = q_goal[0];
  goal[1] = q_goal[1];
  goal[2] = q_goal[2];

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

  // attempt to solve the problem within ten seconds of planning time
  ob::PlannerStatus solved = planner->ob::Planner::solve(5.0);

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