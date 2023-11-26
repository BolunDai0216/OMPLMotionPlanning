#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <iostream>
#include <cmath>

const double pi = M_PI;  // π as a double
namespace ob = ompl::base;
namespace og = ompl::geometric;

// Custom State Validity Checker
class CustomStateValidityChecker : public ob::StateValidityChecker
{
public:
  CustomStateValidityChecker(const ob::SpaceInformationPtr& si) : ob::StateValidityChecker(si)
  {
    pinocchio::Model model;
    pinocchio::Data data;

    // build pin_robot from urdf
    std::string urdf_filename = "/home/AnywareInterview/rrt_star_py/robots/robot.urdf";
    pinocchio::urdf::buildModel(urdf_filename, model);
    data = pinocchio::Data(model);
  }

  // Check if a state is valid
  bool isValid(const ob::State* state) const override
  {
    // Cast the state to the specific type of your state space, e.g., RealVectorStateSpace
    const auto* realState = state->as<ob::RealVectorStateSpace::StateType>();

    // Access the elements
    double x = realState->values[0];
    double y = realState->values[1];
    double z = realState->values[2];

    std::cout << x << std::endl;

    return true;
  }
};

void plan()
{
  // construct the state space we are planning in
  auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));

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

int main(int /*argc*/, char** /*argv*/)
{
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
  plan();
  return 0;
}