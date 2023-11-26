#include <Eigen/Dense>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/model.hpp>

#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <iostream>
#include <cmath>

const double pi = M_PI;  // π as a double
namespace ob = ompl::base;
namespace og = ompl::geometric;
using hpp::fcl::CollisionGeometryPtr_t;

// Custom State Validity Checker
class CustomStateValidityChecker : public ob::StateValidityChecker
{
public:
  pinocchio::Model model;
  mutable pinocchio::Data data;
  std::string urdf_filename;

  pinocchio::Model::FrameIndex link1Id;
  pinocchio::Model::FrameIndex link2Id;
  pinocchio::Model::FrameIndex link3Id;
  pinocchio::Model::FrameIndex boxId;

  hpp::fcl::CollisionGeometryPtr_t link1_col(new hpp::fcl::Box(0.5, 0.1, 0.1));
  hpp::fcl::CollisionGeometryPtr_t link2_col(new hpp::fcl::Box(0.5, 0.1, 0.1));
  hpp::fcl::CollisionGeometryPtr_t link3_col(new hpp::fcl::Box(0.5, 0.1, 0.1));
  hpp::fcl::CollisionGeometryPtr_t box_col(new hpp::fcl::Box(0.4, 0.3, 0.1));

  hpp::fcl::CollisionGeometryPtr_t box1_col(new hpp::fcl::Box(0.4, 0.3, 0.1));
  hpp::fcl::CollisionGeometryPtr_t box2_col(new hpp::fcl::Box(0.4, 0.3, 0.1));
  hpp::fcl::CollisionGeometryPtr_t box3_col(new hpp::fcl::Box(0.4, 0.3, 0.1));

  CustomStateValidityChecker(const ob::SpaceInformationPtr& si) : ob::StateValidityChecker(si)
  {
    // build pin_robot from urdf
    urdf_filename = "/home/AnywareInterview/rrt_star_py/robots/robot.urdf";
    pinocchio::urdf::buildModel(urdf_filename, model);
    data = pinocchio::Data(model);

    link1Id = model.getFrameId("link1");
    link2Id = model.getFrameId("link2");
    link3Id = model.getFrameId("link3");
    boxId = model.getFrameId("box");
  }

  // Check if a state is valid
  bool isValid(const ob::State* state) const override
  {
    // Cast the state to the specific type of your state space, e.g., RealVectorStateSpace
    const auto* realState = state->as<ob::RealVectorStateSpace::StateType>();

    // Access the elements
    double q1 = realState->values[0];
    double q2 = realState->values[1];
    double q3 = realState->values[2];

    Eigen::Matrix<double, 3, 1> q;
    q << q1, q2, q3;

    Eigen::Matrix<double, 3, 1> dq;
    dq << 0.0, 0.0, 0.0;

    // update pinocchio robot model
    pinocchio::forwardKinematics(model, data, q, dq);
    pinocchio::updateFramePlacements(model, data);

    std::cout << data.oMf[boxId].toHomogeneousMatrix() << std::endl;

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