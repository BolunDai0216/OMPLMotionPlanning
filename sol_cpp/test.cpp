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

  hpp::fcl::Box link1_col;
  hpp::fcl::Box link2_col;
  hpp::fcl::Box link3_col;
  hpp::fcl::Box box_col;

  hpp::fcl::Box box1_col;
  hpp::fcl::Box box2_col;
  hpp::fcl::Box box3_col;

  Eigen::Matrix4d T_link_offset;
  Eigen::Matrix4d T_box_offset;
  mutable Eigen::Matrix4d T_link1;
  mutable Eigen::Matrix4d T_link2;
  mutable Eigen::Matrix4d T_link3;
  mutable Eigen::Matrix4d T_box;

  mutable hpp::fcl::Transform3f T_link1_fcl;
  mutable hpp::fcl::Transform3f T_link2_fcl;
  mutable hpp::fcl::Transform3f T_link3_fcl;
  mutable hpp::fcl::Transform3f T_box_fcl;

  hpp::fcl::Transform3f T_box1_fcl;
  hpp::fcl::Transform3f T_box2_fcl;
  hpp::fcl::Transform3f T_box3_fcl;

  Eigen::Matrix3d R_box1;
  Eigen::Matrix3d R_box2;
  Eigen::Matrix3d R_box3;
  Eigen::Vector3d p_box1;
  Eigen::Vector3d p_box2;
  Eigen::Vector3d p_box3;

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

    link1_col = hpp::fcl::Box(0.5, 0.1, 0.1);
    link2_col = hpp::fcl::Box(0.5, 0.1, 0.1);
    link3_col = hpp::fcl::Box(0.5, 0.1, 0.1);
    box_col = hpp::fcl::Box(0.4, 0.3, 0.1);

    box1_col = hpp::fcl::Box(0.4, 0.3, 0.1);
    box2_col = hpp::fcl::Box(0.4, 0.3, 0.1);
    box3_col = hpp::fcl::Box(0.4, 0.3, 0.1);

    T_link_offset << 1.0, 0.0, 0.0, 0.25, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.05, 0.0, 0.0, 0.0, 1.0;
    T_box_offset << 1.0, 0.0, 0.0, 0.2, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.05, 0.0, 0.0, 0.0, 1.0;

    // T_box1_fcl = hpp::fcl::Transform3f();
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

    T_link1 = data.oMf[link1Id].toHomogeneousMatrix() * T_link_offset;
    T_link2 = data.oMf[link2Id].toHomogeneousMatrix() * T_link_offset;
    T_link3 = data.oMf[link3Id].toHomogeneousMatrix() * T_link_offset;
    T_box = data.oMf[boxId].toHomogeneousMatrix() * T_box_offset;

    std::cout << T_box << std::endl;

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