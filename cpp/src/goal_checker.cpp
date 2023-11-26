#include "goal_checker.hpp"

CustomGoal::CustomGoal(const ompl::base::SpaceInformationPtr& si) : ompl::base::GoalRegion(si)
{
  setThreshold(0.1);

  // build pin_robot from urdf
  urdf_filename = "/home/AnywareInterview/python/robots/robot.urdf";
  pinocchio::urdf::buildModel(urdf_filename, model);
  data = pinocchio::Data(model);

  // get link frame indices
  boxId = model.getFrameId("box");
}

void CustomGoal::setGoal(const std::array<double, 3>& goal)
{
  goal_x = goal[0];
  goal_y = goal[1];
  goal_z = goal[2];
}

double CustomGoal::distanceGoal(const ompl::base::State* state) const
{
  // cast the state to the RealVectorStateSpace
  const auto* realState = state->as<ompl::base::RealVectorStateSpace::StateType>();

  // access the elements
  double q1 = realState->values[0];
  double q2 = realState->values[1];
  double q3 = realState->values[2];

  // set the joint angle and joint velocity vectors
  Eigen::Matrix<double, 3, 1> q;
  q << q1, q2, q3;

  Eigen::Matrix<double, 3, 1> dq;
  dq << 0.0, 0.0, 0.0;

  // update pinocchio robot model
  pinocchio::forwardKinematics(model, data, q, dq);
  pinocchio::updateFramePlacements(model, data);

  auto p_box = data.oMf[boxId].translation();

  sum = std::pow(p_box(0) - goal_x, 2) + std::pow(p_box(1) - goal_y, 2) + std::pow(p_box(2) - goal_z, 2);
  error = std::sqrt(sum);

  return error;
}