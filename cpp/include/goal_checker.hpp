#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

class CustomGoal : public ompl::base::GoalRegion
{
public:
  CustomGoal(const ompl::base::SpaceInformationPtr& si);
  virtual double distanceGoal(const ompl::base::State* state) const;

  double goal_x;
  double goal_y;
  double goal_z;

private:
  mutable double sum;
  mutable double error;

  pinocchio::Model model;
  mutable pinocchio::Data data;
  std::string urdf_filename;

  pinocchio::Model::FrameIndex boxId;
};