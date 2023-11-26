#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

class CustomGoal : public ompl::base::GoalRegion
{
public:
  CustomGoal(const ompl::base::SpaceInformationPtr& si);
  virtual double distanceGoal(const ompl::base::State* state) const;

private:
  mutable double sum;
  mutable double error;
};