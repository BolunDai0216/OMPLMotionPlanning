#ifndef STATE_CHECKER_HPP
#define STATE_CHECKER_HPP

#include <Eigen/Dense>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <cmath>
#include <vector>

const double pi = M_PI;  // π as a double
namespace ob = ompl::base;
namespace og = ompl::geometric;

// Custom State Validity Checker
class CustomStateValidityChecker : public ob::StateValidityChecker
{
public:
  CustomStateValidityChecker(const ob::SpaceInformationPtr& si);

  // Check if a state is valid
  bool isValid(const ob::State* state) const override;

private:
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

  mutable pinocchio::SE3 T_link1_pin;
  mutable pinocchio::SE3 T_link2_pin;
  mutable pinocchio::SE3 T_link3_pin;
  mutable pinocchio::SE3 T_box_pin;

  std::vector<hpp::fcl::Transform3f> box_transforms;

  mutable hpp::fcl::CollisionRequest req;
  mutable hpp::fcl::CollisionResult res;
};

#endif  // STATE_CHECKER_HPP