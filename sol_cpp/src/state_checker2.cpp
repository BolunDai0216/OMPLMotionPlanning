#include "state_checker2.hpp"
#include <iostream>
#include <cmath>

CustomStateValidityChecker::CustomStateValidityChecker(const ompl::base::SpaceInformationPtr& si)
  : ompl::base::StateValidityChecker(si)
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

  R_box1 << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  R_box2 << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  R_box3 << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  p_box1 << 1.55, 1.0, 0.05;
  p_box2 << 1.35, 0.2, 0.05;
  p_box3 << 1.75, 0.2, 0.05;

  T_box1_fcl = hpp::fcl::Transform3f(R_box1, p_box1);
  T_box2_fcl = hpp::fcl::Transform3f(R_box2, p_box2);
  T_box3_fcl = hpp::fcl::Transform3f(R_box3, p_box3);

  T_link1_fcl = hpp::fcl::Transform3f();
  T_link2_fcl = hpp::fcl::Transform3f();
  T_link3_fcl = hpp::fcl::Transform3f();
  T_box_fcl = hpp::fcl::Transform3f();

  box_transforms.push_back(T_box1_fcl);
  box_transforms.push_back(T_box2_fcl);
  box_transforms.push_back(T_box3_fcl);
}

bool CustomStateValidityChecker::isValid(const ompl::base::State* state) const
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

  T_link1_pin = pinocchio::SE3(T_link1);
  T_link2_pin = pinocchio::SE3(T_link2);
  T_link3_pin = pinocchio::SE3(T_link3);
  T_box_pin = pinocchio::SE3(T_box);

  T_link1_fcl.setTransform(T_link1_pin.rotation(), T_link1_pin.translation());
  T_link2_fcl.setTransform(T_link2_pin.rotation(), T_link2_pin.translation());
  T_link3_fcl.setTransform(T_link3_pin.rotation(), T_link3_pin.translation());
  T_box_fcl.setTransform(T_box_pin.rotation(), T_box_pin.translation());

  hpp::fcl::CollisionRequest req;
  hpp::fcl::CollisionResult res;

  bool col11 = hpp::fcl::collide(&box1_col, T_box1_fcl, &link1_col, T_link1_fcl, req, res);
  res.clear();
  bool col12 = hpp::fcl::collide(&box1_col, T_box1_fcl, &link2_col, T_link2_fcl, req, res);
  res.clear();
  bool col13 = hpp::fcl::collide(&box1_col, T_box1_fcl, &link3_col, T_link3_fcl, req, res);
  res.clear();
  bool col14 = hpp::fcl::collide(&box1_col, T_box1_fcl, &box_col, T_box_fcl, req, res);
  res.clear();

  bool col21 = hpp::fcl::collide(&box2_col, T_box2_fcl, &link1_col, T_link1_fcl, req, res);
  res.clear();
  bool col22 = hpp::fcl::collide(&box2_col, T_box2_fcl, &link2_col, T_link2_fcl, req, res);
  res.clear();
  bool col23 = hpp::fcl::collide(&box2_col, T_box2_fcl, &link3_col, T_link3_fcl, req, res);
  res.clear();
  bool col24 = hpp::fcl::collide(&box2_col, T_box2_fcl, &box_col, T_box_fcl, req, res);
  res.clear();

  bool col31 = hpp::fcl::collide(&box3_col, T_box3_fcl, &link1_col, T_link1_fcl, req, res);
  res.clear();
  bool col32 = hpp::fcl::collide(&box3_col, T_box3_fcl, &link2_col, T_link2_fcl, req, res);
  res.clear();
  bool col33 = hpp::fcl::collide(&box3_col, T_box3_fcl, &link3_col, T_link3_fcl, req, res);
  res.clear();
  bool col34 = hpp::fcl::collide(&box3_col, T_box3_fcl, &box_col, T_box_fcl, req, res);
  res.clear();

  bool in_collision =
      !col11 * !col12 * !col13 * !col14 * !col21 * !col22 * !col23 * !col24 * !col31 * !col32 * !col33 * !col34;

  return in_collision;
}