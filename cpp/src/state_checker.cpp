#include "state_checker.hpp"

CustomStateValidityChecker::CustomStateValidityChecker(const ob::SpaceInformationPtr& si) : ob::StateValidityChecker(si)
{
  // build pin_robot from urdf
  urdf_filename = "/home/AnywareInterview/rrt_star_py/robots/robot.urdf";
  pinocchio::urdf::buildModel(urdf_filename, model);
  data = pinocchio::Data(model);

  // get link frame indices
  link1Id = model.getFrameId("link1");
  link2Id = model.getFrameId("link2");
  link3Id = model.getFrameId("link3");
  boxId = model.getFrameId("box");

  // set collision objects for each link
  link1_col = hpp::fcl::Box(0.5, 0.1, 0.1);
  link2_col = hpp::fcl::Box(0.5, 0.1, 0.1);
  link3_col = hpp::fcl::Box(0.5, 0.1, 0.1);
  box_col = hpp::fcl::Box(0.4, 0.3, 0.1);

  // set collision objects for each box
  box1_col = hpp::fcl::Box(0.4, 0.3, 0.1);
  box2_col = hpp::fcl::Box(0.4, 0.3, 0.1);
  box3_col = hpp::fcl::Box(0.4, 0.3, 0.1);

  // define offsets
  T_link_offset << 1.0, 0.0, 0.0, 0.25, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.05, 0.0, 0.0, 0.0, 1.0;
  T_box_offset << 1.0, 0.0, 0.0, 0.2, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.05, 0.0, 0.0, 0.0, 1.0;

  // define box transforms
  R_box1 << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  R_box2 << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  R_box3 << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  p_box1 << 1.55, 1.0, 0.05;
  p_box2 << 1.35, 0.2, 0.05;
  p_box3 << 1.75, 0.2, 0.05;

  T_box1_fcl = hpp::fcl::Transform3f(R_box1, p_box1);
  T_box2_fcl = hpp::fcl::Transform3f(R_box2, p_box2);
  T_box3_fcl = hpp::fcl::Transform3f(R_box3, p_box3);

  // define link transforms
  T_link1_fcl = hpp::fcl::Transform3f();
  T_link2_fcl = hpp::fcl::Transform3f();
  T_link3_fcl = hpp::fcl::Transform3f();
  T_box_fcl = hpp::fcl::Transform3f();
}

bool CustomStateValidityChecker::isValid(const ob::State* state) const
{
  // cast the state to the RealVectorStateSpace
  const auto* realState = state->as<ob::RealVectorStateSpace::StateType>();

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

  // get the link transforms
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

  // check for collisions
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