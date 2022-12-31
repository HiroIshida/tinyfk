/*
Copyright (c) 2020 Hirokazu Ishida
This software is released under the MIT License, see LICENSE.
tinyfk: https://github.com/HiroIshida/tinyfk
*/

#include "tinyfk.hpp"
#include <Eigen/Dense>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stdexcept>
#include <utility>

namespace py = pybind11;
using namespace tinyfk;

class RobotModelPyWrapper {
public:
  CacheUtilizedRobotModel robot_model_;
  RobotModelPyWrapper(const std::string &xml_string)
      : robot_model_(CacheUtilizedRobotModel(xml_string)) {}

  void set_joint_angles(const std::vector<size_t> &joint_ids,
                        const std::vector<double> &joint_angles) {
    robot_model_.set_joint_angles(joint_ids, joint_angles);
  }

  void set_base_pose(const std::vector<double> &xytheta) {
    robot_model_.set_base_pose(xytheta[0], xytheta[1], xytheta[2]);
  }

  std::array<Eigen::MatrixXd, 2> solve_forward_kinematics(
      const std::vector<std::vector<double>> joint_angles_sequence,
      const std::vector<size_t> &elink_ids,
      const std::vector<size_t> &joint_ids, bool with_rpy, bool with_base,
      bool with_jacobian, bool use_cache) {

    size_t n_pose_dim = (with_rpy ? 6 : 3); // 7 if rot enabled
    auto n_wp = joint_angles_sequence.size();
    auto n_link = elink_ids.size();
    auto n_joints = joint_ids.size();
    auto n_dof = (with_base ? n_joints + 3 : n_joints);

    if (with_base && (n_joints != joint_angles_sequence[0].size() - 3)) {
      throw std::invalid_argument(
          "dof mismatch!! Probably you forget base's (x, y, theta)");
      // TODO this check try to prevent the potentionally buggy procedure below
    }

    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n_pose_dim, n_wp * n_link);
    Eigen::MatrixXd J =
        Eigen::MatrixXd::Zero(n_wp * n_link * n_pose_dim, n_dof);

    for (size_t i = 0; i < n_wp; i++) {
      robot_model_._set_joint_angles(joint_ids, joint_angles_sequence[i]);
      if (with_base) {
        double x = joint_angles_sequence[i][n_joints + 0];
        double y = joint_angles_sequence[i][n_joints + 1];
        double theta = joint_angles_sequence[i][n_joints + 2];
        robot_model_._set_base_pose(x, y, theta);
      }
      if (!use_cache) {
        robot_model_.clear_cache();
      }

      for (size_t j = 0; j < n_link; ++j) {
        const size_t head = i * n_link + j;
        urdf::Pose pose;
        robot_model_.get_link_pose(elink_ids[j], pose, with_base);
        P(0, head) = pose.position.x;
        P(1, head) = pose.position.y;
        P(2, head) = pose.position.z;
        if (with_rpy) {
          urdf::Vector3 rpy = pose.rotation.getRPY();
          P(3, head) = rpy.x;
          P(4, head) = rpy.y;
          P(5, head) = rpy.z;
        }
      }
      if (with_jacobian) {
        for (size_t j = 0; j < n_link; ++j) {
          const size_t head = i * (n_link * n_pose_dim) + j * n_pose_dim;
          const size_t elink_id = elink_ids[j];
          J.block(head, 0, n_pose_dim, n_dof) = robot_model_.get_jacobian(
              elink_id, joint_ids, with_rpy, with_base);
        }
      }
    }
    return std::array<Eigen::MatrixXd, 2>{P.transpose(), J};
  }

  std::vector<size_t> get_joint_ids(std::vector<std::string> joint_names) {
    size_t n_joint = joint_names.size();
    return robot_model_.get_joint_ids(joint_names);
  }

  std::vector<size_t> get_link_ids(std::vector<std::string> link_names) {
    size_t n_link = link_names.size();
    return robot_model_.get_link_ids(link_names);
  }

  std::vector<std::pair<double, double>>
  get_joint_limits(const std::vector<size_t> &joint_ids) {
    return robot_model_.get_joint_limits(joint_ids);
  }

  void add_new_link(std::string link_name, size_t parent_id,
                    std::array<double, 3> position,
                    std::array<double, 3> rotation) {
    robot_model_.add_new_link(link_name, parent_id, position, rotation);
  }

  std::pair<Eigen::VectorXd, Eigen::MatrixXd>
  compute_inter_link_squared_dists(const std::vector<std::vector<double>> &qs,
                                   const std::vector<size_t> &link_ids1,
                                   const std::vector<size_t> &link_ids2,
                                   const std::vector<size_t> &joint_ids,
                                   bool with_base, bool with_grads,
                                   bool use_cache) {
    const size_t n_wp = qs.size();
    const size_t n_joint = joint_ids.size() + with_base * 3;
    const size_t n_check = link_ids1.size();

    Eigen::VectorXd sqdists = Eigen::VectorXd::Zero(n_check * n_wp);
    Eigen::MatrixXd grads = with_grads
                                ? Eigen::MatrixXd::Zero(n_check * n_wp, n_joint)
                                : Eigen::MatrixXd::Zero(1, 1);

    for (size_t i = 0; i < n_wp; i++) {
      robot_model_._set_joint_angles(joint_ids, qs[i]);
      if (with_base) {
        double x = qs[i][n_joint + 0];
        double y = qs[i][n_joint + 1];
        double theta = qs[i][n_joint + 2];
        robot_model_._set_base_pose(x, y, theta);
      }
      if (!use_cache) {
        robot_model_.clear_cache();
      }

      for (size_t j = 0; j < n_check; ++j) {
        const size_t head = i * n_check + j;
        urdf::Pose pose1, pose2;
        robot_model_.get_link_pose(link_ids1[j], pose1, with_base);
        robot_model_.get_link_pose(link_ids2[j], pose2, with_base);
        const auto diff = pose1.position - pose2.position;
        Eigen::Vector3d diff_vec;
        diff_vec << diff.x, diff.y, diff.z;
        const double sqdist = diff_vec.squaredNorm();
        sqdists(head) = sqdist;

        if (with_grads) {
          const auto &&jac1 = robot_model_.get_jacobian(link_ids1[j], joint_ids,
                                                        false, with_base);
          const auto &&jac2 = robot_model_.get_jacobian(link_ids2[j], joint_ids,
                                                        false, with_base);
          const auto grad = 2 * (jac1 - jac2).transpose() * diff_vec;
          grads.block(head, 0, 1, n_joint) = grad.transpose();
        }
      }
    }
    return std::pair<Eigen::VectorXd, Eigen::MatrixXd>{sqdists, grads};
  }

  void clear_cache() { robot_model_.clear_cache(); }
};

PYBIND11_MODULE(_tinyfk, m) {
  m.doc() = "tiny fast forward kinematics solver"; // optional module docstring
  py::class_<RobotModelPyWrapper>(m, "RobotModel")
      .def(py::init<std::string &>())
      .def("solve_forward_kinematics",
           &RobotModelPyWrapper::solve_forward_kinematics)
      .def("set_joint_angles", &RobotModelPyWrapper::set_joint_angles)
      .def("get_joint_ids", &RobotModelPyWrapper::get_joint_ids)
      .def("get_joint_limits", &RobotModelPyWrapper::get_joint_limits)
      .def("set_base_pose", &RobotModelPyWrapper::set_base_pose)
      .def("get_link_ids", &RobotModelPyWrapper::get_link_ids)
      .def("add_new_link", &RobotModelPyWrapper::add_new_link)
      .def("compute_inter_link_squared_dists",
           &RobotModelPyWrapper::compute_inter_link_squared_dists)
      .def("clear_cache", &RobotModelPyWrapper::clear_cache);
}
