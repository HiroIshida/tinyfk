/*
Copyright (c) 2020 Hirokazu Ishida
This software is released under the MIT License, see LICENSE.
tinyfk: https://github.com/HiroIshida/tinyfk
*/

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "../src/tinyfk.hpp"
#include <Eigen/Dense>

namespace py = pybind11;
using namespace tinyfk;

class RobotModelPyWrapper
{
  public:
    RobotModel _rtree;
    RobotModelPyWrapper(const std::string& urdf_file) :
      _rtree(RobotModel(urdf_file)) {}

    void set_joint_angles(
        const std::vector<unsigned int>& joint_ids, const std::vector<double>& joint_angles)
    {
      _rtree.set_joint_angles(joint_ids, joint_angles);
    }

    void set_base_pose(const std::vector<double>& xytheta)
    {
      _rtree.set_base_pose(xytheta[0], xytheta[1], xytheta[2]);
    }

    std::array<Eigen::MatrixXd, 2> solve_forward_kinematics(
        const std::vector<std::vector<double>> joint_angles_sequence,
        const std::vector<unsigned int>& elink_ids,
        const std::vector<unsigned int>& joint_ids,
        bool rotalso, bool basealso, bool with_jacobian, bool use_cache)
    {

      unsigned int n_pose_dim = (rotalso ? 6 : 3); // 7 if rot enabled
      auto n_wp = joint_angles_sequence.size();
      auto n_links = elink_ids.size();
      auto n_joints = joint_ids.size();
      auto n_dof = (basealso ? n_joints + 3 : n_joints);

      if(basealso && (n_joints != joint_angles_sequence[0].size() - 3)){
        throw std::invalid_argument("dof mismatch!! Probably you forget base's (x, y, theta)");
        // TODO this check try to prevent the potentionally buggy procedure below
      }

      Eigen::MatrixXd J_trajectory = with_jacobian ? 
        Eigen::MatrixXd::Zero(n_wp * (n_links * n_pose_dim), n_dof) : 
        Eigen::MatrixXd::Zero(0, 0);
      Eigen::MatrixXd P_trajectory = Eigen::MatrixXd::Zero(n_pose_dim, n_wp * n_links);

      for(unsigned int i=0; i<n_wp; i++){
        if(!use_cache){
          if(basealso){
            // in this case, latter 3 elements represents x, y, theta of the base
            _rtree.set_joint_angles(joint_ids, joint_angles_sequence[i]);
            // TODO potentionally buggy
            double x = joint_angles_sequence[i][n_joints + 0];
            double y = joint_angles_sequence[i][n_joints + 1];
            double theta = joint_angles_sequence[i][n_joints + 2];
            _rtree.set_base_pose(x, y, theta);
          }else{
            _rtree.set_joint_angles(joint_ids, joint_angles_sequence[i]);
          }
        }

        if(with_jacobian){
          auto tmp = _rtree.get_jacobians_withcache(elink_ids, joint_ids, rotalso, basealso);
          auto& J = tmp[0];
          auto& P = tmp[1];
          J_trajectory.block(i * (n_links * n_pose_dim), 0, n_links * n_pose_dim, n_dof) = J;
          P_trajectory.block(0, i * n_links, n_pose_dim, n_links) = P;
        }else{
          urdf::Pose pose;
          for(int j=0; j<elink_ids.size(); j++){
            _rtree.get_link_point_withcache(elink_ids[j], pose, basealso);
            P_trajectory(0, i * n_links + j) = pose.position.x;
            P_trajectory(1, i * n_links + j) = pose.position.y;
            P_trajectory(2, i * n_links + j) = pose.position.z;
            if(rotalso){
              urdf::Vector3&& rpy = pose.rotation.getRPY(); 
              P_trajectory(3, i * n_links + j) = rpy.x;
              P_trajectory(4, i * n_links + j) = rpy.y;
              P_trajectory(5, i * n_links + j) = rpy.z;
            }
          }
        }
      }
      std::array<Eigen::MatrixXd, 2> ret = {P_trajectory.transpose(), J_trajectory};
      return ret;
    }

    std::vector<unsigned int> get_joint_ids(std::vector<std::string> joint_names){
      int n_joint = joint_names.size();
      return _rtree.get_joint_ids(joint_names);
    }

    std::vector<unsigned int> get_link_ids(std::vector<std::string> link_names){
      int n_link = link_names.size();
      return _rtree.get_link_ids(link_names);
    }

    void add_new_link(std::string link_name, unsigned int parent_id, std::array<double, 3> position){ 
      _rtree.add_new_link(link_name, parent_id, position);
    }

};

PYBIND11_MODULE(_tinyfk, m) {
    m.doc() = "tiny fast forward kinematics solver"; // optional module docstring
    py::class_<RobotModelPyWrapper>(m, "RobotModel")
            .def(py::init<std::string &>())
            .def("solve_forward_kinematics", &RobotModelPyWrapper::solve_forward_kinematics)
            .def("set_joint_angles", &RobotModelPyWrapper::set_joint_angles)
            .def("get_joint_ids", &RobotModelPyWrapper::get_joint_ids)
            .def("set_base_pose", &RobotModelPyWrapper::set_base_pose)
            .def("add_new_link", &RobotModelPyWrapper::add_new_link)
            .def("get_link_ids", &RobotModelPyWrapper::get_link_ids);
}
