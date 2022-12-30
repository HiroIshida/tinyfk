/*
Copyright (c) 2020 Hirokazu Ishida
This software is released under the MIT License, see LICENSE.
tinyfk: https://github.com/HiroIshida/tinyfk
*/

#ifndef tinyfk_hpp
#define tinyfk_hpp

#include "urdf_model/joint.h"
#include "urdf_model/pose.h"
#include "urdf_parser/urdf_parser.h"
#include <Eigen/Core> // slow compile...
#include <array>
#include <assert.h>
#include <fstream>
#include <iostream>
#include <stack>
#include <stdexcept>
#include <unordered_map>

#include "data_structure.hpp"

namespace tinyfk {

using AngleLimit = std::pair<double, double>;

struct RelevancePredicateTable {
  std::vector<std::vector<bool>> table_;
  RelevancePredicateTable() : RelevancePredicateTable(0, 0){};
  RelevancePredicateTable(int N_link, int N_joint) {
    for (int i = 0; i < N_joint; i++) {
      table_.push_back(std::vector<bool>(N_link));
    }
  }
  bool isRelevant(int joint_id, int link_id) const {
    return table_[joint_id][link_id];
  }
};

struct BasePose {
  std::array<double, 3> pose3d_;
  urdf::Pose pose_;
  void set(double x, double y, double theta) {
    pose_.position = urdf::Vector3(x, y, 0.0);
    pose_.rotation =
        urdf::Rotation(0.0, 0.0, 1.0 * sin(0.5 * theta), cos(0.5 * theta));
    pose3d_[0] = x;
    pose3d_[1] = y;
    pose3d_[2] = theta;
  }
};

struct LinkIdAndPose {
  size_t id;
  urdf::Pose pose;
};

class RobotModelBase {
public: // members
  // change them all to private later
  urdf::ModelInterfaceSharedPtr robot_urdf_interface_;

  urdf::LinkSharedPtr root_link_;
  std::vector<urdf::LinkSharedPtr> links_;
  std::unordered_map<std::string, int> link_ids_;

  std::vector<urdf::JointSharedPtr> joints_;
  std::unordered_map<std::string, int> joint_ids_;
  std::vector<double> joint_angles_;
  RelevancePredicateTable rptable_;
  BasePose base_pose_;
  int num_dof_;

  mutable SizedStack<LinkIdAndPose> transform_stack_;
  mutable SizedCache<urdf::Pose> transform_cache_;

public: // functions
  RobotModelBase(const std::string &xml_string);

  virtual ~RobotModelBase() {}

  void set_joint_angles( // this will clear all the cache stored
      const std::vector<size_t> &joint_ids,
      const std::vector<double> &joint_angles);
  void _set_joint_angles( // lower version of the set_joint_angle which does not
                          // clear cache
      const std::vector<size_t> &joint_ids,
      const std::vector<double> &joint_angles);

  void set_base_pose(double x, double y, double theta);
  void _set_base_pose(double x, double y, double theta);

  void clear_cache();

  void set_init_angles();

  std::vector<double>
  get_joint_angles(const std::vector<size_t> &joint_ids) const;

  std::vector<size_t> get_joint_ids(std::vector<std::string> joint_names) const;

  std::vector<AngleLimit>
  get_joint_limits(const std::vector<size_t> &joint_ids) const;

  std::vector<size_t> get_link_ids(std::vector<std::string> link_names) const;

  virtual void get_link_pose(size_t link_id, urdf::Pose &out_tf_root_to_ef,
                             bool usebase) const = 0;

  virtual Eigen::MatrixXd get_jacobian(size_t elink_id,
                                       const std::vector<size_t> &joint_ids,
                                       bool rotalso = false,
                                       bool basealso = false) = 0;

  void set_joint_angle(size_t joint_id, double angle) {
    joint_angles_[joint_id] = angle;
  }

  void add_new_link(std::string link_name, size_t parent_id,
                    std::array<double, 3> position,
                    std::array<double, 3> rotation);

private:
  void update_rptable();
};

class CacheUtilizedRobotModel : public RobotModelBase {
public:
  using RobotModelBase::RobotModelBase;

  void get_link_pose(size_t link_id, urdf::Pose &out_tf_root_to_ef,
                     bool usebase) const;

  Eigen::MatrixXd get_jacobian(size_t elink_id,
                               const std::vector<size_t> &joint_ids,
                               bool rotalso = false, bool basealso = false);

private:
  void get_link_pose_inner(size_t link_id, urdf::Pose &out_tf_root_to_ef,
                           bool usebase) const;
};

class NaiveRobotModel : public RobotModelBase {
public:
  using RobotModelBase::RobotModelBase;

  void get_link_pose(size_t link_id, urdf::Pose &out_tf_root_to_ef,
                     bool basealso) const;

  Eigen::MatrixXd get_jacobian(size_t elink_id,
                               const std::vector<size_t> &joint_ids,
                               bool rotalso = false, bool basealso = false);
};

std::string load_urdf(const std::string &urdf_path);
}; // namespace tinyfk

#endif // include guard
