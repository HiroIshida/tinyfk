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

using MatrixXdC =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
using MatrixXdR =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using AngleLimit = std::pair<double, double>;

// TODO templatize
// an util data structure to handle matrix.
struct SlicedMatrix // coll major (same as eigen)
{
  double *data_; // beginning of the block matrix
  int i_begin_;
  int j_begin_;

  int n_block_;
  int m_block_;

  int n_whole_;
  int m_whole_;

  SlicedMatrix(Eigen::MatrixXd &mat, int i_begin, int j_begin, int n, int m)
      : data_(mat.data()), i_begin_(i_begin), j_begin_(j_begin), n_block_(n),
        m_block_(m), n_whole_(mat.rows()), m_whole_(mat.cols()) {}

  SlicedMatrix(Eigen::MatrixXd &mat)
      : data_(mat.data()), i_begin_(0), j_begin_(0), n_block_(mat.rows()),
        m_block_(mat.cols()), n_whole_(n_block_), m_whole_(m_block_) {}

  SlicedMatrix(double *data, int i_begin, int j_begin, int n, int m,
               int n_whole, int m_whole)
      : data_(data), i_begin_(i_begin), j_begin_(j_begin), n_block_(n),
        m_block_(m), n_whole_(n_whole), m_whole_(m_whole) {}

  inline int rows() const { return n_block_; }

  inline int cols() const { return m_block_; }

  inline int get_idx(int i, int j) const {
    assert(i < n_whole_ && "out of index");
    assert(j < m_whole_ && "out of index");

    int idx = n_whole_ * (j + j_begin_) + (i + i_begin_);
    return idx;
  }

  SlicedMatrix block(int i, int j, int n, int m) const {
    SlicedMatrix mat = {data_, i_begin_ + i, j_begin_ + j, n,
                        m,     n_whole_,     m_whole_};
    return mat;
  }

  SlicedMatrix slice(int i) const { // we consider matrix is coll major.
    return this->block(0, i, n_block_, 1);
  }

  double &operator()(int i, int j) const { return data_[this->get_idx(i, j)]; }

  double &operator[](int i) const { // access to sliced matrix
    return data_[this->get_idx(i, 0)];
  }
};

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

class RobotModel {
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
  RobotModel(const std::string &xml_string);
  RobotModel() {}

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

  // private (I wanna make these function private, but
  // don't know who to do unit test after that
  // anyway, don't use it
  void set_joint_angle(size_t joint_id, double angle) {
    joint_angles_[joint_id] = angle;
  }

  // perfromance of returning array of eigen is actually almost same as pass by
  // reference thanks to the compiler's optimization
  std::array<Eigen::MatrixXd, 2> get_jacobians_withcache(
      const std::vector<size_t> &elink_ids,
      const std::vector<size_t> &joint_ids,
      bool rpyalso = false, // only point jacobian is computed by default
      bool basealso = false) const;

  void solve_forward_kinematics(int elink_id,
                                const std::vector<size_t> &joint_ids,
                                bool with_rot, bool with_base,
                                SlicedMatrix &pose_arr,
                                SlicedMatrix &jacobian) const;

  void solve_batch_forward_kinematics(std::vector<size_t> elink_ids,
                                      const std::vector<size_t> &joint_ids,
                                      bool with_rot, bool with_base,
                                      SlicedMatrix &pose_arr,
                                      SlicedMatrix &jacobian_arr) const;

  void get_link_pose(size_t link_id, urdf::Pose &out_tf_root_to_ef,
                     bool usebase) const;

  void get_link_pose_naive(size_t link_id, urdf::Pose &out_tf_root_to_ef,
                           bool basealso) const;

  Eigen::MatrixXd get_jacobian_naive(size_t elink_id,
                                     const std::vector<size_t> &joint_ids,
                                     bool rotalso = false,
                                     bool basealso = false);

  void add_new_link(std::string link_name, size_t parent_id,
                    std::array<double, 3> position,
                    std::array<double, 3> rotation);

private:
  void get_link_pose_inner(size_t link_id, urdf::Pose &out_tf_root_to_ef,
                           bool usebase) const;
  void update_rptable();
};

RobotModel construct_from_urdfpath(const std::string &urdf_path);
}; // namespace tinyfk

#endif // include guard
