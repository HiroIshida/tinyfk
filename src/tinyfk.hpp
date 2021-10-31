/*
Copyright (c) 2020 Hirokazu Ishida
This software is released under the MIT License, see LICENSE.
tinyfk: https://github.com/HiroIshida/tinyfk
*/

#ifndef tinyfk_hpp
#define tinyfk_hpp

#include "urdf_parser/urdf_parser.h"
#include "urdf_model/pose.h"
#include "urdf_model/joint.h"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <Eigen/Core> // slow compile...  
#include <array>
#include <stack>
#include <unordered_map>
#include <assert.h>

namespace tinyfk
{

  using MatrixXdC = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
  using MatrixXdR = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  // TODO templatize
  // an util data structure to handle matrix. 
  struct TinyMatrix // coll major (same as eigen)
  {
    double* data_; // beginning of the block matrix
    int i_begin_;
    int j_begin_;

    int n_block_;
    int m_block_;

    int n_whole_;
    int m_whole_;

    TinyMatrix(Eigen::MatrixXd& mat, int i_begin, int j_begin, int n, int m) : 
      data_(mat.data()),
      i_begin_(i_begin), j_begin_(j_begin),
      n_block_(n), m_block_(m),
      n_whole_(mat.rows()), m_whole_(mat.cols()) {}

    TinyMatrix(Eigen::MatrixXd& mat) :
      data_(mat.data()),
      i_begin_(0), j_begin_(0),
      n_block_(mat.rows()), m_block_(mat.cols()),
      n_whole_(n_block_), m_whole_(m_block_) {}

    TinyMatrix(double* data, int i_begin, int j_begin, int n, int m, int n_whole, int m_whole) :
      data_(data), i_begin_(i_begin), j_begin_(j_begin), n_block_(n), m_block_(m), 
      n_whole_(n_whole), m_whole_(m_whole) {}

    inline int rows(){
      return n_block_;
    }

    inline int cols(){
      return m_block_;
    }

    inline int get_idx(int i, int j){
      assert(i<n_whole_ && "out of index");
      assert(j<m_whole_ && "out of index");

      int idx = n_whole_ * (j+j_begin_) + (i+i_begin_);
      return idx;
    }

    TinyMatrix block(int i, int j, int n, int m){
      TinyMatrix mat = {data_, i_begin_+i, j_begin_+j, n, m, n_whole_, m_whole_};
      return mat;
    }

    TinyMatrix slice(int i){ // we consider matrix is coll major. 
      return this->block(0, i, n_block_, 1);
    }

    double& operator() (int i, int j){
      return data_[this->get_idx(i, j)];
    }

    double& operator[] (int i){ // access to sliced matrix
      return data_[this->get_idx(i, 0)];
    }
  };

  struct TransformCache
  {
    int N_link_;
    std::vector<urdf::Pose> data_;
    std::vector<bool> isCachedVec_;

    TransformCache(){}
    TransformCache(size_t N_link) : 
      N_link_(N_link), data_(std::vector<urdf::Pose>(N_link)), 
      isCachedVec_(std::vector<bool>(N_link, false)) {}

    void set_cache(size_t link_id, const urdf::Pose& tf){
      assert(!isCachedVec_[link_id] && "attempt to break an existing cache");
      isCachedVec_[link_id] = true;
      data_[link_id] = tf;
    }

    urdf::Pose* get_cache(size_t link_id){
      bool isAlreadyCached = (isCachedVec_[link_id] == true);
      if(!isAlreadyCached){return nullptr;} //the cache does not exists
      return &data_[link_id];
    }

    void extend(){
      N_link_++;
      data_.push_back(urdf::Pose());
      isCachedVec_.push_back(false);
      this->clear();
    }

    void clear(){// performance critical
      // bool's default value is false.
      isCachedVec_ = std::vector<bool>(N_link_);
    }
  };

  struct NastyStack
  {
    std::vector<urdf::Pose> tf_stack_;
    std::vector<size_t> hid_stack_; // here id stack
    NastyStack(){};
    NastyStack(size_t N_link) : tf_stack_(std::vector<urdf::Pose>(N_link)), 
    hid_stack_(std::vector<size_t>(N_link)) {} 
  };


  struct RelevancePredicateTable
  {
    std::vector<std::vector<bool>> table_;
    RelevancePredicateTable(){};
    RelevancePredicateTable(int N_link, int N_joint){ 
      for(int i=0; i<N_joint; i++){
        table_.push_back(std::vector<bool>(N_link));
      }
    }
    bool isRelevant(int joint_id, int link_id) const{
      return table_[joint_id][link_id];
    }
  };

  struct BasePose
  {
    std::array<double, 3> pose3d_;
    urdf::Pose pose_;
    void set(double x, double y, double theta){
      pose_.position = urdf::Vector3(x, y, 0.0);
      pose_.rotation = urdf::Rotation(0.0, 0.0, 1.0 * sin(0.5 * theta), cos(0.5 * theta));
      pose3d_[0] = x; pose3d_[1] = y; pose3d_[2] = theta;
    }
  };

  class RobotModel
  {
    public: //members
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

      mutable NastyStack nasty_stack_; // TODO add constructor??
      mutable TransformCache tf_cache_;

    public: //functions
      RobotModel(const std::string& xml_string);
      RobotModel(){}

      void get_link_point(
          size_t link_id, urdf::Pose& out_tf_root_to_ef, bool basealso) const;
      
      // naive jacobian computation with finite differentiation (just for testing)
      // as in the finite differentiatoin, set_joint_angle is called, this function cannot be
      // const-nized
      Eigen::MatrixXd get_jacobian_naive(
          size_t elink_id, const std::vector<size_t>& joint_ids,
          bool rotalso = false, bool basealso = false
          );

      void set_joint_angles(// this will clear all the cache stored
          const std::vector<size_t>& joint_ids, const std::vector<double>& joint_angles);
      void _set_joint_angles(// lower version of the set_joint_angle which does not clear cache
          const std::vector<size_t>& joint_ids, const std::vector<double>& joint_angles);

      void set_base_pose(double x, double y, double theta);
      void _set_base_pose(double x, double y, double theta);
      

      void clear_cache();

      void set_init_angles();

      std::vector<double> get_joint_angles(const std::vector<size_t>& joint_ids) const;
      std::vector<size_t> get_joint_ids(std::vector<std::string> joint_names) const;
      std::vector<size_t> get_link_ids(std::vector<std::string> link_names) const;

      // private (I wanna make these function private, but 
      // don't know who to do unit test after that
      // anyway, don't use it
      void set_joint_angle(size_t joint_id, double angle){
        joint_angles_[joint_id] = angle;
      }

      // perfromance of returning array of eigen is actually almost same as pass by reference
      // thanks to the compiler's optimization
      std::array<Eigen::MatrixXd, 2> get_jacobians_withcache(
          const std::vector<size_t>& elink_ids,
          const std::vector<size_t>& joint_ids, 
          bool rpyalso = false, // only point jacobian is computed by default
          bool basealso = false
          ) const;

      void _solve_forward_kinematics(
          int elink_id, const std::vector<size_t>& joint_ids,
          bool with_rot, bool with_base, TinyMatrix& pose_arr, TinyMatrix& jacobian) const;

      void _solve_batch_forward_kinematics(
          std::vector<size_t> elink_ids, const std::vector<size_t>& joint_ids,
          bool with_rot, bool with_base, TinyMatrix& pose_arr, TinyMatrix& jacobian_arr) const;

      void get_link_point_withcache(
          size_t link_id, urdf::Pose& out_tf_root_to_ef, 
          bool usebase) const;

      void _get_link_point_creating_cache(
          size_t link_id, urdf::Pose& out_tf_root_to_ef, 
          bool usebase) const;

      void add_new_link(
          std::string link_name, 
          size_t parent_id,
          std::array<double, 3> position){

        bool link_name_exists = (link_ids_.find(link_name) != link_ids_.end());
        if(link_name_exists){
          std::string message = "link name " + link_name + " already exists";
          throw std::invalid_argument("link name : " + link_name + " already exists");
        }

        auto fixed_joint = std::make_shared<urdf::Joint>();
        auto&& vec = urdf::Vector3(position[0], position[1], position[2]);
        // only these two infomation is used in kinematics computation 
        fixed_joint->parent_to_joint_origin_transform.position = vec;
        fixed_joint->type = urdf::Joint::FIXED;

        int link_id = links_.size();
        auto new_link = std::make_shared<urdf::Link>();
        new_link->parent_joint = fixed_joint;
        new_link->setParent(links_[parent_id]);
        new_link->name = link_name;
        new_link->id = link_id;

        link_ids_[link_name] = link_id;
        links_.push_back(new_link);
        links_[parent_id]->child_links.push_back(new_link);
        tf_cache_.extend();

        this->_update_rptable(); // set _rptable
      }

    private:
      void _update_rptable(){
        // this function usually must come in the end of a function
        
        // we must recreate from scratch
        int n_link = link_ids_.size();
        int n_dof = joint_ids_.size();
        auto rptable = RelevancePredicateTable(n_link, n_dof);

        for(urdf::JointSharedPtr joint : joints_){
          int joint_id = joint_ids_.at(joint->name);
          urdf::LinkSharedPtr clink = joint->getChildLink();
          std::stack<urdf::LinkSharedPtr> link_stack;
          link_stack.push(clink);
          while(!link_stack.empty()){
            auto here_link = link_stack.top();
            link_stack.pop();
            rptable.table_[joint_id][here_link->id] = true;
            for(auto& link : here_link->child_links){
              link_stack.push(link);
            }
          }
        }
        rptable_ = rptable;
      }
  };

  RobotModel construct_from_urdfpath(const std::string& urdf_path);
};//end namespace

#endif // include guard
