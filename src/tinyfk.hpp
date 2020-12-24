/*
Copyright (c) 2020 Hirokazu Ishida
This software is released under the MIT License, see LICENSE.
tinyfk: https://github.com/HiroIshida/tinyfk
*/

#include "urdf_parser/urdf_parser.h"
#include "urdf_model/pose.h"
#include "urdf_model/joint.h"
#include <iostream>
#include <stdexcept>
#include <Eigen/Core> // slow compile...  
#include <array>
#include <stack>
#include <unordered_map>

namespace tinyfk
{

  struct TransformCache
  {
    int _N_link;
    std::vector<urdf::Pose> _data;
    std::vector<bool> _isCachedVec;

    TransformCache(){}
    TransformCache(unsigned int N_link) : 
      _N_link(N_link), _data(std::vector<urdf::Pose>(N_link)), 
      _isCachedVec(std::vector<bool>(N_link, false)) {}

    void set_cache(unsigned int link_id, const urdf::Pose& tf){
      bool isAlreadyCached = (_isCachedVec[link_id] == true);
      if(isAlreadyCached){throw std::invalid_argument("attempt to break cache");}
      _isCachedVec[link_id] = true;
      _data[link_id] = tf;
    }

    urdf::Pose* get_cache(unsigned int link_id){
      bool isAlreadyCached = (_isCachedVec[link_id] == true);
      if(!isAlreadyCached){return nullptr;} //the cache does not exists
      return &_data[link_id];
    }

    void extend(){
      _N_link++;
      _data.push_back(urdf::Pose());
      _isCachedVec.push_back(false);
      this->clear();
    }

    void clear(){std::fill(_isCachedVec.begin(), _isCachedVec.end(), false);}
  };

  struct NastyStack
  {
    std::vector<urdf::Pose> _tf_stack;
    std::vector<unsigned int> _hid_stack; // here id stack
    NastyStack(){};
    NastyStack(unsigned int N_link) : _tf_stack(std::vector<urdf::Pose>(N_link)), 
    _hid_stack(std::vector<unsigned int>(N_link)) {} 
  };


  struct RelevancePredicateTable
  {
    std::vector<std::vector<bool>> _table;
    RelevancePredicateTable(){};
    RelevancePredicateTable(int N_link, int N_joint){ 
      for(int i=0; i<N_joint; i++){
        _table.push_back(std::vector<bool>(N_link));
      }
    }
    bool isRelevant(int joint_id, int link_id) const{
      return _table[joint_id][link_id];
    }
  };

  struct BasePose
  {
    std::array<double, 3> _pose3d;
    urdf::Pose _pose;
    void set(double x, double y, double theta){
      urdf::Vector3&& pos = {x, y, 0.0};
      urdf::Rotation&& rot = {0.0, 0.0, 1.0 * sin(0.5 * theta), cos(0.5 * theta)};
      _pose.position = std::move(pos);
      _pose.rotation = std::move(rot);
      _pose3d[0] = x; _pose3d[1] = y; _pose3d[2] = theta;
    }
  };

  class RobotModel
  {
    public: //members
      std::string _urdf_file;
      // change them all to private later
      urdf::ModelInterfaceSharedPtr _robot_urdf_interface;

      urdf::LinkSharedPtr _root_link;
      std::vector<urdf::LinkSharedPtr> _links;
      std::unordered_map<std::string, int> _link_ids;

      std::vector<urdf::JointSharedPtr> _joints;
      std::unordered_map<std::string, int> _joint_ids;
      std::vector<double> _joint_angles;
      RelevancePredicateTable _rptable;
      BasePose _base_pose;
      int _num_dof;

      mutable NastyStack _nasty_stack; // TODO add constructor??
      mutable TransformCache _tf_cache;

    public: //functions
      RobotModel(const std::string& urdf_file);
      RobotModel(){}

      void get_link_point(
          unsigned int link_id, urdf::Pose& out_tf_root_to_ef, bool basealso) const;
      
      // naive jacobian computation with finite differentiation (just for testing)
      // as in the finite differentiatoin, set_joint_angle is called, this function cannot be
      // const-nized
      Eigen::MatrixXd get_jacobian_naive(
          unsigned int elink_id, const std::vector<unsigned int>& joint_ids,
          bool rotalso = false, bool basealso = false
          );

      void set_joint_angles(
          const std::vector<unsigned int>& joint_ids, const std::vector<double>& joint_angles);

      void set_init_angles();

      std::vector<double> get_joint_angles(const std::vector<unsigned int>& joint_ids) const;
      std::vector<unsigned int> get_joint_ids(std::vector<std::string> joint_names) const;
      std::vector<unsigned int> get_link_ids(std::vector<std::string> link_names) const;

      // private (I wanna make these function private, but 
      // don't know who to do unit test after that
      // anyway, don't use it
      void set_joint_angle(unsigned int joint_id, double angle){
        _joint_angles[joint_id] = angle;
      }

      void set_base_pose(double x, double y, double theta){_base_pose.set(x, y, theta);}
      
      void set_base_pose(const std::array<double ,3>& pose3d){
        _base_pose.set(pose3d[0], pose3d[1], pose3d[2]);
      }

      // perfromance of returning array of eigen is actually almost same as pass by reference
      // thanks to the compiler's optimization
      std::array<Eigen::MatrixXd, 2> get_jacobians_withcache(
          const std::vector<unsigned int>& elink_ids,
          const std::vector<unsigned int>& joint_ids, 
          bool rpyalso = false, // only point jacobian is computed by default
          bool basealso = false
          ) const;

      void get_link_point_withcache(
          unsigned int link_id, urdf::Pose& out_tf_root_to_ef, 
          bool usebase) const;

      void add_new_link(
          std::string link_name, 
          unsigned int parent_id,
          std::array<double, 3> position){

        bool link_name_exists = (_link_ids.find(link_name) != _link_ids.end());
        if(link_name_exists){
          std::string message = "link name " + link_name + " already exists";
          throw std::invalid_argument("link name : " + link_name + " already exists");
        }

        auto fixed_joint = std::make_shared<urdf::Joint>();
        auto&& vec = urdf::Vector3(position[0], position[1], position[2]);
        // only these two infomation is used in kinematics computation 
        fixed_joint->parent_to_joint_origin_transform.position = vec;
        fixed_joint->type = urdf::Joint::FIXED;

        int link_id = _links.size();
        auto new_link = std::make_shared<urdf::Link>();
        new_link->parent_joint = fixed_joint;
        new_link->setParent(_links[parent_id]);
        new_link->name = link_name;
        new_link->id = link_id;

        _link_ids[link_name] = link_id;
        _links.push_back(new_link);
        _links[parent_id]->child_links.push_back(new_link);
        _tf_cache.extend();

        this->_update_rptable(); // set _rptable
      }

    private:
      void _update_rptable(){
        // this function usually must come in the end of a function
        
        // we must recreate from scratch
        int n_link = _link_ids.size();
        int n_dof = _joint_ids.size();
        auto rptable = RelevancePredicateTable(n_link, n_dof);

        for(urdf::JointSharedPtr joint : _joints){
          int joint_id = _joint_ids.at(joint->name);
          urdf::LinkSharedPtr clink = joint->getChildLink();
          std::stack<urdf::LinkSharedPtr> link_stack;
          link_stack.push(clink);
          while(!link_stack.empty()){
            auto here_link = link_stack.top();
            link_stack.pop();
            rptable._table[joint_id][here_link->id] = true;
            for(auto& link : here_link->child_links){
              link_stack.push(link);
            }
          }
        }
        _rptable = rptable;
      }
  };

};//end namespace
