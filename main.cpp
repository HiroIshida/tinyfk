#include "core.hpp"
#include <time.h>
#include "unistd.h"

#define print(a) std::cout<<a<<std::endl
#define RESET   "\033[0m"
#define GREEN   "\033[32m"      /* Green */
using namespace std;
using namespace Eigen;
using uint = unsigned int;

std::string red_color(const std::string& s){
  return "\033[1;31m " + s + "\033[0m ";
}

void failure_throw(const std::string& s){
  auto message = red_color("[TEST FAIL] " + s);
  throw invalid_argument(message);
}

void show_pose(const urdf::Pose& pose){
  auto& pos = pose.position;
  auto& rot = pose.rotation;
  std::cout << "xyz: " << pos.x << " " << pos.y << " " << pos.z  << 
    "rpy: " << rot.x << " " << rot.y << " " << rot.z << " " << rot.w << std::endl;
}

int main(int argc, char** argv)
{
  print("======================================");
  // common
  std::string urdf_file = "../data/fetch_description/fetch.urdf";
  auto a = RobotModel(urdf_file);


  print("======================================");
  print("check quaternion inv");
  {
    urdf::Rotation rot, rot_inv;
    rot.setFromRPY(1.0, 2.0, 3.0);
    rot_inv = rot.inverse();
    auto rot_new = rot_inv * rot;
    std::cout << rot_new.x << " " << rot_new.y << " " << rot_new.z << " " << rot_new.w << std::endl; 
  }


  print("======================================");
  print("check pose (Vector3 & Rotation) inplace inverse operation");
  {
    urdf::Pose pose;
    auto& pos = pose.position;
    pos.x = 1.0; pos.y = 2.0, pos.z = 3.0;
    pose.rotation.setFromRPY(1.0, 2.0, 3.0);
    auto pose_inv = pose;
    pose_inv.inverse_inplace();
    auto pose_unit = pose_transform(pose_inv, pose);
    auto& pos_unit = pose_unit.position;
    auto& rot_unit = pose_unit.rotation;
    print(pos_unit.x << " " << pos_unit.y << " " << pos_unit.z);
    print(rot_unit.x << " " << rot_unit.y << " " << rot_unit.z << " " << rot_unit.w);
  }

  print("======================================");
  print("check cross product (expect to be [0, 0, 0], [-3, 6, -3])");
  {
    urdf::Vector3 a = {1., 2., 3.};
    auto b = a;
    urdf::Vector3 out;
    cross_product(a, b, out);
    print(out.x << " " << out.y << " " << out.z);

    urdf::Vector3 c = {4., 5., 6.};
    cross_product(a, c, out);
    print(out.x << " " << out.y << " " << out.z);
  }

  print("======================================");
  print("check rot * vec");
  {
    urdf::Vector3 a = {1., 2., 3.};
    urdf::Rotation rot;
    rot.setFromRPY(0, 0, 3.1415926);
    auto b = rot * a;
    print("?????????????");
  }

  print("======================================");
  print("test joint_id and joints");
  {
    for(auto& joint : a._joints){
      auto id = a._joint_ids[joint->name];
      auto joint_result = a._joints[id];
      if(joint->name != joint_result->name){
        failure_throw("must match");
      }
    }
  }

  

  print("======================================");
  print("test get_link_ids & get_joint_ids");
  {
    {
      vector<string> link_names = {"gripper_link", "unko_link"};
      try{
        a.get_link_ids(link_names);
        failure_throw("supporsed to rise error");
      }catch(const std::exception& e){
        std::cout << "link ok" << std::endl; 
      }
    }

    {
      vector<string> joint_names = {"torso_lift_joint", "unko_joint"};
      try{
        a.get_joint_ids(joint_names);
        failure_throw("supporsed to rise error");
      }catch(const std::exception& e){
        std::cout << "joint ok" << std::endl; 
      }
    }

    {
      vector<string> joint_names = {"torso_lift_joint", "shoulder_pan_joint"};
      auto ids = a.get_joint_ids(joint_names);
      for(uint i=0; i<ids.size(); i++){
        const auto& name_obtained = a._joints[ids[i]]->name;
        if(name_obtained != joint_names[i]){
          failure_throw(name_obtained + " must matches with " + joint_names[i]);
        }
      }
      print("[OK]");
    }
  }
  {

      auto robot = a;
      vector<string> names = {"torso_lift_joint","shoulder_pan_joint","shoulder_lift_joint","upperarm_roll_joint","elbow_flex_joint","forearm_roll_joint","wrist_flex_joint","wrist_roll_joint"};
      auto joint_ids = robot.get_joint_ids(names);
      vector<double> av = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
      for(uint i=0; i<av.size(); i++){
        robot.set_joint_angle(joint_ids[i], av[i]);
      }
      urdf::Pose pose_naive, pose_better;
      int gripper_id = robot._link_ids["gripper_link"];

    robot.get_link_point(gripper_id, pose_naive, true);
    robot.get_link_point_withcache(gripper_id, pose_better, true);
    show_pose(pose_naive);
    show_pose(pose_better);

    robot.set_base_pose(0.3, 0.3, 0.3);

    print("naive:");
    MatrixXd J = robot.get_jacobian_naive(gripper_id, joint_ids, true, true);
    print(J);
    print("better:");
    robot._tf_cache.clear();
    std::vector<unsigned int> elink_ids = {gripper_id};
    auto tmp = robot.get_jacobians_withcache(elink_ids, joint_ids, true, true);
    auto J_ = tmp[0];
    print(J_);
  }

}
