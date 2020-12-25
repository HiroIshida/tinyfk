#include "tinyfk.hpp"

namespace tinyfk
{

  int rotation_dim(int rotation_mode){
    switch(rotation_mode){
      case NO_ROTATION:
        return 0;
      case RPY:
        return 3;
      case YPR:
        return 3;
      case XYZW:
        return 4;
      case WXYZ:
        return 4;
    }
  }

  void copy_pose_to_arr(const urdf::Pose& pose, TinyMatrix& arr, int rotation_mode)
  {
    const urdf::Vector3& pos = pose.position;
    arr[0] = pos.x;
    arr[1] = pos.y;
    arr[2] = pos.z;
    if(rotation_mode != NO_ROTATION){
      const urdf::Rotation& rot = pose.rotation;
      switch(rotation_mode){
        case RPY:
          {
            urdf::Vector3 rpy = rot.getRPY();
            arr[3] = rpy.x;
            arr[4] = rpy.y;
            arr[5] = rpy.z;
            return;
          }
        case YPR:
          {
            urdf::Vector3 rpy = rot.getRPY();
            arr[3] = rpy.z;
            arr[4] = rpy.y;
            arr[5] = rpy.x;
            return;
          }
        case XYZW:
          {
            arr[3] = rot.x;
            arr[4] = rot.y;
            arr[5] = rot.z;
            arr[6] = rot.w;
            return;
          }
        case WXYZ:
          {
            arr[3] = rot.x;
            arr[4] = rot.y;
            arr[5] = rot.z;
            arr[6] = rot.w;
            return;
          }
      }
    }
  }

}

