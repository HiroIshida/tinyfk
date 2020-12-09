#ifdef __cplusplus
extern "C"{
#endif
  void* capi_create_robot_model(char* urdf_file_);
  unsigned int capi_get_link_id(void* robot_model_ptr, char* link_name);
  unsigned int capi_get_joint_id(void* robot_model_ptr, char* joint_name);
  void capi_set_joint_angle(void* robot_model_, unsigned int joint_id, double angle);
  void hello();
#ifdef __cplusplus
}
#endif
