const mylib = joinpath(pwd(), "../build/libtinyfk_capi.so")

create_robot_model(str) = ccall((:capi_create_robot_model, mylib), Ptr{Cvoid}, (Cstring,), str)
get_link_id(ptr_robot, str) = ccall((:capi_get_link_id, mylib), Cuint, (Ptr{Cvoid}, Cstring,), ptr_robot, str)
get_joint_id(ptr_robot, str) = ccall((:capi_get_joint_id, mylib), Cuint, (Ptr{Cvoid}, Cstring,), ptr_robot, str)

hello() = ccall((:hello, mylib), Cvoid, ())

urdf_file = "../data/fetch_description/fetch.urdf"
ptr_robot = create_robot_model(urdf_file)

link_names = ["shoulder_pan_link", "shoulder_lift_link",
              "upperarm_roll_link", "elbow_flex_link",
              "forearm_roll_link", "wrist_flex_link",
              "wrist_roll_link"]

for link_name in link_names
    link_id = get_link_id(ptr_robot, link_name)
    println(link_id)
end

joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
for joint_name in joint_names
    joint_id = get_joint_id(ptr_robot, joint_name)
    println(joint_id)
end


