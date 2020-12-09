const mylib = joinpath(pwd(), "../build/libtinyfk_capi.so")
c_create_robot_model(str) = ccall((:capi_create_robot_model, mylib), Ptr{Cvoid}, (Cstring,), str)
c_clear_cache(ptr_robot) = ccall((:capi_clear_cache, mylib), Cvoid, (Ptr{Cvoid},), ptr_robot) 
c_get_link_id(ptr_robot, str) = ccall((:capi_get_link_id, mylib), Cuint, (Ptr{Cvoid}, Cstring,), ptr_robot, str)
c_get_joint_id(ptr_robot, str) = ccall((:capi_get_joint_id, mylib), Cuint, (Ptr{Cvoid}, Cstring,), ptr_robot, str)
c_set_joint_angle(ptr_robot, joint_id, angle) = ccall((:capi_set_joint_angle, mylib), Cvoid, (Ptr{Cvoid}, Cuint, Cdouble), ptr_robot, joint_id, angle)

c_get_link_point(ptr_robot, link_id, with_rot, with_base, pose) = ccall((:capi_get_link_point, mylib), 
                                                         Cvoid, 
                                                         (Ptr{Cvoid}, Cuint, Cuchar, Cuchar, Ptr{Cdouble}), 
                                                         ptr_robot, link_id, with_rot, with_base, pose
                                                         )
struct TinyfkSolver
    ptr_robot::Ptr{Cvoid}
    function TinyfkSolver(urdf_path)
        ptr_robot = c_create_robot_model(urdf_path)
        new(ptr_robot)
    end
end
get_link_id(fksolver::TinyfkSolver, link_name) = c_get_link_id(fksolver.ptr_robot, link_name)
get_joint_id(fksolver::TinyfkSolver, joint_name) = c_get_joint_id(fksolver.ptr_robot, joint_name)
set_joint_angle(fksolver::TinyfkSolver, joint_id, angle) = c_set_joint_angle(fksolver.ptr_robot, joint_id, angle)
get_link_point(fksolver::TinyfkSolver, link_id, with_rot, with_base, pose::AbstractArray{Float64, 1}) = c_get_link_point(fksolver.ptr_robot, link_id, with_rot, with_base, pose)
clear_cache(fksolver::TinyfkSolver) = c_clear_cache(fksolver.ptr_robot)

function test()
    urdf_file = "../data/fetch_description/fetch.urdf"
    fksolver = TinyfkSolver(urdf_file)

    link_names = [
        "l_gripper_finger_link", 
        "r_gripper_finger_link", 
        "wrist_flex_link",
        "wrist_roll_link",
        "shoulder_lift_link",
        "upperarm_roll_link"]

    joint_names = [
        "torso_lift_joint",
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "upperarm_roll_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint"];

    joint_ids::Array{UInt32, 1} = [get_joint_id(fksolver, jname) for jname in joint_names]
    link_ids::Array{UInt32, 1} = [get_link_id(fksolver, lname) for lname in link_names]
    P = zeros(Float64, 3, 6)
    for i in 1:1000000
        for jid in joint_ids
            set_joint_angle(fksolver, jid, -0.4)
        end
        clear_cache(fksolver)

        for idx in 1:6
            get_link_point(fksolver, link_ids[idx], false, false, @view P[:, idx])
        end
    end
end
@time test()
