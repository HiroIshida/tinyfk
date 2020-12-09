const mylib = joinpath(pwd(), "../build/libtinyfk_capi.so")

cfunc(str) = ccall((:c_create_robotic_tree, mylib), Ptr{Cvoid}, (Cstring,), str)
hello() = ccall((:hello, mylib), Cvoid, ())

urdf_file = "../data/fetch_description/fetch.urdf"
ptr_robot = cfunc(urdf_file)
