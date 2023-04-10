cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -Wall -fPIC")

add_subdirectory("urdf_parser")
include_directories("third_party" "urdf_parser/include")

find_package(Eigen3 REQUIRED)
ADD_DEFINITIONS(-DEIGEN_NO_DEBUG)
include_directories(${EIGEN3_INCLUDE_DIR})

set(TINYFK_SRC src/tinyfk.cpp src/kinematics.cpp)
include_directories(include/)
add_library(tinyfk STATIC ${TINYFK_SRC})
target_link_libraries(tinyfk urdfdom_model)

# testing
option(BUILD_TEST "build google test" OFF)
if(BUILD_TEST)
    find_package(GTest REQUIRED)

    function(setup_tinyfk_test test_name test_src)
        add_executable(${test_name} ${test_src})
        target_link_libraries(${test_name} tinyfk ${GTEST_LIBRARIES} pthread)
        target_include_directories(${test_name} PUBLIC ${GTEST_INCLUDE_DIRS})
    endfunction()

    setup_tinyfk_test(test_others test/test_others.cpp)
    setup_tinyfk_test(test_data_structure test/test_data_structure.cpp)
    setup_tinyfk_test(test_kinematics test/test_kinematics.cpp)

    set(nlohmann_json_INCLUDE_DIRS third_party/json/include)
    target_include_directories(test_kinematics PUBLIC ${nlohmann_json_INCLUDE_DIRS})

    enable_testing()
    gtest_discover_tests(test_data_structure)
    gtest_discover_tests(test_kinematics)
endif(BUILD_TEST)

# benchmark (kdl)
option(BENCH_KDL "benchmarking kdl" OFF)
if(BENCH_KDL)
    find_package(orocos_kdl REQUIRED)
    message(${orocos_kdl_INCLUDE_DIRS})

    add_library(kdl_parser STATIC bench/kdl_parser/kdl_parser.cpp)
    target_link_libraries(kdl_parser ${orocos_kdl_LIBRARIES})
    target_include_directories(kdl_parser PUBLIC ${orocos_kdl_INCLUDE_DIRS} bench/kdl_parser)

    add_executable(bench_kdl bench/bench_kdl.cpp)
    target_link_libraries(bench_kdl urdfdom_model ${orocos_kdl_LIBRARIES} kdl_parser)
    target_include_directories(bench_kdl PUBLIC ${orocos_kdl_INCLUDE_DIRS} bench/kdl_parser)
endif(BENCH_KDL)

# benchmark (tinyfk)
add_executable(bench_tinyfk bench/bench_tinyfk.cpp)
target_link_libraries(bench_tinyfk tinyfk)

option(BUILD_PYTHON_INTERFACE "Set when you want to build python interface" ON)
if(BUILD_PYTHON_INTERFACE)
    option(INSTALL_VIA_PIP "if you install via pip install . " ON)

    set(LIBRARY_NAME _tinyfk)
    add_subdirectory(third_party/pybind11)
    pybind11_add_module(_tinyfk src/wrapper.cpp)
    target_link_libraries(_tinyfk PRIVATE tinyfk)

    if(INSTALL_VIA_PIP)
        install(TARGETS _tinyfk DESTINATION .) # for pip install 
    else() # gdb friendly. 
        option(PYTHON_GLOBAL_SITE_PKG 
            "install into global site package. if OFF, or by default, will be installed into per-user site-packages" 
            OFF)
        if(PYTHON_GLOBAL_SITE_PKG)
            set(CMD_SITE_PKG "import site; import sys;sys.stdout.write(site.getsitepackages()[0])")
        else(PYTHON_GLOBAL_SITE_PKG)
            set(CMD_SITE_PKG "import site; import sys;sys.stdout.write(site.getusersitepackages())")
        endif(PYTHON_GLOBAL_SITE_PKG)
        execute_process(
            COMMAND ${PYTHON_EXECUTABLE} -c "${CMD_SITE_PKG}"
            OUTPUT_VARIABLE PYTHON_SITE_PACKAGES_INSTALL_DIR
            )
        install(TARGETS _tinyfk DESTINATION ${PYTHON_SITE_PACKAGES_INSTALL_DIR})
        message("destination site-packages:" ${PYTHON_SITE_PACKAGES_INSTALL_DIR})
    endif()
endif(BUILD_PYTHON_INTERFACE)
