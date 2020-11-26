cmake_minimum_required(VERSION 3.4 FATAL_ERROR)

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
include_directories(src/)
add_library(tinyfk STATIC ${TINYFK_SRC})
target_link_libraries(tinyfk urdfdom_model)

# testing
set(nlohmann_json_INCLUDE_DIRS third_party/json/include)
add_executable(test_kinematics test/test.cpp)
target_link_libraries(test_kinematics tinyfk)
target_include_directories(test_kinematics PRIVATE ${nlohmann_json_INCLUDE_DIRS})

# bench
find_package(orocos_kdl REQUIRED)
message(${orocos_kdl_INCLUDE_DIRS})
include_directories(${orocos_kdl_INCLUDE_DIRS})
add_library(kdl_parser bench/kdl_parser.cpp)
add_executable(bench bench/bench.cpp)
target_link_libraries(bench urdfdom_model ${orocos_kdl_LIBRARIES} kdl_parser tinyfk)

option(BUILD_PYTHON_INTERFACE "Set when you want to build python interface" ON)
if(BUILD_PYTHON_INTERFACE)
    option(INSTALL_VIA_PIP "if you install via pip install . " ON)

    set(LIBRARY_NAME _tinyfk)
    add_subdirectory(third_party/pybind11)
    pybind11_add_module(_tinyfk python/wrapper.cpp)
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
