cmake_minimum_required(VERSION 3.5)
project(azureloong_control)
set(CMAKE_BUILD_TYPE Release) 
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    genmsg
    roscpp
    roslib
    rospy
    sensor_msgs
    nav_msgs
    geometry_msgs 
    tf
)

catkin_package(
  CATKIN_DEPENDS roscpp
)

set(dirEigen "third_party/eigen3")
set(dirGlfw "third_party/glfw")
set(dirPino "third_party/pinocchio")
set(dirJson "third_party/jsoncpp")
set(dirQuill "third_party/quill")
set(dirQP "third_party/qpOASES")

set(libURDF "third_party/urdfdom")

set(incFmt "third_party/quill/quill/bundled")
set(incBoost "third_party/boost")
set(incMujoco "third_party/mujoco")

set(allDir ${dirEigen} ${dirGlfw} ${dirPino} ${dirJson} ${dirQuill} ${dirQP} )
set(allLib ${allDir} ${libURDF})
set(allInc ${allDir} ${incFmt} ${incBoost} ${incMujoco})

include_directories(${allInc})
include_directories("algorithm")
include_directories("common")
include_directories("math")
include_directories("sim_interface")
include_directories(${catkin_INCLUDE_DIRS})

list(APPEND allLib "third_party/mujoco/lin_x64")
link_directories(${allLib})

file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp algorithm/*.cpp common/*.cpp math/*.cpp sim_interface/*.cpp)
file(GLOB HEADER_FILES *.h algorithm/*.h common/*.h math/*.h sim_interface/*.h)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES} ${HEADER_FILES})

set(corLibs pinocchio urdfdom_model tinyxml console_bridge jsoncpp quill qpOASES)
set(simLibs glfw3) 
set(sysCoreLibs)
set(sysSimLibs)

foreach(lib ${corLibs})
    list(APPEND sysCoreLibs ${lib}_lin_x64)
endforeach()
foreach(lib ${simLibs})
    list(APPEND sysSimLibs ${lib}_lin_x64)
endforeach()

message(${sysCoreLibs})

add_library(core ${SOURCES})
target_link_libraries(core ${sysCoreLibs} pthread)

add_executable(azureloong_ctrl src/azureloong_ctrl.cpp)
target_link_libraries(azureloong_ctrl core mujoco ${sysSimLibs} ${catkin_LIBRARIES} dl)

