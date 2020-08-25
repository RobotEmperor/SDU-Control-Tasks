CMAKE_MINIMUM_REQUIRED ( VERSION 2.8 )

PROJECT ( "belt_task" )

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)

set(RW_ROOT "$ENV{RW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")
find_package(PkgConfig)

pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)



INCLUDE_DIRECTORIES (${CMAKE_SOURCE_DIR}/include
                     /opt/ros/melodic/include
                     /usr/include/xenomai
                     ${EIGEN3_INCLUDE_DIR}
                     ${ROBWORK_INCLUDE_DIR}
                     ${yaml_cpp_INCLUDE_DIRS}
                     /home/yik/sdu_ws/include
                     )
                     


execute_process(COMMAND xeno-config --skin=alchemy --cflags OUTPUT_VARIABLE XENO_ALCHENMY_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND xeno-config --skin=alchemy --ldflags OUTPUT_VARIABLE XENO_ALCHENMY_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${XENO_ALCHENMY_CFLAGS} ${XENO_ALCHENMY_LDFLAGS}")


set( SOURCE_FILES 
  src/data_logging.cpp
  src/task_motion.cpp
  src/ros_node.cpp
  src/main.cpp
  src/tool_estimation.cpp
  src/task_robot.cpp
)

link_directories(${CMAKE_SOURCE_DIR}/lib
                 /home/yik/sdu_ws/lib
                 /opt/ros/melodic/lib
                 /usr/lib)


add_executable(belt_task ${SOURCE_FILES})
target_link_libraries(belt_task rtde sensor_filter sdu_math rt roscpp rosconsole roscpp_serialization boost_system rostime ${ROBWORK_LIBRARIES} ${yaml_cpp_LIBRARIES})
