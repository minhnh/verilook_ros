cmake_minimum_required(VERSION 2.8.3)
project(verilook_ros)

set(CMAKE_BUILD_TYPE Debug)
set(NODELET_LIBRARY face_recognition_verilook_nodelet)
set(NODE_EXEC_FACE_RECOGNITION face_recognition_verilook_node)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    nodelet
    sensor_msgs
    std_msgs
    roslint
    message_generation
    image_transport
    ${catkin_BUILD_PACKAGES}
)

## Additional header files
include_directories(
    ros/include
    ${catkin_INCLUDE_DIRS}
)

## Available service files in the 'srv' folder
add_service_files(
    FILES
        CreateTemplate.srv
)
generate_messages(DEPENDENCIES
    std_msgs 
    sensor_msgs
)

catkin_package(
    LIBRARIES
        face_recognition_verilook
    CATKIN_DEPENDS
        nodelet
        roscpp
        message_runtime
        std_msgs
        sensor_msgs
)



## Create the nodelet tutorial library
add_library(${NODELET_LIBRARY}
    ros/src/face_recognition_verilook_nodelet.cpp
    ros/src/face_recognition_verilook_node.cpp
    ros/src/verilook_wrapper.cpp
    ros/src/verilook_enroll_from_image.cpp
)

add_dependencies(${NODELET_LIBRARY} 
   ${PROJECT_NAME}_gencpp
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${NODELET_LIBRARY}
    -lNBiometricClient
    -lNBiometrics
    -lNLicensing
    -lNCore
    ${catkin_LIBRARIES}
)

add_executable(${NODE_EXEC_FACE_RECOGNITION}
    ros/src/face_recognition_verilook_node.cpp
    ros/src/verilook_wrapper.cpp
    ros/src/verilook_enroll_from_image.cpp
)

add_dependencies(${NODE_EXEC_FACE_RECOGNITION} 
    ${PROJECT_NAME}_gencpp
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${NODE_EXEC_FACE_RECOGNITION}
    -lNBiometricClient
    -lNBiometrics
    -lNLicensing
    -lNCore
    -lNMedia
    ${catkin_LIBRARIES}
)


## Mark the nodelet library for installations
install(TARGETS ${NODELET_LIBRARY}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
    FILES
        nodelet_face_recognition_verilook.xml
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
    DIRECTORY
        ros/launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

# Checking convention
roslint_cpp()
roslint_add_test()

### TESTS
if(CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)
    roslaunch_add_file_check(ros/launch)
endif()
