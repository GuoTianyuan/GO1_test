cmake_minimum_required(VERSION 3.10)
project(legged_unitree_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_hw
        )

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        legged_hw
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        # $ENV{UNITREE_LEGGED_SDK_PATH}/include
)
# link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
link_directories(lib)
# define robot types
set(SOURCE src/legged_unitree_hw.cpp src/UnitreeHW.cpp)
# add_compile_definitions(ROBOT_TYPE_A1=123)
# add_compile_definitions(ROBOT_TYPE_GO1=456)
# add_compile_definitions(ROBOT_TYPE_ALIENGO=789)

# if ("$ENV{ROBOT_TYPE}" STREQUAL "a1")
#         add_compile_definitions(ROBOT_TYPE=123)
#         link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
#         if ("$ENV{UNITREE_PLATFORM}" STREQUAL "amd64")
#                 set(EXTRA_LIBS libunitree_legged_sdk_amd64.so lcm)
#         endif()

#         if("$ENV{UNITREE_PLATFORM}" STREQUAL "arm64")
#                 set(EXTRA_LIBS libunitree_legged_sdk_arm64.so lcm)
#         endif()
# endif()

# if("$ENV{ROBOT_TYPE}" STREQUAL "go1")
#         add_compile_definitions(ROBOT_TYPE=456)
#         link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
#         if ("$ENV{UNITREE_PLATFORM}" STREQUAL "amd64")
#                 set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)
#         endif()
#         if ("$ENV{UNITREE_PLATFORM}" STREQUAL "arm64")
#                 set(EXTRA_LIBS -pthread libunitree_legged_sdk_arm64.so lcm)
#         endif()
# endif()

# if($ENV{ROBOT_TYPE} STREQUAL "go1")
#         add_compile_definitions(ROBOT_TYPE=456)
#         if ($ENV{UNITREE_PLATFORM} STREQUAL "amd64")
#                 link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib/cpp/amd64)
#                 set(EXTRA_LIBS -pthread libunitree_legged_sdk.a lcm)
#         endif()
#         if ($ENV{UNITREE_PLATFORM} STREQUAL "arm64")
#                 link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib/cpp/arm64)
#                 set(EXTRA_LIBS -pthread libunitree_legged_sdk.a lcm)
#         endif()
# endif()

## Declare cpp executables
add_executable(${PROJECT_NAME}
        ${SOURCE}
        )

target_compile_definitions(${PROJECT_NAME} PRIVATE UNITREE_SDK)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        libunitree_legged_sdk_amd64.so lcm
        )
# add_executable(${PROJECT_NAME}
#         src/legged_unitree_hw.cpp
#         src/UnitreeHW.cpp
#         )

## Specify libraries to link executable targets against
# target_link_libraries(${PROJECT_NAME}
#         ${catkin_LIBRARIES}
#         ${EXTRA_LIBS}
#         libunitree_legged_sdk_amd64.so  lcm
#         )

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CF_WERROR
    )

endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(DIRECTORY config launch lib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
