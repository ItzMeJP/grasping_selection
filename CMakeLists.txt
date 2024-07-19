cmake_minimum_required(VERSION 3.24)
project(grasping_selection
        VERSION 28.12.2022
        DESCRIPTION
        "Package to build grasping_selection pipeline")

# Set if it will have export lib capabilities
# If the lib is placed inside the project folder, it nos needed to install it.

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 14)
set (INSTALL_GRASPING_SELECTION_IN_ROOT True)


#External packages
find_package(Boost COMPONENTS date_time system REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED) # To matrix operations
find_package(orocos_kdl REQUIRED) # To robot kinematics
find_package(PCL REQUIRED) # To pointcloud operations


add_library(transform_manipulation SHARED IMPORTED GLOBAL)
set_property(TARGET transform_manipulation PROPERTY IMPORTED_LOCATION "/usr/local/lib/transform_manipulation/libtransform_manipulation.so")

add_library(std_vector_complement SHARED IMPORTED GLOBAL)
set_property(TARGET std_vector_complement PROPERTY IMPORTED_LOCATION "/usr/local/lib/std_vector_complement/libstd_vector_complement.so")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
       include/grasping_selection
        ${BOOST_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
        src/grasping_selection/core.cpp
        src/grasping_selection/core_setup_base.cpp
        src/grasping_selection/core_setup_from_json.cpp
        src/grasping_selection/loading_grasping_dataset_from_json.cpp
        src/grasping_selection/loading_grasping_selection_pipeline_from_json.cpp
        src/grasping_selection/heuristics/grasping_heuristics_base.cpp
        )

## Set dependencies of the library
target_link_libraries(${PROJECT_NAME}
       transform_manipulation
       std_vector_complement
       ${BOOST_LIBRARIES}
       ${CMAKE_THREAD_LIBS_INIT}
        ${PCL_LIBRARIES}
       )


# To export the lib and install it
set_target_properties(${PROJECT_NAME} PROPERTIES
       VERSION ${PROJECT_VERSION}
       PUBLIC_HEADER "include/grasping_selection/core.h;include/core_setup_base.h;include/loading_grasping_selection_pipeline_base;include/heuristics/grasping_heuristics_base.h;include/heuristics/grasping_heuristics_data;include/loading_grasping_dataset_base"

       )

target_include_directories(${PROJECT_NAME} PRIVATE .)

configure_file(install/${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)


if(${INSTALL_GRASPING_SELECTION_IN_ROOT})
   MESSAGE(">>>>> The grasping_selection is set to be installed in root. Use sudo to install." )
   install(TARGETS ${PROJECT_NAME}
           LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
           PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
           PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

   install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc
           DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)

else()
   MESSAGE(">>>>> The grasping_selection is set to be installed locally." )
   install(TARGETS ${PROJECT_NAME}
           LIBRARY DESTINATION ${CMAKE_BINARY_DIR}
           PUBLIC_HEADER DESTINATION ${CMAKE_BINARY_DIR}/public_headers)

   install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc
           DESTINATION ${CMAKE_BINARY_DIR}/pkgconfig)
endif()


################################################## UNIT Tests ##################################################
add_executable(test
       src/tests/test_01.cpp)

target_link_libraries(test
       ${PROJECT_NAME}
       )
