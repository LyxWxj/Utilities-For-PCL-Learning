cmake_minimum_required(VERSION 3.10)
project(3DPointCloudMatch)



add_executable(${PROJECT_NAME} main.cpp )

enable_language(OpenMP)


set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)


target_sources(3DPointCloudMatch PUBLIC ${SRC_DIR}/pcl_utility.cpp)
target_sources(3DPointCloudMatch PUBLIC ${SRC_DIR}/random_engine.cpp)
target_sources(3DPointCloudMatch PUBLIC ${SRC_DIR}/registrator.cpp)
target_sources(3DPointCloudMatch PUBLIC ${SRC_DIR}/goicp_impl/ConfigMap.cpp)
target_sources(3DPointCloudMatch PUBLIC ${SRC_DIR}/goicp_impl/jly_3ddt.cpp)
target_sources(3DPointCloudMatch PUBLIC ${SRC_DIR}/goicp_impl/jly_goicp.cpp)
target_sources(3DPointCloudMatch PUBLIC ${SRC_DIR}/goicp_impl/matrix.cpp)
target_sources(3DPointCloudMatch PUBLIC ${SRC_DIR}/goicp_impl/StringTokenizer.cpp)

include_directories(
	${INCLUDE_DIR}
    	${INCLUDE_DIR}/gr
	${INCLUDE_DIR}/goicp
)


if( NOT EIGEN3_INCLUDE_DIR )
    find_package(Eigen3 QUIET)
    if( (NOT Eigen3_FOUND) OR EIGEN_VERSION_NUMBER VERSION_LESS 3.3 )

        set(EIGEN3_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/3rdparty/Eigen")

        if( NOT EXISTS ${EIGEN3_INCLUDE_DIR}/signature_of_eigen3_matrix_library )
            execute_process(COMMAND git submodule update --init -- ${EIGEN3_INCLUDE_DIR}
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
        endif( NOT EXISTS ${EIGEN3_INCLUDE_DIR}/signature_of_eigen3_matrix_library )
    endif()
endif( NOT EIGEN3_INCLUDE_DIR )
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "Eigen3 root path: ${EIGEN3_INCLUDE_DIR}")

find_package(PCL 1.1 REQUIRED)  
  
if(PCL_FOUND)  
    include_directories(${PCL_INCLUDE_DIRS})  

    link_directories(${PCL_LIBRARY_DIRS})  
      
    target_link_libraries(3DPointCloudMatch ${PCL_LIBRARIES})  
else()  
    message(FATAL_ERROR "PCL library not found!")  
endif()

