#Top Level of project

#Has to be the first line of CMakeLists.txt , if not cmake gona show should be added at the top of the file
cmake_minimum_required(VERSION 3.10) 

project(LearnCeres)

#set CMAKE var
set(CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE debug)
endif(NOT CMAKE_BUILD_TYPE)
#end setting CMAKE vars

message(STATUS "*************${PROJECT_NAME} Message: ***************")

#Package Managment

set(ALL_3DParty_LIB "")

include(cmake/Eigen3.cmake)
include(cmake/ceres.cmake)
include(cmake/glog.cmake)
include(cmake/Sophus.cmake)
include(cmake/opencv.cmake)
# include(cmake/global_difination.cmake)
include(cmake/yaml.cmake)
#include dir
include_directories(${PROJECT_SOURCE_DIR}/include)


#TODO change the name of xxx.cpp
add_executable( ${PROJECT_NAME} 
                testmain.cpp
                # main.cpp 
                # src/Auto_Derivatives.cpp
                # src/Numeric_Derivatives.cpp
                # src/Analytic_Derivatives.cpp
                # src/PowellFunction.cpp
                # src/CurveFitting.cpp
                )


#!!Since v2.0 ,Ceres has used the target namespace feature of CMake to prefix its 
#export targets:Ceres::ceres.
#So target_link_libraries(LearnCeres Ceres::ceres) isn't usefull;
#However , historically the Ceres did not have a namespcae and was just called ceres
target_link_libraries(${PROJECT_NAME} ${ALL_3DParty_LIB} )

