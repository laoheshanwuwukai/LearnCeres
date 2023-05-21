
find_package(OpenCV)

if(${OpenCV_FOUND})
    include_directories(${OpenCV_INCLUDE_DIRS})
    list(APPEND ALL_3DParty_LIB "${OpenCV_LIBS}")
    message(STATUS "Opencv Found , ${Opencv_INClUDE_DIRS}")
endif()