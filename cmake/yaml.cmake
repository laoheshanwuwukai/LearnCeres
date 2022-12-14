find_package(yaml-cpp)
if(${yaml-cpp_FOUND})
    include_directories(${YAML_CPP_INCLUDE_DIR})

    list(APPEND ALL_3DParty_LIB "${YAML_CPP_LIBRARIES}")
endif()