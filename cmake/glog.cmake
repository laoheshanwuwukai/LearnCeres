
find_package(glog )
if(glog_FOUND)
message(STATUS "glog found")
list(APPEND ALL_3DParty_LIB "glog::glog")
endif()
