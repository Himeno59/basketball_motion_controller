add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  basketball_motion_controller_msgs)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS})

add_library(BasketballMotionControllerROSBridge SHARED BasketballMotionControllerROSBridge.cpp)
target_link_libraries(BasketballMotionControllerROSBridge
  ${catkin_LIBRARIES}
  ${openrtm_aist_LIBRARIES})
set_target_properties(BasketballMotionControllerROSBridge PROPERTIES PREFIX "")

add_executable(BasketballMotionControllerROSBridgeComp BasketballMotionControllerROSBridgeComp.cpp)
target_link_libraries(BasketballMotionControllerROSBridgeComp BasketballMotionControllerROSBridge)

install(TARGETS BasketballMotionControllerROSBridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS BasketballMotionControllerROSBridgeComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
