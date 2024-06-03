add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  basketball_motion_controller_msgs
  )
include_directories(
  ${catkin_INCLUDE_DIRS}
  )
    
rtmbuild_add_library(BasketballMotionController SHARED
  BasketballMotionController.cpp
  BasketballMotionControllerService_impl.cpp
  )
target_link_libraries(BasketballMotionController
  ${catkin_LIBRARIES}
  )
set_target_properties(BasketballMotionController PROPERTIES PREFIX "") # libBasketballMotionController.so -> BasketballMotionController.so                                                                           
add_dependencies(BasketballMotionController RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl                                                                                                        

rtmbuild_add_executable(BasketballMotionControllerComp BasketballMotionControllerComp.cpp)
target_link_libraries(BasketballMotionControllerComp BasketballMotionController)
