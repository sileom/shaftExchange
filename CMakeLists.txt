cmake_minimum_required(VERSION 3.0)

project(libfranka-uca_franka CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-variable -Wall")


find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OpenCV 4.2.0 REQUIRED )
find_package(realsense2 REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_library(TENSORFLOW_LIB tensorflow HINT /home/labarea-franka/tensorflow_cpp/lib)

add_library(common_uca_2 STATIC
  src/common_uca_2.cpp
  src/dynModel_uca_2.cpp
  src/Model_uca_2.cpp
  include/Model_uca_2.h
  src/Tensor_uca_2.cpp
  include/Tensor_uca_2.h
)

target_link_libraries(common_uca_2 PUBLIC Franka::Franka Eigen3::Eigen3)

set(UCA_FRANKA_2
  detection_scarico_franka
  graspScarico_franka
  detection_aspirazione_franka
  graspAspirazione_franka
  openGripper_franka
  moveCobot_franka
  moveCobot_franka_disAsp
  closeGripper_franka
  #cedente
  #moveCobot
  #graspAspirazione
  #graspScarico_franka_joint
  #graspScarico
)

set(OpenCV_LIBS /usr/local/lib/libopencv_dnn.so /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_core.so.4.2 /usr/local/lib/libopencv_imgproc.so.4.2 opencv_core opencv_highgui)

foreach(uca_franka ${UCA_FRANKA_2})
  add_executable(${uca_franka} ${uca_franka}.cpp include/utils_cam_uca_2.hpp include/utils_uca_2.hpp src/Model_uca_2.cpp src/Tensor_uca_2.cpp)
  target_include_directories(common_uca_2 PUBLIC include /home/labarea-franka/tensorflow_cpp/include/)
  target_link_libraries(${uca_franka} Franka::Franka common_uca_2 Eigen3::Eigen3)
  target_link_libraries(${uca_franka} "${TENSORFLOW_LIB}" ${OpenCV_LIBS} )
  target_link_libraries(${uca_franka} ${realsense2_LIBRARY})
  target_link_libraries(${uca_franka} Threads::Threads)

endforeach()


include(GNUInstallDirs)
install(TARGETS ${UCA_FRANKA_2}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

