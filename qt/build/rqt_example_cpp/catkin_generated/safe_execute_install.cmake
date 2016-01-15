execute_process(COMMAND "/home/biorobotics/Nico/daVinci/qt/build/rqt_example_cpp/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/biorobotics/Nico/daVinci/qt/build/rqt_example_cpp/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
