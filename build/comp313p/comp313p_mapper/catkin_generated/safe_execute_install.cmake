execute_process(COMMAND "/home/tahlia/cw2_redo/build/comp313p/comp313p_mapper/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/tahlia/cw2_redo/build/comp313p/comp313p_mapper/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
