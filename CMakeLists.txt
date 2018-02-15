SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

project(ubitrack_device_camera_kinect2)

cmake_minimum_required(VERSION 3.4.0)

include(CheckIncludeFile)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(KEEP_RPATHS)

include(GNUInstallDirs)
include(cmake/UbitrackConfig)


# External Libraries (could this be solved with conan dependencies ??)

# OpenGL
SET(HAVE_OPENGL 0)
find_package(OpenGL)
IF(OpenGL_FOUND)
  set(UBITRACK_COMPILE_DEFINITIONS ${UBITRACK_COMPILE_DEFINITIONS} "HAVE_OPENGL")
  SET(HAVE_OPENGL 1)
  MESSAGE(STATUS "Found OpenGL: ${OpenGL_INCLUDE_DIR} - ${OpenGL_LIBRARIES}")
ENDIF(OpenGL_FOUND)

# OpenCL
SET(HAVE_OPENCL 0)
find_package(OpenCL)
IF(OpenCL_FOUND)
  set(UBITRACK_COMPILE_DEFINITIONS ${UBITRACK_COMPILE_DEFINITIONS} "HAVE_OPENCL")
  include_directories(${OpenCL_INCLUDE_DIR})
  SET(HAVE_OPENCL 1)
  MESSAGE(STATUS "Found OpenCL: ${OpenCL_INCLUDE_DIR} - ${OpenCL_LIBRARY}")
ENDIF(OpenCL_FOUND)

set(UBITRACK_EXTRA_LIBS ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${OpenCL_LIBRARY})

FIND_PACKAGE(KinectSDK2)
IF(KinectSDK2_FOUND)
    set(UBITRACK_COMPILE_DEFINITIONS ${UBITRACK_COMPILE_DEFINITIONS} "HAVE_KINECT2")
	set(UBITRACK_EXTRA_LIBS ${UBITRACK_EXTRA_LIBS} ${KinectSDK2_LIBRARIES})
	include_directories(${KinectSDK2_INCLUDE_DIRS})

	add_subdirectory(src/kinect)

	# collect all utql patterns from module for installation
	file(GLOB _utql_patterns "doc/utql/*.xml" "doc/utql/*/*.xml"  "doc/utql/*/*/*.xml")
	foreach(pfile ${_utql_patterns})
	  string(REGEX REPLACE "^.*/doc/utql/" "" pfile2 "${pfile}")
	  GET_FILENAME_COMPONENT(fpath ${pfile2} PATH)
	  IF(fpath)
		install(FILES ${pfile} DESTINATION "${UBITRACK_UTQLPATTERN_INSTALL_DIRECTORY}/${fpath}" COMPONENT data)
	  ELSE(fpath)
	    install(FILES ${pfile} DESTINATION "${UBITRACK_UTQLPATTERN_INSTALL_DIRECTORY}" COMPONENT data)
	  ENDIF(fpath)
	endforeach()
ENDIF(KinectSDK2_FOUND)
