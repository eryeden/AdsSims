#For OpenGL testing

cmake_minimum_required(VERSION 3.5)
project(AdsSims)

#For auto-complete
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#output make commands
#set(CMAKE_VERBOSE_MAKEFILE ON)


#COMPILE COMMANDS configuration
set(CMAKE_C_COMPILER  /usr/local/bin/gcc-5.3)
set(CMAKE_CXX_COMPILER /usr/local/bin/g++-5.3)

#Libraries configurations
##############################################################
find_package(PkgConfig REQUIRED)

#For OpenGL libraries
#GLFW
pkg_search_module(GLFW REQUIRED glfw3)

#for OSX
find_package(GLEW REQUIRED)

#for framework
find_package(OpenGL REQUIRED)

#################################################################
#Libraries for GLFW
set(GLFW_STATIC_LIBS "-L/usr/local/lib -lglfw3 -framework Cocoa -framework IOKit -framework CoreFoundation -framework CoreVideo -framework Carbon")
#For glew
set(GLEW_LIBS "-lglew")
#For opengl
set(GL_PKGLIBS ${OPENGL_LIBRARIES})

SET(GL_LIBS ${GLFW_STATIC_LIBS} ${GLEW_LIBS} ${GL_PKGLIBS} pthread)
#################################################################

#################################################################
#FreeType
# True type font render
#find_package(Freetype REQUIRED)

set(FREETYPE_ROOT ${PROJECT_SOURCE_DIR}/Common/FreeType)
set(FREETYPE_INCLUDE ${FREETYPE_ROOT}/freetype)
set(FREETYPE_LIB ${FREETYPE_ROOT}/libfreetype.dylib) # Should be linked

#################################################################

#################################################################
#OpenGL
#SET(GL_INCLUDE ${PROJECT_SOURCE_DIR}/OpenGL/include)
SET(GL_INCLUDE ${GLFW_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS})

#Common libraries
SET(COMMON_INCLUDE ${PROJECT_SOURCE_DIR}/Common)

INCLUDE_DIRECTORIES(
        ${GL_INCLUDE}
        ${COMMON_INCLUDE}
        ${FREETYPE_INCLUDE}
)

message(${GL_INCLUDE})
#################################################################

#################################################################
# add Common sources
file(GLOB COMMON_SRCS ${PROJECT_SOURCE_DIR}/Common/*.cpp)
#################################################################

#For C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

#Directory configurations
add_definitions(-DSHADER_PREFIX="${PROJECT_SOURCE_DIR}/Common/GLSL/")
add_definitions(-DFONTS_DIR_PREFIX="${PROJECT_SOURCE_DIR}/Fonts/")
add_definitions(-DIMAGE_DIR="${PROJECT_SOURCE_DIR}/MovieDir/")
add_definitions(-DPROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}/")


#Subdirectories
add_subdirectory(ActiveRotator)
add_subdirectory(CoupledOscillator)
add_subdirectory(OV)
add_subdirectory(Earthworm)

#Delete All PPM files
add_custom_target(cleanMD rm -rf ${PROJECT_SOURCE_DIR}/MovieDir/*ppm)



