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
#CLIONに付属しているCMAKEは"/usr/local/bin"までパスが通っていないため、g++-5だけでは、コマンドのドライブができない。なので、リンクの場所までパスを張る。
#set(CMAKE_C_COMPILER  gcc-5)
#set(CMAKE_CXX_COMPILER g++-5)

#Libraries configurations
##############################################################
find_package(PkgConfig REQUIRED)

#For OpenGL libraries
#GLFW
pkg_search_module(GLFW REQUIRED glfw3)
#assimp
find_package(ASSIMP REQUIRED)

#for OSX
find_package(GLEW REQUIRED)

#for framework
find_package(OpenGL REQUIRED)
#find_package(GLUT REQUIRED)

#################################################################
#Libraries for GLFW
set(GLFW_STATIC_LIBS "-L/usr/local/lib -lglfw3 -framework Cocoa -framework IOKit -framework CoreFoundation -framework CoreVideo -framework Carbon")
#set(GLFW_STATIC_LIBS ${GLFW_LIBRARIES})
#For assimps
set(ASSIMPS_LIBS "-L/usr/local/lib -lassimp -lz")
#For glew
set(GLEW_LIBS "-lglew")
#For glut
#set(GL_LIBS "-I/usr/X11R6/include -L/usr/X11R6/lib -framework OpenGL -framework GLUT -framework Foundation")
set(GL_PKGLIBS ${OPENGL_LIBRARIES})

#SET(GL_LIBS GL ${GLFW_STATIC_LIBRARIES} ${ASSIMP_LIBRARIES} ${GLEW_LIBRARIES})
#SET(GL_LIBS ${GLFW_STATIC_LIBS} ${ASSIMP_LIBRARIES} ${GLEW_LIBRARIES})
#SET(GL_LIBS ${GLFW_STATIC_LIBS} ${ASSIMPS_LIBS} ${GLEW_LIBS} ${GL_LIBS})
SET(GL_LIBS ${GLFW_STATIC_LIBS} ${ASSIMPS_LIBS} ${GLEW_LIBS} ${GL_PKGLIBS})
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
SET(GL_INCLUDE ${GLFW_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} ${ASSIMP_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS})

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
file(GLOB COMMON_SRCS ${PROJECT_SOURCE_DIR}/Common/*.cpp)
#################################################################

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

#Directory configurations
add_definitions(-DSHADER_PREFIX="${PROJECT_SOURCE_DIR}/Common/GLSL/")
add_definitions(-DFONTS_DIR_PREFIX="${PROJECT_SOURCE_DIR}/Fonts/")
add_definitions(-DIMAGE_DIR="${PROJECT_SOURCE_DIR}/MovieDir/")

#Subdirectories
add_subdirectory(ActiveRotator)
add_subdirectory(CoupledOscillator)
add_subdirectory(OV)





