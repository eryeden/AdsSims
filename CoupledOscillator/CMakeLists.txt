#For OpenGL testing

cmake_minimum_required(VERSION 3.5)
set(BN "CoupledOscillator")

set(SOURCE_FILES main.cpp ${COMMON_SRCS})
add_executable(${BN} ${SOURCE_FILES})

target_link_libraries(${BN} ${GL_LIBS} ${FREETYPE_LIB})



