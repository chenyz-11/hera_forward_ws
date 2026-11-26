cmake_minimum_required(VERSION 3.10)
project(SigmaSdkDemo)
set(CMAKE_CXX_STANDARD 11)

add_executable(SigmaSdkDemo test.cpp sigmaSDK.cpp)
target_link_libraries(SigmaSdkDemo pthread)
