cmake_minimum_required(VERSION 3.16)

project(cop_on_linux)

add_executable(${PROJECT_NAME} )
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    cop::cop
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

add_subdirectory(src)

add_subdirectory(../cop build)


