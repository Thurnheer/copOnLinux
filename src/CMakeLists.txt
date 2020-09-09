cmake_minimum_required( VERSION 3.16)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)

target_sources(${PROJECT_NAME}
    PRIVATE
    main.cpp
)
