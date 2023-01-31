add_executable(pbrt_project
    core/geometry.cpp
    core/geometry.h
    core/ray.h
    core/ray.cpp
    core/transform.h
    core/transform.cpp)

target_include_directories(pbrt_project PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(pbrt_project glm)