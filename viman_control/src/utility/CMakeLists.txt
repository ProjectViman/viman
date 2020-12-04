set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(/usr/include/python3.6/)
include_directories(/usr/lib/x86_64-linux-gnu/)

add_executable(visualize_map visualize_map.cpp)
target_link_libraries(visualize_map python3.6m map)