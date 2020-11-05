add_executable(mapping map_storage_test.cpp)
target_link_libraries(mapping map)
set_target_properties(mapping 
	PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY
	${CMAKE_CURRENT_SOURCE_DIR}/bin
)