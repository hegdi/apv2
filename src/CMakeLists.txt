add_executable(${PROJECT_NAME} main.cpp APV2.cpp matchfinder.c humanize_number.c progress_bar.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_BINARY_DIR}")
target_link_libraries(${PROJECT_NAME} libesa_matchfinder)
target_link_libraries(${PROJECT_NAME} Boost::headers)
