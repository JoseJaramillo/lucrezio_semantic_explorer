add_library(semantic_explorer_library SHARED
  semantic_explorer.cpp semantic_explorer.h
)
add_definitions(-DPATH_TO_JSON="${CMAKE_SOURCE_DIR}/lucrezio_semantic_explorer/src/semantic_explorer/typical_object_sizes.json")

target_link_libraries(semantic_explorer_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
