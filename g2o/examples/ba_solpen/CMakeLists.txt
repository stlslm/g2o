set(CMAKE_BUILD_TYPE Debug) #Release

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(ba_solpen
  ba_solpen.cpp
)

target_link_libraries(ba_solpen core types_sba solver_dense)
if(CHOLMOD_FOUND)
  target_link_libraries(ba_solpen solver_cholmod  ${OpenCV_LIBS})
else()
  target_link_libraries(ba_solpen solver_eigen  ${OpenCV_LIBS})
endif()
