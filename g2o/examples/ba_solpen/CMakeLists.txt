set(CMAKE_BUILD_TYPE Debug) #Release

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED COMPONENTS filesystem)
include_directories( ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

add_executable(ba_solpen
  ba_solpen.cpp
)

if(Boost_FOUND)
  message("LIBS: ${Boost_LIBRARIES}")
  message("include: ${Boost_INCLUDE_DIRS}")
endif()

target_link_libraries(ba_solpen core types_sba solver_dense)
if(CHOLMOD_FOUND)
  target_link_libraries(ba_solpen solver_cholmod  ${OpenCV_LIBS} ${Boost_LIBRARIES})
else()
  target_link_libraries(ba_solpen solver_eigen  ${OpenCV_LIBS} ${Boost_LIBRARIES})
endif()
