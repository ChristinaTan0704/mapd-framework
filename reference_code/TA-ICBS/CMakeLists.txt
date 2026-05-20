set(BOOST_COMPONENTS IOStreams Filesystem Regex Serialization Signals System Timer program_options)
FIND_PACKAGE( Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS} )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${CMAKE_CXX_FLAGS}")
aux_source_directory(. SRC)
set (CMAKE_CXX_STANDARD 11)
ADD_EXECUTABLE( driver driver.cpp ${SRC})

include_directories(${Boost_INCLUDE_DIRS}) 
TARGET_LINK_LIBRARIES( driver ${Boost_LIBRARIES} )