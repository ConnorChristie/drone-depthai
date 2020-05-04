set (TARGET_NAME "ceSerial")

file (GLOB MAIN_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        )

file (GLOB LIBRARY_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/*.h
        )

add_definitions(-DWIN_EXPORT)

if(UNIX)
    list(REMOVE_ITEM MAIN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/dllmain.cpp)
endif()

# Create named folders for the sources within the .vcproj
# Empty name lists them directly under the .vcproj
source_group("src" FILES ${MAIN_SRC})
source_group("include" FILES ${LIBRARY_HEADERS})


# Create library file from sources.
add_library(${TARGET_NAME} SHARED ${MAIN_SRC} ${LIBRARY_HEADERS})
target_link_libraries(${TARGET_NAME})

if(CMAKE_VERSION VERSION_LESS "2.8.11")
	include_directories (${CMAKE_CURRENT_SOURCE_DIR})
else()
	target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
endif()

set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_PDB_NAME ${TARGET_NAME})
