set(APP_NAME cpp_drone_depthai)

set(dldt_dir 3rdparty/depthai-api/shared/3rdparty/dldt)

cmake_minimum_required(VERSION 2.8.12)

project(${APP_NAME})


add_definitions(-D__PC__)
add_definitions(-DUSE_USB_VSC) # for XLink communication
# add_definitions(-DXLINK_USB_DATA_TIMEOUT=0)
# add_definitions(-DXLINK_COMMON_TIMEOUT_MSEC=0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")


find_package(OpenCV REQUIRED )
# find_package(PythonLibs 3 REQUIRED)

include(${dldt_dir}/inference-engine/thirdparty/movidius/XLink/XLink.cmake)

include_directories(
    ./
    3rdparty/depthai-api/host/core/
    3rdparty/depthai-api/shared/
    ${XLINK_INCLUDE_DIRECTORIES}
)

add_executable( ${APP_NAME}
    main.cpp
    depthai_core.cpp
    msp.cpp
    pid.cpp
    web_server.cpp
    drone.cpp
    3rdparty/depthai-api/shared/logger/logs_writer.cpp
    3rdparty/depthai-api/host/core/3rdparty/loguru/loguru.cpp
    3rdparty/depthai-api/shared/general/data_writer.cpp
    3rdparty/depthai-api/shared/json_helper.cpp
    3rdparty/depthai-api/host/core/pipeline/host_pipeline.cpp
    3rdparty/depthai-api/host/core/pipeline/host_pipeline_config.cpp
    3rdparty/depthai-api/host/core/pipeline/cnn_host_pipeline.cpp
    3rdparty/depthai-api/shared/stream/stream_info.cpp
    3rdparty/depthai-api/shared/xlink/xlink_wrapper.cpp
    3rdparty/depthai-api/host/core/types.cpp
    3rdparty/depthai-api/host/core/host_json_helper.cpp
    3rdparty/depthai-api/host/core/host_data_reader.cpp
    3rdparty/depthai-api/host/core/host_data_reader.cpp
    3rdparty/depthai-api/host/core/host_capture_command.cpp
    3rdparty/depthai-api/host/core/device_support_listener.cpp
    3rdparty/depthai-api/host/core/disparity_stream_post_processor.cpp
    3rdparty/ceSerial/ceSerial.cpp
    ${XLINK_SOURCES}
)



# Find OpenCV components if exist
find_package(OpenCV COMPONENTS highgui QUIET)
if(NOT(OpenCV_FOUND))
    message(WARNING "OPENCV is disabled or not found, " ${APP_NAME} " skipped")
    return()
endif()



# nlohman JSON
set(nlohmann_json_DIR 3rdparty/depthai-api/shared/3rdparty/json)

include_directories(${nlohmann_json_DIR}/include/)
set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/${nlohmann_json_DIR} ${CMAKE_CURRENT_BINARY_DIR}/json)


# nlohman JSON validator
set(BUILD_TESTS OFF CACHE INTERNAL "")
set(BUILD_EXAMPLES OFF CACHE INTERNAL "")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/depthai-api/shared/3rdparty/json-schema-validator/ ${CMAKE_CURRENT_BINARY_DIR}/json-schema-validator)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/websocketpp/)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/websocketpp/ ${CMAKE_CURRENT_BINARY_DIR}/websocketpp)

include_directories(3rdparty/ceSerial/)


set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC_RUNTIME ON)
set(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED COMPONENTS atomic thread system)

# link libraries
target_link_libraries(
    ${APP_NAME}
    PRIVATE
    -lusb-1.0
    -lpthread
    -ldl
    nlohmann_json::nlohmann_json
    nlohmann_json_schema_validator
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
)
