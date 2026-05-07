cmake_minimum_required(VERSION 3.8)
project(serial)

if(UNIX)
    # If Linux, add rt and pthread
    set(rt_LIBRARIES rt)
    set(pthread_LIBRARIES pthread)
endif()

## Sources
set(serial_SRCS
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)

if(UNIX)
    # If unix
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)
else()
    # If windows
    list(APPEND serial_SRCS src/impl/win.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_win.cc)
endif()

## Add serial library
add_library(${PROJECT_NAME} STATIC ${serial_SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC "include")
#if(UNIX)
#    target_link_libraries(${PROJECT_NAME} rt pthread)
#else()
#    target_link_libraries(${PROJECT_NAME} setupapi)
#endif()

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    # If Linux, add pthread
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    target_link_libraries(${PROJECT_NAME} pthread)
else()
    target_link_libraries(${PROJECT_NAME} setupapi)
endif()

## Include headers
include_directories(include)