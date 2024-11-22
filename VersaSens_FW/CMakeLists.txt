cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

add_definitions(-DFIXED_POINT)
add_definitions(-DVAR_ARRAYS)
# add_definitions(-DUSE_ALLOCA)
# add_definitions(-DHAVE_ALLOCA_H)
add_definitions(-DDISABLE_FLOAT_API)
add_definitions(-DOPUS_BUILD)
add_definitions(-DCONFIG_OPUS_MODE)
add_definitions(-DCONFIG_OPUS_MODE_CELT)
add_definitions(-DCONFIG_OPUS_MODE_SILK)
add_definitions(-DFF_USE_FASTSEEK=1)

add_definitions(-DUSER_APP_STACK_SIZE=1024)

include_directories(
    opus-1.2.1
    opus-1.2.1/arm
    src/drivers
    src/VersaAPI
    src/user_app/inc
)

project(VersaSens)

file(GLOB OPUS_SOURCES 
    "opus-1.2.1/*.c" 
    "opus-1.2.1/arm/*.c" 
)

file(GLOB DRIVERS 
    "src/drivers/*.c" 
)

file(GLOB API 
    "src/VersaAPI/*.c" 
)

file(GLOB USER_APP 
    "src/user_app/src/*.c" 
)

target_sources(app PRIVATE 
    src/testbench/test_led.c 
    ${DRIVERS}
    ${API}
    ${OPUS_SOURCES}
    ${USER_APP}
)
