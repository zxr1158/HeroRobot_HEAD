# Board (dm-h723)

file(GLOB_RECURSE BOARD_SOURCES CONFIGURE_DEPENDS
    "Core/Src/*.c" "Core/Src/*.cpp" "Core/Src/*.s" "Core/Src/*.S"
    "Port/*.c" "Port/*.cpp" "Port/*.s" "Port/*.S"
    "Core/Startup/startup_stm32f407ighx.s" "Core/Startup/startup_stm32f407ighx.S"
    "Drivers/STM32F4xx_HAL_Driver/Src/*.c"
    "USB_DEVICE/*.c"
    "Middlewares/*.c"
    "USB_DEVICE/*.s"
    "Middlewares/*.s"
    "USB_DEVICE/*.S"
    "Middlewares/*.S"
)

list(LENGTH BOARD_SOURCES BOARD_SOURCES_COUNT)
if(BOARD_SOURCES_COUNT EQUAL 0)
    add_library(dust_board INTERFACE)
    target_link_libraries(dust_board INTERFACE dust_options)
else()
    add_library(dust_board OBJECT)
    target_sources(dust_board PRIVATE ${BOARD_SOURCES})
    target_link_libraries(dust_board PRIVATE dust_options)
endif()
