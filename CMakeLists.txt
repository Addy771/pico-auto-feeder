cmake_minimum_required(VERSION 3.12)

set(TARGET_NAME auto-feeder)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(${TARGET_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
	message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()


# include(CheckIncludeFile)
# include(CheckIncludeFileCXX)
# include(CheckIncludeFiles)
# include(ExternalProject)

add_compile_options(-Wall
		-Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
		-Wno-unused-function # we have some for the docs that aren't called
		-Wno-maybe-uninitialized
		-fpermissive
		)
		
set(CMAKE_SYSTEM_NAME Generic)		

#include_directories("./bitmap")
# include_directories("./bldc")
# include_directories("./bldc/comm/")
# include_directories("./bldc/util/")
#include_directories("./bldc/tests/utils_math")	# ch.h needed
#include_directories("./bldc/ChibiOS_3.0.5/os/ext/CMSIS/ST/") #stm32f4xx.h needed
#include_directories("./bldc/ChibiOS_3.0.5/os/ext/CMSIS/include/") #core_cm4.h needed
#add_subdirectory(FatFS-SD/FatFs_SPI build)

add_subdirectory(../pico-stepper/lib stepper)



add_executable(${TARGET_NAME}
	auto-feeder.cpp
	)

pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/Button-debouncer/button_debounce.pio)
pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

target_sources(${TARGET_NAME} PRIVATE ./Button-debouncer/button_debounce.cpp)


#set_source_files_properties(crc.c PROPERTIES LANGUAGE CXX)
#set_source_files_properties(buffer.c PROPERTIES LANGUAGE CXX)
set_source_files_properties(stepper.c PROPERTIES LANGUAGE CXX)

set_target_properties(${TARGET_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_link_options(${TARGET_NAME} PRIVATE -Xlinker --print-memory-usage)

# pull in common dependencies
target_link_libraries(${TARGET_NAME} 
	pico_stdlib 
	#pico_multicore 
	#pico_sync
	#pico_malloc
	#hardware_i2c 
	#hardware_adc 
	#hardware_pwm 
	#hardware_uart 
	hardware_pio
	hardware_dma
	hardware_rtc
	#hardware_flash
	#FatFs_SPI
	pico-stepper
	pico_bootsel_via_double_reset
	)


# for double tap reset, define an LED to light up during bootloader mode
target_compile_definitions(${TARGET_NAME} PRIVATE
PICO_BOOTSEL_VIA_DOUBLE_RESET_ACTIVITY_LED=25
)


# enable usb output, disable uart output
pico_enable_stdio_usb(${TARGET_NAME} 1)
#pico_enable_stdio_uart(${TARGET_NAME} 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(${TARGET_NAME})

