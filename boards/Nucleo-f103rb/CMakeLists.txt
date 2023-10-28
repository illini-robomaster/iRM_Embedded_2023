# ---------------------------------------------------------------------- #
#                                                                        #
#  Copyright (C) 2023                                                    #
#  Illini RoboMaster @ University of Illinois at Urbana-Champaign.       #
#                                                                        #
#  This program is free software: you can redistribute it and/or modify  #
#  it under the terms of the GNU General Public License as published by  #
#  the Free Software Foundation, either version 3 of the License, or     #
#  (at your option) any later version.                                   #
#                                                                        #
#  This program is distributed in the hope that it will be useful,       #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
#  GNU General Public License for more details.                          #
#                                                                        #
#  You should have received a copy of the GNU General Public License     #
#  along with this program. If not, see <http://www.gnu.org/licenses/>.  #
#                                                                        #
# ---------------------------------------------------------------------- #

# Define the current directory name as the project name
get_filename_component(CURRENT_DIR_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${CURRENT_DIR_NAME} C ASM)

# MCU Configuration
set(MCU_LINE STM32F103xB)
set(MCU_ARCH cortex-m3)
set(MCU_LINKER_SCRIPT STM32F103RBTx_FLASH.ld)
set(MCU_OPTIONS -mcpu=${MCU_ARCH} -mthumb)

######################################################
# ----- Create board specific interface library -----#
######################################################
add_library(${PROJECT_NAME}_interface INTERFACE)

target_compile_definitions(${PROJECT_NAME}_interface INTERFACE
        ${MCU_LINE}
        )

target_compile_options(${PROJECT_NAME}_interface INTERFACE
        ${MCU_OPTIONS}
        -fdata-sections -ffunction-sections # To remove unused code/data
        -Wall # Enable all warnings
        )

target_link_options(${PROJECT_NAME}_interface INTERFACE
        ${MCU_OPTIONS}
        -specs=nano.specs
        -T${CMAKE_CURRENT_SOURCE_DIR}/${MCU_LINKER_SCRIPT}
        -Wl,-Map=${PROJECT_NAME}.map,--cref
        -Wl,--gc-sections
        )

target_link_libraries(${PROJECT_NAME}_interface INTERFACE board_interface)

target_include_directories(${PROJECT_NAME}_interface INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
        # Add other include directories as required
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F1xx/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3
        )

###################################################
# ----- create board specific driver library -----#
###################################################
file(GLOB_RECURSE CORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/*.c)
file(GLOB_RECURSE DRIVER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/*.c)
file(GLOB_RECURSE MIDDLEWARE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/*.c)
set(STARTUP_ASM ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f103xb.s)

# Combine sources and create a library
#add_library(${PROJECT_NAME} OBJECT
#        ${USER_SOURCES} ${HAL_SOURCES} ${STARTUP_ASM}
#        )
add_library(${PROJECT_NAME} OBJECT
        ${CORE_SOURCES} ${DRIVER_SOURCES} ${MIDDLEWARE_SOURCES} ${STARTUP_ASM})

target_link_libraries(${PROJECT_NAME}
        PRIVATE ${PROJECT_NAME}_interface
        )

# ST libraries throws those warnings
target_compile_options(${PROJECT_NAME} PRIVATE
        -Wno-maybe-uninitialized -Wno-unused-parameter -Wno-sign-compare -Wno-int-conversion)

## Create executable output
#add_executable(${PROJECT_NAME}.elf $<TARGET_OBJECTS:${PROJECT_NAME}>)
#set_target_properties(${PROJECT_NAME}.elf PROPERTIES LINKER_LANGUAGE C)
#target_link_libraries(${PROJECT_NAME}.elf PRIVATE ${PROJECT_NAME}_interface)

## Generate .bin and .hex from the .elf output
#add_custom_command(TARGET ${PROJECT_NAME}.elf
#        POST_BUILD
#        COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
#        COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
#        COMMENT "Generating ${PROJECT_NAME}.hex and ${PROJECT_NAME}.bin"
#        )
