# ---------------------------------------------------------------------- #
#                                                                        #
#  Copyright (C) 2025                                                    #
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

project(vehicle_engineer2026 ASM C CXX)

# Steering6020 is a vehicle-specific class from the Engineer chassis.
# We pull in just its source file and expose its include directory.
irm_add_arm_executable(${PROJECT_NAME}_mc02
    TARGET DM_MC_02
    SOURCES
        main_mc02.cc
        arm_mc02.cc
        uart_framing.cc
        ${CMAKE_SOURCE_DIR}/vehicles/Engineer/chassis/src/steering_6020.cc
    INCLUDES
        ${CMAKE_SOURCE_DIR}/vehicles/Engineer/chassis/include)
