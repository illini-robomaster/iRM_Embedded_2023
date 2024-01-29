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

project(Engineer ASM C CXX)

irm_add_arm_executable(Engineer_chassis
    TARGET DJI_Board_TypeC
    SOURCES
    src/main.cc
    src/chassis_task.cc
    src/engineer_steering.cc
    src/steering_6020.cc
    src/referee_task.cc
    src/selfcheck_task.cc
#     src/ui_task.cc
    INCLUDES include
)
