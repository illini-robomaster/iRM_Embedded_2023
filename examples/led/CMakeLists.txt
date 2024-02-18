project(example_led ASM C CXX)

irm_add_arm_executable(${PROJECT_NAME}_typeA
    TARGET DJI_Board_TypeA
    SOURCES typeA.cc)

irm_add_arm_executable(${PROJECT_NAME}_typeC
    TARGET DJI_Board_TypeC
    SOURCES typeC.cc)

irm_add_arm_executable(${PROJECT_NAME}_mc01
    TARGET DM_MC_01
    SOURCES mc01.cc)
