project(example_uart_mc02_comm_engineer ASM C CXX)

irm_add_arm_executable(${PROJECT_NAME}
    TARGET DM_MC_02
    SOURCES main.cc)
