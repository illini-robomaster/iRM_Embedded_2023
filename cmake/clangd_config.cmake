option(
  IRM_GENERATE_CLANGD
  "Generate a local .clangd from the configured ARM toolchain during configure"
  ON
)

function(irm_generate_clangd)
  if(NOT IRM_GENERATE_CLANGD)
    return()
  endif()

  if(NOT CMAKE_C_COMPILER OR NOT CMAKE_CXX_COMPILER)
    message(STATUS "Skipping .clangd generation: C/C++ compilers are not configured yet.")
    return()
  endif()

  get_filename_component(IRM_CLANGD_C_COMPILER "${CMAKE_C_COMPILER}" REALPATH)
  get_filename_component(IRM_CLANGD_CXX_COMPILER "${CMAKE_CXX_COMPILER}" REALPATH)
  get_filename_component(IRM_CLANGD_CXX_BIN_DIR "${IRM_CLANGD_CXX_COMPILER}" DIRECTORY)
  get_filename_component(IRM_CLANGD_GCC_TOOLCHAIN_ROOT "${IRM_CLANGD_CXX_BIN_DIR}" DIRECTORY)

  configure_file(
    "${CMAKE_SOURCE_DIR}/.clangd.in"
    "${CMAKE_SOURCE_DIR}/.clangd"
    @ONLY
  )

  message(STATUS "Generated local clangd config: ${CMAKE_SOURCE_DIR}/.clangd")
endfunction()
